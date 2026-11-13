#include "amr_reobservation/scan_preprocessor.hpp"
#include "amr_reobservation/vehicle_side_estimator.hpp"
#include "amr_reobservation/msg/vehicle_side_estimate.hpp"
#include "amr_reobservation/msg/vehicle_side_fit_diagnostics.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <memory>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace amr_reobservation
{
namespace
{
using Marker = visualization_msgs::msg::Marker;
using EstimateMessage = amr_reobservation::msg::VehicleSideEstimate;
using DiagnosticsMessage = amr_reobservation::msg::VehicleSideFitDiagnostics;
constexpr double kPi = 3.14159265358979323846;

geometry_msgs::msg::Point point_message(const std::array<double, 2> & point)
{
  geometry_msgs::msg::Point output;
  output.x = point[0];
  output.y = point[1];
  return output;
}

geometry_msgs::msg::Vector3 vector_message(const std::array<double, 2> & vector)
{
  geometry_msgs::msg::Vector3 output;
  output.x = vector[0];
  output.y = vector[1];
  return output;
}

std_msgs::msg::ColorRGBA color(float r, float g, float b)
{
  std_msgs::msg::ColorRGBA output;
  output.r = r;
  output.g = g;
  output.b = b;
  output.a = 1.0F;
  return output;
}
}  // namespace

class VehicleSideEstimatorNode : public rclcpp::Node
{
public:
  VehicleSideEstimatorNode()
  : Node("vehicle_side_estimator")
  {
    const auto input_topic = parameter<std::string>("input_topic", "/observation_lidar/scan");
    const auto estimate_topic = parameter<std::string>("estimate_topic", "/vehicle_side/estimate");
    const auto marker_topic = parameter<std::string>("marker_topic", "/debug/vehicle_side");
    const auto diagnostics_topic = parameter<std::string>(
      "fit_diagnostics_topic", "/debug/vehicle_side/fit_diagnostics");
    PreprocessConfig pre;
    pre.range_min_m = parameter("range_min_m", 0.1);
    pre.range_max_m = parameter("range_max_m", 6.0);
    pre.roi_min_deg = parameter("roi_min_deg", -180.0);
    pre.roi_max_deg = parameter("roi_max_deg", 180.0);
    preprocessor_ = ScanPreprocessor(pre);

    SideEstimatorConfig config;
    config.cluster_gap_m = parameter("cluster_gap_m", 0.15);
    config.max_missing_beams = count_parameter("max_missing_beams", 2, 0, 100);
    config.min_cluster_points = count_parameter("min_cluster_points", 20, 2, 100000);
    config.ransac_iterations = count_parameter("ransac_iterations", 100, 1, 10000);
    config.refinement_limit = count_parameter("refinement_limit", 20, 1, 100);
    config.ransac_seed = static_cast<std::uint32_t>(count_parameter("ransac_seed", 7, 0, 2147483647));
    config.inlier_distance_m = parameter("inlier_distance_m", 0.03);
    config.min_inliers = count_parameter("min_inliers", 20, 2, 100000);
    config.min_inlier_ratio = parameter("min_inlier_ratio", 0.5);
    config.min_observed_length_m = parameter("min_observed_length_m", 0.5);
    config.max_rmse_m = parameter("max_rmse_m", 0.02);
    estimator_ = VehicleSideEstimator(config);
    marker_lifetime_sec_ = parameter("marker_lifetime_sec", 1.0);
    if (!std::isfinite(marker_lifetime_sec_) || marker_lifetime_sec_ <= 0.0 ||
      marker_lifetime_sec_ > 60.0)
    {
      throw std::invalid_argument("Require 0 < marker_lifetime_sec <= 60.");
    }

    estimate_pub_ = create_publisher<EstimateMessage>(estimate_topic, rclcpp::QoS(1));
    diagnostics_pub_ = create_publisher<DiagnosticsMessage>(diagnostics_topic, rclcpp::QoS(10));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, rclcpp::QoS(1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {on_scan(*scan);});
    RCLCPP_INFO(get_logger(), "Listening on %s -> %s and %s",
      input_topic.c_str(), estimate_topic.c_str(), marker_topic.c_str());
    RCLCPP_INFO(get_logger(), "Per-scan fit diagnostics: %s", diagnostics_topic.c_str());
    RCLCPP_INFO(get_logger(), "PCA refinement limit: %zu (stops early on convergence)",
      config.refinement_limit);
  }

private:
  template<typename T>
  T parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Startup parameter; edit YAML and restart to apply.";
    return declare_parameter<T>(name, value, descriptor);
  }

  std::size_t count_parameter(
    const std::string & name, std::int64_t value, std::int64_t minimum, std::int64_t maximum)
  {
    const auto result = parameter<std::int64_t>(name, value);
    if (result < minimum || result > maximum) {
      throw std::invalid_argument(name + " must be in [" + std::to_string(minimum) +
              ", " + std::to_string(maximum) + "].");
    }
    return static_cast<std::size_t>(result);
  }

  Marker marker_base(
    const std_msgs::msg::Header & header, const std::string & ns, std::int32_t type) const
  {
    Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = 0;
    marker.type = type;
    marker.action = Marker::DELETE;  // Every scan explicitly replaces/deletes each display.
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.025;
    marker.scale.y = 0.025;
    marker.color = color(1.0F, 1.0F, 1.0F);
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
    return marker;
  }

  void publish_markers(
    const std_msgs::msg::Header & source_header, const std::vector<ScanPoint> & points,
    const SideEstimationResult & result)
  {
    auto header = source_header;
    if (header.frame_id.empty()) {
      if (last_marker_frame_.empty()) {
        return;
      }
      header.frame_id = last_marker_frame_;
      header.stamp.sec = 0;
      header.stamp.nanosec = 0;
    } else {
      last_marker_frame_ = header.frame_id;
    }

    auto clusters = marker_base(header, "clusters", Marker::POINTS);
    auto inliers = marker_base(header, "inliers", Marker::POINTS);
    auto line = marker_base(header, "side_line", Marker::LINE_LIST);
    auto tangent = marker_base(header, "tangent", Marker::ARROW);
    auto normal = marker_base(header, "normal", Marker::ARROW);
    auto summary = marker_base(header, "summary", Marker::TEXT_VIEW_FACING);
    const std::array<std_msgs::msg::ColorRGBA, 6> palette{
      color(0.9F, 0.35F, 0.9F), color(0.4F, 0.9F, 0.3F), color(1.0F, 0.6F, 0.3F),
      color(0.5F, 0.6F, 1.0F), color(0.8F, 0.8F, 0.4F), color(0.6F, 0.9F, 0.8F)};
    for (std::size_t c = 0; c < result.clusters.size(); ++c) {
      for (const auto index : result.clusters[c]) {
        clusters.points.push_back(point_message({points[index].x, points[index].y}));
        clusters.colors.push_back(palette[c % palette.size()]);
      }
    }
    if (!clusters.points.empty()) {
      clusters.action = Marker::ADD;
    }

    if (result.estimate) {
      const auto & estimate = *result.estimate;
      inliers.action = Marker::ADD;
      inliers.scale.x = inliers.scale.y = 0.04;
      inliers.color = color(0.0F, 1.0F, 1.0F);
      for (const auto index : estimate.inlier_indices) {
        inliers.points.push_back(point_message({points[index].x, points[index].y}));
      }
      line.action = Marker::ADD;
      line.scale.x = 0.025;
      line.color = color(1.0F, 0.9F, 0.0F);
      line.points = {point_message(estimate.endpoint_a), point_message(estimate.endpoint_b)};
      for (auto * arrow : {&tangent, &normal}) {
        arrow->action = Marker::ADD;
        arrow->scale.x = 0.025;
        arrow->scale.y = 0.075;
        arrow->scale.z = 0.10;
      }
      const auto & center = estimate.center;
      tangent.color = color(0.25F, 0.5F, 1.0F);
      tangent.points = {point_message(center), point_message({
          center[0] + 0.6 * estimate.tangent[0], center[1] + 0.6 * estimate.tangent[1]})};
      normal.color = color(1.0F, 0.4F, 0.1F);
      normal.points = {point_message(center), point_message({
          center[0] + 0.5 * estimate.normal_toward_sensor[0],
          center[1] + 0.5 * estimate.normal_toward_sensor[1]})};
      summary.action = Marker::ADD;
      summary.pose.position = point_message(center);
      summary.pose.position.z = 0.2;
      summary.scale.z = 0.15;
      std::ostringstream text;
      text << std::fixed << std::setprecision(2) << "span " << estimate.observed_length_m
           << " m | RMSE " << estimate.rmse_m * 1000.0 << " mm\ninliers "
           << estimate.inlier_indices.size() << " (" << estimate.inlier_ratio * 100.0 << "%)";
      summary.text = text.str();
    }
    visualization_msgs::msg::MarkerArray output;
    output.markers = {clusters, inliers, line, tangent, normal, summary};
    marker_pub_->publish(output);
  }

  void publish_diagnostics(
    const EstimateMessage & estimate, const SideEstimationResult & result,
    double processing_time_ms)
  {
    DiagnosticsMessage output;
    output.header = estimate.header;
    output.valid = estimate.valid;
    output.status = estimate.status;
    output.input_point_count = estimate.input_point_count;
    output.cluster_count = estimate.cluster_count;
    output.selected_cluster_index = result.estimate ?
      static_cast<std::int32_t>(result.estimate->cluster_index) : -1;
    output.processing_time_ms = processing_time_ms;
    output.clusters.reserve(result.diagnostics.size());
    for (const auto & diagnostic : result.diagnostics) {
      amr_reobservation::msg::ClusterFitDiagnostics cluster;
      cluster.cluster_index = static_cast<std::uint32_t>(diagnostic.cluster_index);
      cluster.point_count = static_cast<std::uint32_t>(diagnostic.point_count);
      cluster.reason = fit_reason_name(diagnostic.reason);
      cluster.selected = result.estimate &&
        diagnostic.cluster_index == result.estimate->cluster_index;
      cluster.ransac_iterations = static_cast<std::uint32_t>(diagnostic.ransac_iterations);
      cluster.refinement_iterations = static_cast<std::uint32_t>(diagnostic.refinement_iterations);
      cluster.refinement_limit = static_cast<std::uint32_t>(diagnostic.refinement_limit);
      cluster.converged = diagnostic.converged;
      cluster.fit_inlier_count = static_cast<std::uint32_t>(diagnostic.fit_inlier_count);
      cluster.inlier_count = static_cast<std::uint32_t>(diagnostic.inlier_count);
      cluster.inlier_ratio = diagnostic.inlier_ratio;
      cluster.reclassification_available = diagnostic.reclassification_available;
      cluster.inlier_change_count = static_cast<std::uint32_t>(diagnostic.inlier_change_count);
      cluster.metrics_available = diagnostic.metrics_available;
      cluster.observed_length_m = diagnostic.observed_length_m;
      cluster.rmse_m = diagnostic.rmse_m;
      output.clusters.push_back(std::move(cluster));
    }
    diagnostics_pub_->publish(output);

    // Terminal output is only a summary. The diagnostic topic carries every
    // processed scan, independent of this steady-time log window.
    ++summary_scans_;
    summary_valid_ += output.valid ? 1 : 0;
    ++summary_statuses_[output.status];
    summary_processing_sum_ms_ += processing_time_ms;
    summary_processing_max_ms_ = std::max(summary_processing_max_ms_, processing_time_ms);
    for (const auto & cluster : output.clusters) {
      ++summary_reasons_[cluster.reason];
      summary_refinement_max_ = std::max(summary_refinement_max_, cluster.refinement_iterations);
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - summary_started_ >= std::chrono::seconds(5)) {
      std::ostringstream statuses, reasons;
      for (const auto & item : summary_statuses_) {
        statuses << item.first << '=' << item.second << ' ';
      }
      for (const auto & item : summary_reasons_) {
        reasons << item.first << '=' << item.second << ' ';
      }
      RCLCPP_INFO(get_logger(),
        "Fit window: scans=%zu valid=%zu; frame_status={%s}; cluster_reason={%s}; "
        "PCA max=%u; compute mean/max=%.3f/%.3f ms",
        summary_scans_, summary_valid_, statuses.str().c_str(), reasons.str().c_str(),
        summary_refinement_max_, summary_processing_sum_ms_ / summary_scans_,
        summary_processing_max_ms_);
      summary_started_ = now;
      summary_scans_ = summary_valid_ = 0;
      summary_statuses_.clear();
      summary_reasons_.clear();
      summary_processing_sum_ms_ = summary_processing_max_ms_ = 0.0;
      summary_refinement_max_ = 0;
    }
  }

  void on_scan(const sensor_msgs::msg::LaserScan & scan)
  {
    const auto processing_started = std::chrono::steady_clock::now();
    const ScanGeometry geometry{scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max};
    const auto preprocessed = preprocessor_.process(scan.ranges, geometry);
    SideEstimationResult result;
    std::string status;
    if (scan.header.frame_id.empty() || preprocessed.status == ScanStatus::InvalidMetadata) {
      result.status = SideStatus::InvalidInput;
      status = scan.header.frame_id.empty() ? "empty_frame" : "invalid_scan_metadata";
    } else {
      result = estimator_.estimate(preprocessed.points, geometry, scan.ranges.size());
      status = side_status_name(result.status);
    }
    const double processing_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - processing_started).count();

    EstimateMessage output;
    output.header = scan.header;
    output.valid = result.estimate.has_value();
    output.status = status;
    output.input_point_count = static_cast<std::uint32_t>(preprocessed.points.size());
    output.cluster_count = static_cast<std::uint32_t>(result.clusters.size());
    if (result.estimate) {
      const auto & estimate = *result.estimate;
      output.center = point_message(estimate.center);
      output.endpoint_a = point_message(estimate.endpoint_a);
      output.endpoint_b = point_message(estimate.endpoint_b);
      output.tangent = vector_message(estimate.tangent);
      output.normal_toward_sensor = vector_message(estimate.normal_toward_sensor);
      output.observed_length_m = estimate.observed_length_m;
      output.rmse_m = estimate.rmse_m;
      output.inlier_ratio = estimate.inlier_ratio;
      output.inlier_count = static_cast<std::uint32_t>(estimate.inlier_indices.size());
      RCLCPP_INFO_THROTTLE(get_logger(), steady_clock_, 5000,
        "valid: span=%.3f m, axis=%.1f deg (mod 180), RMSE=%.4f m, inliers=%u/%zu, clusters=%u",
        output.observed_length_m, std::atan2(estimate.tangent[1], estimate.tangent[0]) * 180.0 / kPi,
        output.rmse_m, output.inlier_count, result.clusters[estimate.cluster_index].size(),
        output.cluster_count);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000,
        "No valid side: %s; points=%u, clusters=%u", status.c_str(),
        output.input_point_count, output.cluster_count);
    }
    estimate_pub_->publish(output);
    publish_diagnostics(output, result, processing_time_ms);
    publish_markers(scan.header, preprocessed.points, result);
  }

  ScanPreprocessor preprocessor_;
  VehicleSideEstimator estimator_;
  double marker_lifetime_sec_{1.0};
  std::string last_marker_frame_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<EstimateMessage>::SharedPtr estimate_pub_;
  rclcpp::Publisher<DiagnosticsMessage>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::chrono::steady_clock::time_point summary_started_{std::chrono::steady_clock::now()};
  std::size_t summary_scans_{0}, summary_valid_{0};
  std::map<std::string, std::size_t> summary_statuses_, summary_reasons_;
  std::uint32_t summary_refinement_max_{0};
  double summary_processing_sum_ms_{0.0}, summary_processing_max_ms_{0.0};
};
}  // namespace amr_reobservation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<amr_reobservation::VehicleSideEstimatorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("vehicle_side_estimator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
