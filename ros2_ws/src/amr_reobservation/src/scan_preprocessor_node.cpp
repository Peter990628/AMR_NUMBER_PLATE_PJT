#include "amr_reobservation/scan_preprocessor.hpp"

#include <cmath>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace amr_reobservation
{

class ScanPreprocessorNode : public rclcpp::Node
{
public:
  ScanPreprocessorNode()
  : Node("scan_preprocessor")
  {
    const auto input_topic = startup_parameter<std::string>(
      "input_topic", "/observation_lidar/scan", "Input LaserScan topic.");
    const auto marker_topic = startup_parameter<std::string>(
      "marker_topic", "/debug/observation_points", "Output visualization_msgs/Marker topic.");
    PreprocessConfig config;
    config.range_min_m = startup_parameter("range_min_m", 0.1, "Minimum accepted distance in meters.");
    config.range_max_m = startup_parameter("range_max_m", 6.0, "Maximum accepted distance in meters.");
    config.roi_min_deg = startup_parameter("roi_min_deg", -180.0, "ROI start in sensor-frame degrees.");
    config.roi_max_deg = startup_parameter("roi_max_deg", 180.0, "ROI end; min > max wraps past 180.");
    processor_ = ScanPreprocessor(config);
    marker_size_m_ = startup_parameter("marker_size_m", 0.035, "RViz point width/height in meters.");
    marker_lifetime_sec_ = startup_parameter("marker_lifetime_sec", 1.0, "Marker lifetime in ROS seconds.");
    if (!std::isfinite(marker_size_m_) || marker_size_m_ <= 0.0 ||
      !std::isfinite(marker_lifetime_sec_) || marker_lifetime_sec_ <= 0.0 ||
      marker_lifetime_sec_ > 60.0)
    {
      throw std::invalid_argument("Require positive marker size and 0 < marker_lifetime_sec <= 60.");
    }
    // Reliable output works with RViz's default Marker subscription.
    publisher_ = create_publisher<visualization_msgs::msg::Marker>(marker_topic, rclcpp::QoS(1));
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {on_scan(*scan);});
    RCLCPP_INFO(
      get_logger(), "Listening on %s -> %s; range [%.2f, %.2f] m, ROI [%.1f, %.1f] deg",
      input_topic.c_str(), marker_topic.c_str(), config.range_min_m, config.range_max_m,
      config.roi_min_deg, config.roi_max_deg);
  }

private:
  template<typename T>
  T startup_parameter(const std::string & name, const T & value, const std::string & description)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = description + " Set before launch; restart to apply changes.";
    return declare_parameter<T>(name, value, descriptor);
  }

  void on_scan(const sensor_msgs::msg::LaserScan & scan)
  {
    const auto result = processor_.process(
      scan.ranges, {scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max});
    visualization_msgs::msg::Marker marker;
    marker.header = scan.header;  // Keep acquisition time and sensor frame; no invented TF.
    marker.ns = "observation_points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_size_m_;
    marker.scale.y = marker_size_m_;
    marker.color.r = 0.1F;
    marker.color.g = 1.0F;
    marker.color.b = 0.2F;
    marker.color.a = 1.0F;
    marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);

    if (scan.header.frame_id.empty() || result.status == ScanStatus::InvalidMetadata) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
      RCLCPP_WARN_THROTTLE(
        get_logger(), steady_clock_, 5000, "Invalid scan metadata or empty frame_id; clearing points.");
    } else if (result.points.empty()) {
      marker.action = visualization_msgs::msg::Marker::DELETE;
    } else {
      marker.action = visualization_msgs::msg::Marker::ADD;
      last_marker_frame_ = scan.header.frame_id;
      marker.points.reserve(result.points.size());
      for (const auto & point : result.points) {
        geometry_msgs::msg::Point output;
        output.x = point.x;
        output.y = point.y;
        output.z = 0.0;  // Points are on the scan plane in the LiDAR frame.
        marker.points.push_back(output);
      }
    }
    if (marker.action == visualization_msgs::msg::Marker::DELETE && marker.header.frame_id.empty()) {
      // RViz filters even DELETE messages through TF. An empty frame is dropped.
      if (last_marker_frame_.empty()) {
        return;  // This node has not displayed any points yet.
      }
      marker.header.frame_id = last_marker_frame_;
      marker.header.stamp.sec = 0;  // Latest TF suffices to remove the old marker.
      marker.header.stamp.nanosec = 0;
    }
    publisher_->publish(marker);
    if (result.status != ScanStatus::InvalidMetadata && !scan.header.frame_id.empty()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), steady_clock_, 5000,
        "scan=%zu kept=%zu invalid_range=%zu outside_distance=%zu outside_angle=%zu frame=%s",
        result.input_count, result.points.size(), result.invalid_range_count,
        result.outside_distance_count, result.outside_angle_count, scan.header.frame_id.c_str());
    }
  }

  ScanPreprocessor processor_;
  double marker_size_m_{0.035};
  double marker_lifetime_sec_{1.0};
  std::string last_marker_frame_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

}  // namespace amr_reobservation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<amr_reobservation::ScanPreprocessorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("scan_preprocessor"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
