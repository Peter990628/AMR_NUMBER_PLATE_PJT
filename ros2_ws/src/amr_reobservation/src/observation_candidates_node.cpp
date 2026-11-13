#include "amr_reobservation/observation_candidates.hpp"
#include "amr_reobservation/msg/observation_candidates.hpp"
#include "amr_reobservation/msg/vehicle_side_estimate.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace amr_reobservation
{
class ObservationCandidatesNode : public rclcpp::Node
{
  using Input = msg::VehicleSideEstimate;
  using Output = msg::ObservationCandidates;
  using Marker = visualization_msgs::msg::Marker;
  using Steady = std::chrono::steady_clock;
public:
  ObservationCandidatesNode() : Node("observation_candidates")
  {
    config_.extension_m = parameter("extension_m", 0.8);
    config_.standoff_m = parameter("standoff_m", 1.0);
    timeout_ = parameter("input_timeout_sec", 1.0);
    max_age_ = parameter("max_input_age_sec", 0.5);
    future_tolerance_ = parameter("future_tolerance_sec", 0.1);
    frame_ = parameter<std::string>("expected_frame", "observation_lidar");
    const auto input = parameter<std::string>("input_topic", "/vehicle_side/estimate");
    const auto output = parameter<std::string>("output_topic", "/observation_candidates");
    const auto markers = parameter<std::string>("marker_topic", "/debug/observation_candidates");
    if (!std::isfinite(config_.extension_m) || config_.extension_m <= 0 ||
      !std::isfinite(config_.standoff_m) || config_.standoff_m <= 0 ||
      !std::isfinite(timeout_) || timeout_ <= 0 || timeout_ > 60 ||
      !std::isfinite(max_age_) || max_age_ <= 0 || max_age_ > 60 ||
      !std::isfinite(future_tolerance_) || future_tolerance_ < 0 || future_tolerance_ > 1 ||
      frame_.empty()) {throw std::invalid_argument("Invalid candidate configuration");}
    output_ = create_publisher<Output>(output, rclcpp::QoS(1));
    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(markers, rclcpp::QoS(1));
    input_ = create_subscription<Input>(input, rclcpp::QoS(1),
      [this](Input::ConstSharedPtr message) {on_input(*message);});
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      if (!received_) {invalidate("no_input"); return;}
      if (std::chrono::duration<double>(Steady::now() - received_at_).count() > timeout_) {
        invalidate("input_timeout"); return;
      }
      const auto error = time_error();
      if (!error.empty()) {invalidate(error);}
    });
    RCLCPP_INFO(get_logger(), "Geometric viewpoints: L=%.2fm, D=%.2fm; no motion commands",
      config_.extension_m, config_.standoff_m);
  }

private:
  template<typename T> T parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.read_only = true;
    return declare_parameter<T>(name, value, d);
  }

  std::string time_error()
  {
    if (header_.stamp.sec < 0 || header_.stamp.nanosec >= 1000000000U) {return "invalid_stamp";}
    const double age = (now() - rclcpp::Time(header_.stamp, get_clock()->get_clock_type())).seconds();
    if (age > max_age_) {return "stale_input";}
    if (age < -future_tolerance_) {return "future_input";}
    return {};
  }

  Output base_output() const
  {
    Output out;
    out.header = header_;
    out.extension_m = config_.extension_m;
    out.standoff_m = config_.standoff_m;
    return out;
  }

  Marker marker(int id, int type) const
  {
    Marker m;
    m.header = header_;
    m.header.frame_id = frame_;
    m.ns = "viewpoints"; m.id = id; m.type = type; m.action = Marker::ADD;
    m.pose.orientation.w = 1;
    m.color.a = 1;
    m.lifetime = rclcpp::Duration::from_seconds(timeout_);
    return m;
  }

  static geometry_msgs::msg::Point point(const Vec2 & v, double z = 0)
  {
    geometry_msgs::msg::Point p;
    p.x = v[0]; p.y = v[1]; p.z = z; return p;
  }

  void invalidate(const std::string & reason)
  {
    auto out = base_output();
    out.status = reason;
    output_->publish(out);
    auto clear = marker(0, Marker::SPHERE);
    clear.action = Marker::DELETEALL;
    clear.header.stamp = builtin_interfaces::msg::Time{};
    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(clear);
    markers_->publish(array);
  }

  void on_input(const Input & in)
  {
    header_ = in.header; received_ = true; received_at_ = Steady::now();
    if (header_.frame_id != frame_) {invalidate("unexpected_frame"); return;}
    const auto error = time_error();
    if (!error.empty()) {invalidate(error); return;}
    const auto stamp = rclcpp::Time(header_.stamp).nanoseconds();
    if (have_stamp_ && stamp <= previous_stamp_) {
      const auto reason = stamp == previous_stamp_ ? "duplicate_stamp" : "time_reset";
      previous_stamp_ = stamp;
      invalidate(reason); return;
    }
    have_stamp_ = true; previous_stamp_ = stamp;
    if (!in.valid) {invalidate("invalid_side:" + in.status); return;}
    for (double z : {in.endpoint_a.z, in.endpoint_b.z, in.tangent.z, in.normal_toward_sensor.z}) {
      if (!std::isfinite(z) || std::abs(z) > 1e-6) {invalidate("nonplanar_input"); return;}
    }
    CandidateSide side{{in.endpoint_a.x, in.endpoint_a.y}, {in.endpoint_b.x, in.endpoint_b.y},
      {in.tangent.x, in.tangent.y}, {in.normal_toward_sensor.x, in.normal_toward_sensor.y}};
    std::array<ObservationCandidate, 2> candidates;
    try {candidates = make_observation_candidates(side, config_);}
    catch (const std::invalid_argument &) {invalidate("invalid_geometry"); return;}
    auto out = base_output();
    out.valid = true; out.status = "geometric_candidates";
    visualization_msgs::msg::MarkerArray array;
    auto side_line = marker(0, Marker::LINE_STRIP);
    side_line.scale.x = .03; side_line.color.r = 1; side_line.color.g = 1;
    side_line.points = {point(side.a), point(side.b)};
    array.markers.push_back(side_line);
    auto origin = marker(1, Marker::SPHERE);
    origin.scale.x = origin.scale.y = origin.scale.z = .12;
    origin.color.r = origin.color.g = origin.color.b = 1;
    array.markers.push_back(origin);
    for (size_t i = 0; i < candidates.size(); ++i) {
      const auto & c = candidates[i];
      geometry_msgs::msg::Pose pose;
      pose.position = point(c.position);
      pose.orientation.z = std::sin(c.yaw / 2); pose.orientation.w = std::cos(c.yaw / 2);
      out.viewpoints.push_back(pose); out.look_at.push_back(point(c.target));
      const int id = 10 + static_cast<int>(i) * 10;
      auto position = marker(id, Marker::SPHERE);
      position.pose.position = pose.position;
      position.scale.x = position.scale.y = position.scale.z = .16;
      position.color.r = i ? 1.F : 0.F; position.color.g = i ? .4F : 1.F; position.color.b = i ? .8F : .4F;
      array.markers.push_back(position);
      auto arrow = marker(id + 1, Marker::ARROW);
      arrow.color = position.color;
      arrow.scale.x = .035; arrow.scale.y = .10; arrow.scale.z = .15;
      arrow.points = {point(c.position), point(c.target)};
      array.markers.push_back(arrow);
      auto offsets = marker(id + 2, Marker::LINE_STRIP);
      offsets.color = position.color; offsets.scale.x = .012;
      const Vec2 corner{c.position[0] - config_.standoff_m * side.normal[0],
        c.position[1] - config_.standoff_m * side.normal[1]};
      offsets.points = {point(c.target), point(corner), point(c.position)};
      array.markers.push_back(offsets);
      auto text = marker(id + 3, Marker::TEXT_VIEW_FACING);
      text.color = position.color; text.scale.z = .13;
      text.pose.position = point({c.position[0], c.position[1] + .23}, .2);
      std::ostringstream label;
      label << std::fixed << std::setprecision(2) << (i ? "B" : "A") << " ("
            << c.position[0] << ", " << c.position[1] << ")\nL=" << config_.extension_m
            << " D=" << config_.standoff_m << "\nyaw=" << c.yaw * 180 / 3.141592653589793;
      text.text = label.str(); array.markers.push_back(text);
    }
    output_->publish(out); markers_->publish(array);
  }

  CandidateConfig config_;
  std::string frame_;
  double timeout_, max_age_, future_tolerance_;
  bool received_{false}, have_stamp_{false};
  int64_t previous_stamp_{0};
  Steady::time_point received_at_;
  std_msgs::msg::Header header_;
  rclcpp::Publisher<Output>::SharedPtr output_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Subscription<Input>::SharedPtr input_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<amr_reobservation::ObservationCandidatesNode>());}
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("observation_candidates"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
