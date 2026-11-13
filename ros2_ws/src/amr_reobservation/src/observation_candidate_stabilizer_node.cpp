#include "amr_reobservation/candidate_stability.hpp"
#include "amr_reobservation/msg/observation_candidates.hpp"

#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace amr_reobservation
{
// A confirmation gate, not a motion controller or an average of stale poses.
// Comparisons use fixed-frame geometry at the acquisition stamp. Accepted
// outputs retain the CURRENT source-frame geometry and acquisition stamp.
class ObservationCandidateStabilizer : public rclcpp::Node
{
  using Message = msg::ObservationCandidates;
  using Marker = visualization_msgs::msg::Marker;
  using Steady = std::chrono::steady_clock;
  struct Pending {Message message; Steady::time_point received;};

public:
  ObservationCandidateStabilizer() : Node("observation_candidate_stabilizer")
  {
    CandidateStabilityConfig config;
    const auto count = parameter<int>("confirmation_count", 3);
    if (count < 2 || count > 20) {throw std::invalid_argument("confirmation_count must be 2..20");}
    config.confirmation_count = static_cast<uint32_t>(count);
    config.max_position_delta_m = parameter("max_position_delta_m", .15);
    config.max_yaw_delta_rad = parameter("max_yaw_delta_deg", 3.) * std::acos(-1.) / 180.;
    config.max_gap_sec = parameter("max_source_gap_sec", .35);
    gate_ = std::make_unique<CandidateStabilityGate>(config);
    confirmation_count_ = count;
    frame_ = parameter<std::string>("expected_frame", "observation_lidar");
    fixed_frame_ = parameter<std::string>("fixed_frame", "odom");
    timeout_ = parameter("input_timeout_sec", 1.);
    max_age_ = parameter("max_input_age_sec", .5);
    future_tolerance_ = parameter("future_tolerance_sec", .1);
    tf_wait_ = parameter("tf_wait_timeout_sec", .2);
    if (frame_.empty() || fixed_frame_.empty() || frame_ == fixed_frame_ ||
      !positive(timeout_, 60) || !positive(max_age_, 60) || !positive(tf_wait_, 2) ||
      !std::isfinite(future_tolerance_) || future_tolerance_ < 0 || future_tolerance_ > 1)
    {throw std::invalid_argument("Invalid stabilizer frame/timing configuration");}
    const auto input = parameter<std::string>("input_topic", "/observation_candidates/raw");
    const auto output = parameter<std::string>("output_topic", "/observation_candidates");
    const auto markers = parameter<std::string>("marker_topic", "/debug/observation_candidates");
    if (input == output) {throw std::invalid_argument("Stabilizer input/output must differ");}
    output_ = create_publisher<Message>(output, rclcpp::QoS(1));
    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(markers, rclcpp::QoS(1));
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(
      *buffer_, this, false, rclcpp::QoS(100).best_effort());
    input_ = create_subscription<Message>(input, rclcpp::QoS(5),
      [this](Message::ConstSharedPtr m) {receive(*m);});
    processing_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {process();});
    watchdog_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      if (!have_input_) {invalidate("no_input", latest_); return;}
      if (std::chrono::duration<double>(Steady::now() - received_at_).count() > timeout_) {
        invalidate("input_timeout", latest_); return;
      }
      const auto error = time_error(latest_);
      if (!error.empty()) {invalidate(error, latest_);}
    });
    RCLCPP_INFO(get_logger(), "Candidate confirmation: %d observations in %s, %.3fm / %.2fdeg",
      count, fixed_frame_.c_str(), config.max_position_delta_m,
      config.max_yaw_delta_rad * 180. / std::acos(-1.));
  }

private:
  template<typename T> T parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor d; d.read_only = true;
    return declare_parameter<T>(name, value, d);
  }
  static bool positive(double value, double limit)
  {return std::isfinite(value) && value > 0 && value <= limit;}

  std::string time_error(const Message & m) const
  {
    if (m.header.stamp.sec < 0 || m.header.stamp.nanosec >= 1000000000U ||
      (m.header.stamp.sec == 0 && m.header.stamp.nanosec == 0)) {return "invalid_stamp";}
    const double age = (now() - rclcpp::Time(m.header.stamp, get_clock()->get_clock_type())).seconds();
    if (age > max_age_) {return "stale_input";}
    if (age < -future_tolerance_) {return "future_input";}
    return {};
  }

  void publish_invalid(const std::string & status, const Message & source)
  {
    Message out; out.header = source.header; out.extension_m = source.extension_m;
    out.standoff_m = source.standoff_m; out.status = status;
    output_->publish(out);
    Marker clear; clear.header.frame_id = frame_; clear.action = Marker::DELETEALL;
    visualization_msgs::msg::MarkerArray array; array.markers.push_back(clear);
    markers_->publish(array);
  }

  void invalidate(const std::string & status, const Message & source)
  {
    gate_->reset(); pending_.clear(); publish_invalid(status, source);
  }

  void receive(const Message & m)
  {
    latest_ = m; have_input_ = true; received_at_ = Steady::now();
    if (m.header.frame_id != frame_) {invalidate("unexpected_frame", m); return;}
    const auto error = time_error(m);
    if (!error.empty()) {invalidate(error, m); return;}
    const auto stamp = rclcpp::Time(m.header.stamp).nanoseconds();
    if (have_stamp_ && stamp <= previous_stamp_) {
      const auto reason = stamp == previous_stamp_ ? "duplicate_stamp" : "time_reset";
      previous_stamp_ = stamp; invalidate(reason, m); return;
    }
    have_stamp_ = true; previous_stamp_ = stamp;
    if (!m.valid) {invalidate("invalid_raw:" + m.status, m); return;}
    if (m.viewpoints.size() != 2 || m.look_at.size() != 2 ||
      !positive(m.extension_m, 100) || !positive(m.standoff_m, 100))
    {invalidate("invalid_candidates", m); return;}
    if (have_parameters_ && (extension_ != m.extension_m || standoff_ != m.standoff_m)) {
      gate_->reset(); pending_.clear();
      publish_invalid("parameters_changed", m);
    }
    extension_ = m.extension_m; standoff_ = m.standoff_m; have_parameters_ = true;
    if (pending_.size() >= 5) {invalidate("tf_queue_overflow", m); return;}
    pending_.push_back(Pending{m, received_at_});
    process();
  }

  static tf2::Quaternion quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    tf2::Quaternion value(q.x, q.y, q.z, q.w);
    if (!std::isfinite(value.length2()) || std::abs(value.length() - 1.) > 1e-3) {
      throw std::invalid_argument("invalid quaternion");
    }
    value.normalize(); return value;
  }

  static std::array<double, 3> position(const tf2::Transform & t,
    const geometry_msgs::msg::Point & p)
  {
    const auto v = t * tf2::Vector3(p.x, p.y, p.z);
    return {v.x(), v.y(), v.z()};
  }

  void process()
  {
    const auto current_time = now().nanoseconds();
    if (last_now_ && current_time < last_now_) {
      have_stamp_ = false; invalidate("time_reset", latest_);
    }
    last_now_ = current_time;
    while (!pending_.empty()) {
      const auto m = pending_.front().message;
      const auto error = time_error(m);
      if (!error.empty()) {invalidate(error, m); return;}
      geometry_msgs::msg::TransformStamped tf;
      try {
        // BufferCore's nonblocking overload: the wall timer owns retries.
        // The timed Buffer overload expects a dedicated TF thread even for
        // a zero timeout in Humble; this listener uses our executor instead.
        tf = buffer_->lookupTransform(fixed_frame_, frame_, tf2_ros::fromRclcpp(
            rclcpp::Time(m.header.stamp, get_clock()->get_clock_type())));
      } catch (const tf2::TransformException &) {
        if (std::chrono::duration<double>(Steady::now() - pending_.front().received).count() > tf_wait_) {
          invalidate("tf_unavailable", m);
        }
        return;
      }
      pending_.pop_front();
      StabilityPair pair;
      try {
        const auto rotation = quaternion(tf.transform.rotation);
        const auto & t = tf.transform.translation;
        if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z)) {
          throw std::invalid_argument("invalid transform");
        }
        const tf2::Transform transform(rotation, tf2::Vector3(t.x, t.y, t.z));
        for (size_t i = 0; i < pair.size(); ++i) {
          pair[i].position = position(transform, m.viewpoints[i].position);
          pair[i].target = position(transform, m.look_at[i]);
          pair[i].yaw = tf2::getYaw(rotation * quaternion(m.viewpoints[i].orientation));
        }
      } catch (const std::invalid_argument &) {invalidate("invalid_geometry_or_tf", m); return;}
      const auto decision = gate_->update(pair, rclcpp::Time(m.header.stamp).nanoseconds());
      if (!decision.valid) {
        publish_invalid(decision.status + ":" + std::to_string(decision.consecutive) + "/" +
          std::to_string(confirmation_count_), m);
        continue;
      }
      auto out = m; out.status = decision.status; output_->publish(out); publish_markers(out);
    }
  }

  void publish_markers(const Message & m)
  {
    visualization_msgs::msg::MarkerArray array;
    auto make = [&](int id, int type) {
        Marker v; v.header = m.header; v.ns = "viewpoints"; v.id = id;
        v.type = type; v.action = Marker::ADD; v.pose.orientation.w = 1;
        v.color.a = 1; v.lifetime = rclcpp::Duration::from_seconds(timeout_); return v;
      };
    auto side = make(0, Marker::LINE_STRIP); side.scale.x = .03;
    side.color.r = side.color.g = 1; side.points = m.look_at; array.markers.push_back(side);
    for (size_t i = 0; i < 2; ++i) {
      const int id = 10 + static_cast<int>(i) * 10;
      auto dot = make(id, Marker::SPHERE); dot.pose = m.viewpoints[i];
      dot.scale.x = dot.scale.y = dot.scale.z = .16;
      dot.color.r = i ? 1.F : 0.F; dot.color.g = i ? .4F : 1.F; dot.color.b = i ? .8F : .4F;
      array.markers.push_back(dot);
      auto arrow = make(id + 1, Marker::ARROW); arrow.color = dot.color;
      arrow.scale.x = .035; arrow.scale.y = .10; arrow.scale.z = .15;
      arrow.points = {m.viewpoints[i].position, m.look_at[i]}; array.markers.push_back(arrow);
      auto text = make(id + 3, Marker::TEXT_VIEW_FACING); text.color = dot.color;
      text.scale.z = .13; text.pose.position = m.viewpoints[i].position;
      text.pose.position.y += .23; text.pose.position.z += .2;
      text.text = std::string(i ? "B" : "A") + " confirmed (" + std::to_string(confirmation_count_) + ")";
      array.markers.push_back(text);
    }
    markers_->publish(array);
  }

  std::unique_ptr<CandidateStabilityGate> gate_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::deque<Pending> pending_;
  std::string frame_, fixed_frame_;
  int confirmation_count_;
  double timeout_, max_age_, future_tolerance_, tf_wait_, extension_{0}, standoff_{0};
  bool have_input_{false}, have_stamp_{false}, have_parameters_{false};
  int64_t previous_stamp_{0}, last_now_{0};
  Steady::time_point received_at_;
  Message latest_;
  rclcpp::Publisher<Message>::SharedPtr output_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Subscription<Message>::SharedPtr input_;
  rclcpp::TimerBase::SharedPtr processing_, watchdog_;
};
}  // namespace amr_reobservation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<amr_reobservation::ObservationCandidateStabilizer>());}
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("observation_candidate_stabilizer"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
