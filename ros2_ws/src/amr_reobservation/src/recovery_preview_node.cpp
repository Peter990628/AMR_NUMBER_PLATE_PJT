#include "amr_reobservation/recovery_preview.hpp"
#include "amr_reobservation/recovery_contour.hpp"
#include "amr_reobservation/msg/plate_observation.hpp"
#include "amr_reobservation/msg/recovery_preview.hpp"
#include "amr_reobservation/msg/recovery_request.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace amr_reobservation
{
// This node has no motion-command publisher. Eligibility only checks observed
// geometry; it does not certify unseen free space or a traversable path.
class RecoveryPreviewNode : public rclcpp::Node
{
  using Request = msg::RecoveryRequest;
  using Output = msg::RecoveryPreview;
  using Scan = sensor_msgs::msg::LaserScan;
  using Marker = visualization_msgs::msg::Marker;
  using Steady = std::chrono::steady_clock;
  using Point = geometry_msgs::msg::Point;
  struct RequestRecord
  {
    Request message;
    Steady::time_point received;
    uint64_t epoch;
    bool blocked{false};
  };

public:
  RecoveryPreviewNode() : Node("recovery_preview")
  {
    fixed_frame_ = parameter<std::string>("fixed_frame", "odom");
    base_frame_ = parameter<std::string>("base_frame", "base_link");
    camera_frame_ = parameter<std::string>("camera_optical_frame", "front_stereo_camera_left_optical");
    scan_timeout_ = parameter("scan_timeout_sec", 1.5);
    max_scan_age_ = parameter("max_scan_age_sec", .5);
    future_tolerance_ = parameter("future_tolerance_sec", .1);
    tf_wait_ = parameter("tf_wait_sec", .3);
    request_timeout_ = parameter("request_timeout_sec", 300.);
    association_radius_ = parameter("target_association_radius_m", 2.);
    const double approach_margin = parameter("approach_association_margin_m", .10);
    range_min_ = parameter("range_min_m", .1);
    range_max_ = parameter("range_max_m", 6.);
    config_.forward_step_m = parameter("forward_step_m", .35);
    config_.lateral_step_m = parameter("lateral_step_m", .35);
    config_.min_target_distance_m = parameter("min_target_distance_m", 1.);
    config_.max_turn_rad = parameter("max_turn_deg", 60.) * std::acos(-1.) / 180.;
    config_.clearance_radius_m = parameter("clearance_radius_m", .75);
    contour_config_.cluster_gap_m = parameter("contour_cluster_gap_m", .18);
    const int missing_beams = parameter("contour_max_missing_beams", 2);
    const int cluster_points = parameter("contour_min_cluster_points", 12);
    contour_config_.association_radius_m = association_radius_;
    contour_config_.ambiguity_margin_m = parameter("contour_ambiguity_margin_m", .15);
    contour_config_.split_distance_m = parameter("contour_split_distance_m", .06);
    contour_config_.min_segment_length_m = parameter("contour_min_segment_length_m", .45);
    const int segment_points = parameter("contour_min_segment_points", 12);
    const int max_aims = parameter("contour_max_aims", 3);
    if (fixed_frame_.empty() || base_frame_.empty() || camera_frame_.empty() ||
      fixed_frame_ == base_frame_ || base_frame_ == camera_frame_ ||
      !positive(scan_timeout_, 60) || !positive(max_scan_age_, 60) ||
      !positive(tf_wait_, 2) || !positive(request_timeout_, 3600) ||
      !positive(association_radius_, 20) || !positive(range_min_, 100) ||
      approach_margin < 0 || !positive(association_radius_ + approach_margin, 20) ||
      !positive(range_max_, 100) || range_min_ >= range_max_ ||
      !std::isfinite(future_tolerance_) || future_tolerance_ < 0 || future_tolerance_ > 1 ||
      !positive(config_.forward_step_m, 5) || !positive(config_.lateral_step_m, 5) ||
      !positive(config_.min_target_distance_m, 20) ||
      !positive(config_.max_turn_rad, std::acos(-1.)) ||
      !positive(config_.clearance_radius_m, 5) ||
      !positive(contour_config_.cluster_gap_m, 5) ||
      missing_beams < 0 || missing_beams > 64 || cluster_points < 2 || cluster_points > 10000 ||
      !std::isfinite(contour_config_.ambiguity_margin_m) ||
      contour_config_.ambiguity_margin_m < 0 || contour_config_.ambiguity_margin_m > 20 ||
      !positive(contour_config_.split_distance_m, 1) ||
      !positive(contour_config_.min_segment_length_m, 20) ||
      segment_points < 2 || segment_points > 10000 || max_aims < 1 || max_aims > 8)
    {throw std::invalid_argument("Invalid recovery preview configuration");}
    contour_config_.max_missing_beams = static_cast<size_t>(missing_beams);
    contour_config_.min_cluster_points = static_cast<size_t>(cluster_points);
    contour_config_.min_segment_points = static_cast<size_t>(segment_points);
    contour_config_.max_aims = static_cast<size_t>(max_aims);
    approach_contour_config_ = contour_config_;
    approach_contour_config_.association_radius_m += approach_margin;
    rcl_interfaces::msg::ParameterDescriptor marker_descriptor;
    marker_descriptor.description =
      "Zero-based aim whose six candidate poses are displayed; display only, not goal selection";
    rcl_interfaces::msg::IntegerRange marker_range;
    marker_range.from_value = 0; marker_range.to_value = max_aims - 1; marker_range.step = 1;
    marker_descriptor.integer_range.push_back(marker_range);
    const int marker_index = declare_parameter<int>("marker_aim_index", 0, marker_descriptor);
    if (marker_index < 0 || marker_index >= max_aims) {
      throw std::invalid_argument("marker_aim_index is outside contour_max_aims");
    }
    const auto scan_topic = parameter<std::string>("scan_topic", "/observation_lidar/scan");
    const auto request_topic = parameter<std::string>("request_topic", "/reobservation/request");
    const auto output_topic = parameter<std::string>("output_topic", "/reobservation/preview");
    const auto marker_topic = parameter<std::string>("marker_topic", "/debug/reobservation/preview");
    const auto plate_topic = parameter<std::string>("plate_topic", "/plate/observation");
    output_ = create_publisher<Output>(output_topic, rclcpp::QoS(1));
    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, rclcpp::QoS(1));
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(
      *buffer_, this, false, rclcpp::QoS(100).best_effort());
    requests_ = create_subscription<Request>(request_topic, rclcpp::QoS(1),
      [this](Request::ConstSharedPtr m) {receive_request(*m);});
    scans_ = create_subscription<Scan>(scan_topic, rclcpp::SensorDataQoS(),
      [this](Scan::ConstSharedPtr m) {receive_scan(m);});
    plates_ = create_subscription<msg::PlateObservation>(plate_topic, rclcpp::QoS(1).best_effort(),
      [this](msg::PlateObservation::ConstSharedPtr m) {
        check_clock(); plate_ = *m; plate_received_ = Steady::now();
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {process();});
    RCLCPP_INFO(get_logger(), "Re-observation PREVIEW in %s: no navigation or velocity commands",
      fixed_frame_.c_str());
  }

private:
  template<typename T> T parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor d; d.read_only = true;
    return declare_parameter<T>(name, value, d);
  }
  static bool positive(double v, double limit)
  {return std::isfinite(v) && v > 0 && v <= limit;}
  static bool finite(const Point & p)
  {return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);}
  static double elapsed(Steady::time_point start)
  {return std::chrono::duration<double>(Steady::now() - start).count();}
  static bool valid_stamp(const builtin_interfaces::msg::Time & s)
  {return s.sec >= 0 && s.nanosec < 1000000000U && (s.sec > 0 || s.nanosec > 0);}
  double age(const builtin_interfaces::msg::Time & s) const
  {return (now() - rclcpp::Time(s, get_clock()->get_clock_type())).seconds();}
  static bool same_key(const Request & a, const Request & b)
  {return a.request_id == b.request_id && a.header.stamp == b.header.stamp;}
  static tf2::Quaternion quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
    if (!std::isfinite(rotation.length2()) || std::abs(rotation.length() - 1.) > 1e-3) {
      throw std::invalid_argument("invalid quaternion");
    }
    rotation.normalize(); return rotation;
  }
  static bool planar(const tf2::Quaternion & q)
  {
    const auto up = tf2::quatRotate(q, tf2::Vector3(0, 0, 1));
    return up.z() >= std::cos(5. * std::acos(-1.) / 180.);
  }
  static tf2::Transform transform(const geometry_msgs::msg::TransformStamped & m)
  {
    const auto & p = m.transform.translation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      throw std::invalid_argument("invalid translation");
    }
    return tf2::Transform(quaternion(m.transform.rotation), tf2::Vector3(p.x, p.y, p.z));
  }
  static Point point(double x, double y, double z = 0)
  {Point p; p.x = x; p.y = y; p.z = z; return p;}
  static geometry_msgs::msg::Pose pose(const RecoveryPose & p, double z)
  {
    geometry_msgs::msg::Pose v; v.position = point(p.position[0], p.position[1], z);
    v.orientation.z = std::sin(p.yaw / 2); v.orientation.w = std::cos(p.yaw / 2); return v;
  }

  bool check_clock()
  {
    const auto current = now().nanoseconds();
    const bool reset = last_now_ && current < last_now_;
    last_now_ = current;
    if (reset) {
      // Old identities remain tied to their old clock/odom epoch. Replaying
      // an anchored request is not a new request after the scene resets.
      ++epoch_;
      request_.reset(); request_context_.reset(); latest_scan_.reset(); pending_.reset(); plate_.reset();
      previous_scan_stamp_ = 0; buffer_->clear(); request_error_ = "time_reset";
      invalidate("time_reset");
    }
    return reset;
  }

  std::string request_error() const
  {
    if (!request_) {return request_error_;}
    if (!request_->enabled) {return "disabled";}
    if (age(request_->header.stamp) < -future_tolerance_) {return "request_future";}
    if (age(request_->header.stamp) > request_->valid_for_sec) {return "request_expired";}
    if (elapsed(request_received_) > request_timeout_) {return "request_timeout";}
    return {};
  }

  void receive_request(const Request & m)
  {
    check_clock();
    request_context_ = m;  // Diagnostic identity only; never used as authorization.
    if (!m.enabled) {
      if (request_) {
        const auto active_key = std::make_pair(
          request_->request_id, rclcpp::Time(request_->header.stamp).nanoseconds());
        request_records_.at(active_key).blocked = true;
      }
      if (valid_stamp(m.header.stamp)) {
        const auto key = std::make_pair(m.request_id, rclcpp::Time(m.header.stamp).nanoseconds());
        const auto found = request_records_.find(key);
        if (found != request_records_.end()) {found->second.blocked = true;}
      }
      request_.reset(); pending_.reset(); request_error_ = "disabled";
      invalidate("disabled"); return;
    }
    if (valid_stamp(m.header.stamp)) {
      const auto key = std::make_pair(m.request_id, rclcpp::Time(m.header.stamp).nanoseconds());
      const auto found = request_records_.find(key);
      if (found != request_records_.end()) {
        auto & previous = found->second;
        if (previous.epoch != epoch_ || previous.blocked || previous.message != m) {
          previous.blocked = true; request_.reset(); pending_.reset();
          request_error_ = previous.epoch != epoch_ ? "time_reset" : "request_conflict";
          invalidate(request_error_); return;
        }
        const bool already_active = request_ && same_key(m, *request_);
        request_ = m; request_received_ = previous.received;
        const auto error = request_error();
        if (!error.empty()) {pending_.reset(); invalidate(error); return;}
        if (!already_active) {invalidate("awaiting_scan");}
        // Identical heartbeats never refresh either ROS or wall-clock expiry.
        return;
      }
    }
    std::string error;
    if (m.header.frame_id != fixed_frame_) {error = "request_frame_mismatch";}
    else if (!valid_stamp(m.header.stamp) || m.request_id.empty() || m.target_id.empty() ||
      !finite(m.vehicle_hint) || !finite(m.approach_pose.position) ||
      std::abs(m.vehicle_hint.z) > 1e-6 || std::abs(m.approach_pose.position.z) > 1e-6 ||
      !positive(m.valid_for_sec, 600)) {error = "invalid_request";}
    else {
      try {
        const auto rotation = quaternion(m.approach_pose.orientation);
        if (std::abs(rotation.x()) > 1e-6 || std::abs(rotation.y()) > 1e-6) {
          error = "invalid_request";
        }
      } catch (const std::invalid_argument &) {error = "invalid_request";}
      if (error.empty() && age(m.header.stamp) < -future_tolerance_) {error = "request_future";}
      if (error.empty() && age(m.header.stamp) > m.valid_for_sec) {error = "request_expired";}
      // Never evict an identity and inadvertently let its heartbeat renew a
      // lease. This preview session fails closed at a bounded history size.
      if (error.empty() && request_records_.size() >= 1024) {error = "request_history_full";}
    }
    pending_.reset();
    if (!error.empty()) {
      request_.reset(); request_error_ = error; invalidate(error); return;
    }
    request_ = m; request_received_ = Steady::now(); request_error_.clear();
    request_records_.emplace(
      std::make_pair(m.request_id, rclcpp::Time(m.header.stamp).nanoseconds()),
      RequestRecord{m, request_received_, epoch_, false});
    invalidate("awaiting_scan");
    if (latest_scan_ && scan_error(*latest_scan_).empty() && elapsed(scan_received_) <= scan_timeout_) {
      pending_ = latest_scan_; pending_started_ = Steady::now(); process();
    }
  }

  std::string scan_error(const Scan & m) const
  {
    if (m.header.frame_id.empty() || !valid_stamp(m.header.stamp) ||
      !std::isfinite(m.angle_min) || !std::isfinite(m.angle_max) ||
      !std::isfinite(m.angle_increment) || m.angle_increment == 0 ||
      !std::isfinite(m.time_increment) || m.time_increment < 0 ||
      !std::isfinite(m.scan_time) || m.scan_time < 0 ||
      !std::isfinite(m.range_min) || !std::isfinite(m.range_max) ||
      m.range_min < 0 || m.range_max <= m.range_min)
    {return "invalid_scan";}
    if (age(m.header.stamp) > max_scan_age_) {return "stale_scan";}
    if (age(m.header.stamp) < -future_tolerance_) {return "future_scan";}
    return {};
  }

  void receive_scan(Scan::ConstSharedPtr m)
  {
    check_clock();
    const auto error = scan_error(*m);
    if (!error.empty()) {
      latest_scan_ = m; pending_.reset(); scan_received_ = Steady::now(); invalidate(error); return;
    }
    const auto stamp = rclcpp::Time(m->header.stamp).nanoseconds();
    if (stamp <= previous_scan_stamp_) {return;}
    previous_scan_stamp_ = stamp; latest_scan_ = m; scan_received_ = Steady::now();
    // Replacing the queued scan does not keep extending a missing-TF deadline.
    if (!pending_) {pending_started_ = Steady::now();}
    pending_ = m; process();
  }

  std::string plate_status() const
  {
    if (!plate_) {return "no_plate";}
    if (!valid_stamp(plate_->header.stamp) || plate_->header.frame_id != camera_frame_) {
      return "invalid_plate";
    }
    if (elapsed(plate_received_) > scan_timeout_ || age(plate_->header.stamp) > max_scan_age_ ||
      age(plate_->header.stamp) < -future_tolerance_) {return "stale_plate";}
    if (!plate_->input_valid) {return "invalid_plate:" + plate_->status;}
    if (plate_->status == "usable" && (!plate_->usable || !plate_->detected)) {return "invalid_plate";}
    return plate_->status;
  }

  Output base_output() const
  {
    Output out; out.header.frame_id = fixed_frame_;
    if (latest_scan_ && valid_stamp(latest_scan_->header.stamp)) {
      out.header.stamp = latest_scan_->header.stamp;
    }
    if (request_context_) {
      out.request_header = request_context_->header; out.request_id = request_context_->request_id;
      out.target_id = request_context_->target_id; out.vehicle_hint = request_context_->vehicle_hint;
      out.approach_pose = request_context_->approach_pose;
    }
    out.marker_aim_index = -1;
    out.plate_status = plate_status(); return out;
  }

  void invalidate(const std::string & reason, size_t contour_clusters = 0,
    const std::string & approach_status = {})
  {
    const auto out_base = base_output();
    if (!last_valid_ && last_status_ == reason && last_header_ == out_base.header &&
      last_request_id_ == out_base.request_id && elapsed(last_publish_) < .5) {return;}
    auto out = out_base; out.status = reason;
    if (reason.rfind("contour_", 0) == 0) {
      out.contour_status = reason.substr(8);
      out.approach_contour_status = approach_status;
      out.contour_cluster_count = static_cast<uint32_t>(contour_clusters);
    }
    output_->publish(out);
    Marker clear; clear.header.frame_id = fixed_frame_; clear.action = Marker::DELETEALL;
    visualization_msgs::msg::MarkerArray array; array.markers.push_back(clear); markers_->publish(array);
    last_valid_ = false; last_status_ = reason; last_header_ = out.header;
    last_request_id_ = out.request_id; last_publish_ = Steady::now();
  }

  void process()
  {
    if (check_clock()) {return;}
    auto error = request_error();
    if (!error.empty()) {pending_.reset(); invalidate(error); return;}
    if (!latest_scan_) {invalidate("no_scan"); return;}
    if (elapsed(scan_received_) > scan_timeout_) {pending_.reset(); invalidate("scan_timeout"); return;}
    error = scan_error(*latest_scan_);
    if (!error.empty()) {pending_.reset(); invalidate(error); return;}
    if (!pending_) {return;}
    const auto scan = pending_;
    tf2::Transform fixed_scan, fixed_base, base_camera;
    try {
      const auto stamp = tf2_ros::fromRclcpp(rclcpp::Time(scan->header.stamp, get_clock()->get_clock_type()));
      // Exact nonzero acquisition time, never a latest-transform fallback.
      fixed_scan = transform(buffer_->lookupTransform(fixed_frame_, scan->header.frame_id, stamp));
      fixed_base = transform(buffer_->lookupTransform(fixed_frame_, base_frame_, stamp));
      base_camera = transform(buffer_->lookupTransform(base_frame_, camera_frame_, stamp));
    } catch (const tf2::TransformException &) {
      if (elapsed(pending_started_) > tf_wait_) {pending_.reset(); invalidate("tf_unavailable");}
      return;
    } catch (const std::invalid_argument &) {
      pending_.reset(); invalidate("invalid_tf"); return;
    }
    pending_.reset();
    if (!planar(fixed_scan.getRotation()) || !planar(fixed_base.getRotation())) {
      invalidate("nonplanar_tf"); return;
    }
    const auto forward = tf2::quatRotate(base_camera.getRotation(), tf2::Vector3(0, 0, 1));
    if (std::abs(forward.z()) > std::sin(5. * std::acos(-1.) / 180.)) {
      invalidate("nonplanar_camera"); return;
    }
    std::vector<Vec2> points;
    std::vector<ScanPoint> scan_points;
    std::vector<Point> display_points;
    std::vector<double> beam_heights(scan->ranges.size(), 0.);
    const double minimum = std::max(range_min_, static_cast<double>(scan->range_min));
    const double maximum = std::min(range_max_, static_cast<double>(scan->range_max));
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      const double r = scan->ranges[i];
      if (!std::isfinite(r) || r < minimum || r > maximum) {continue;}
      const double angle = scan->angle_min + static_cast<double>(i) * scan->angle_increment;
      if (!std::isfinite(angle)) {invalidate("invalid_scan"); return;}
      const auto p = fixed_scan * tf2::Vector3(r * std::cos(angle), r * std::sin(angle), 0);
      points.push_back({p.x(), p.y()}); display_points.push_back(point(p.x(), p.y(), p.z()));
      scan_points.push_back({p.x(), p.y(), i}); beam_heights[i] = p.z();
    }
    if (points.empty()) {invalidate("no_observed_points"); return;}
    RecoveryPose current{{fixed_base.getOrigin().x(), fixed_base.getOrigin().y()},
      tf2::getYaw(fixed_base.getRotation())};
    CameraPlanarExtrinsic camera{{base_camera.getOrigin().x(), base_camera.getOrigin().y()},
      std::atan2(forward.y(), forward.x())};
    RecoveryContourResult contour;
    std::string approach_status;
    const Vec2 hint{request_->vehicle_hint.x, request_->vehicle_hint.y};
    const ScanGeometry geometry{
      scan->angle_min, scan->angle_increment, scan->range_min, scan->range_max};
    try {
      contour = extract_recovery_contour(
        scan_points, geometry, scan->ranges.size(), hint, contour_config_);
      approach_status = contour.status;
      // Widening can admit an out-of-radius cluster or introduce a competitor.
      if ((contour.status == "target_unassociated" || contour.cluster_count > 1) &&
        approach_contour_config_.association_radius_m != contour_config_.association_radius_m)
      {
        approach_status = extract_recovery_contour(
          scan_points, geometry, scan->ranges.size(), hint, approach_contour_config_).status;
      }
    } catch (const std::invalid_argument &) {invalidate("invalid_geometry"); return;}
    const double camera_height = fixed_base.getOrigin().z() + base_camera.getOrigin().z();
    const auto camera_fixed = fixed_base * base_camera.getOrigin();
    const auto current_camera = point(camera_fixed.x(), camera_fixed.y(), camera_fixed.z());
    // Finite TF components can still overflow when composing translations.
    if (!std::isfinite(camera_height) || !finite(current_camera)) {
      invalidate("invalid_geometry"); return;
    }
    if (contour.status != "ok") {
      invalidate("contour_" + contour.status, contour.cluster_count, approach_status); return;
    }
    auto out = base_output(); out.header.stamp = scan->header.stamp;
    out.valid = true; out.status = "preview_only";
    out.contour_status = contour.status;
    out.approach_contour_status = approach_status;
    out.contour_cluster_count = static_cast<uint32_t>(contour.cluster_count);
    out.current_pose = pose(current, fixed_base.getOrigin().z());
    for (const auto & p : contour.contour) {
      out.contour_points.push_back(point(p.x, p.y, beam_heights.at(p.beam_index)));
      out.contour_beam_indices.push_back(static_cast<uint32_t>(p.beam_index));
    }
    for (size_t aim_index = 0; aim_index < contour.aims.size(); ++aim_index) {
      const auto & aim = contour.aims[aim_index];
      const std::string label = "A" + std::to_string(aim_index + 1);
      out.aim_labels.push_back(label);
      // These are XY surface observations projected to camera height, not
      // inferred plate positions or measured plate heights.
      out.aim_points.push_back(point(aim.point[0], aim.point[1], camera_height));
      out.aim_endpoint_a.push_back(point(aim.endpoint_a[0], aim.endpoint_a[1], camera_height));
      out.aim_endpoint_b.push_back(point(aim.endpoint_b[0], aim.endpoint_b[1], camera_height));
      out.aim_support_counts.push_back(static_cast<uint32_t>(aim.support_count));
      out.aim_observed_lengths_m.push_back(aim.observed_length_m);
      out.aim_max_deviations_m.push_back(aim.max_deviation_m);
      std::vector<RecoveryPreviewCandidate> candidates;
      try {
        candidates = make_recovery_preview_for_aim(current, hint, aim.point, camera, points, config_);
      } catch (const std::invalid_argument &) {invalidate("invalid_geometry"); return;}
      for (const auto & c : candidates) {
        out.labels.push_back(label + "/" + c.label);
        out.candidate_aim_indices.push_back(static_cast<uint32_t>(aim_index));
        out.base_poses.push_back(pose(c.base_pose, fixed_base.getOrigin().z()));
        out.camera_positions.push_back(point(c.camera_position[0], c.camera_position[1], camera_height));
        out.look_at.push_back(point(c.look_at[0], c.look_at[1], camera_height));
        out.eligible.push_back(c.eligible); out.reasons.push_back(c.reason);
        out.min_observed_distance_m.push_back(c.min_observed_distance_m);
      }
    }
    const int64_t marker_index = get_parameter("marker_aim_index").as_int();
    if (marker_index >= 0 && static_cast<size_t>(marker_index) < out.aim_points.size()) {
      out.marker_aim_index = static_cast<int32_t>(marker_index);
    }
    output_->publish(out);
    publish_markers(out, display_points, current_camera);
    last_valid_ = true; last_status_ = out.status; last_header_ = out.header;
    last_request_id_ = out.request_id; last_publish_ = Steady::now();
  }

  void publish_markers(
    const Output & out, const std::vector<Point> & points, const Point & current_camera)
  {
    visualization_msgs::msg::MarkerArray array;
    Marker clear; clear.header = out.header; clear.action = Marker::DELETEALL; array.markers.push_back(clear);
    auto make = [&](int id, int type, float r, float g, float b,
        const std::string & marker_namespace = "recovery_preview") {
        Marker m; m.header = out.header; m.ns = marker_namespace; m.id = id;
        m.type = type; m.action = Marker::ADD; m.pose.orientation.w = 1;
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1;
        m.lifetime = rclcpp::Duration::from_seconds(scan_timeout_); return m;
      };
    auto text = [&](int id, const Point & p, const std::string & label, float r, float g, float b,
        const std::string & marker_namespace = "recovery_preview") {
        auto m = make(id, Marker::TEXT_VIEW_FACING, r, g, b, marker_namespace); m.pose.position = p;
        m.pose.position.z += .3; m.scale.z = .095; m.text = label; array.markers.push_back(m);
      };
    auto arrow = [&](int id, const geometry_msgs::msg::Pose & p, float r, float g, float b,
        const std::string & marker_namespace = "recovery_preview") {
        auto m = make(id, Marker::ARROW, r, g, b, marker_namespace); m.pose = p;
        m.scale.x = .35; m.scale.y = .06; m.scale.z = .09; array.markers.push_back(m);
      };
    auto cloud = make(1, Marker::POINTS, .55F, .55F, .55F);
    cloud.scale.x = cloud.scale.y = .025; cloud.points = points; array.markers.push_back(cloud);
    auto contour = make(0, Marker::POINTS, 0, 1, 1, "recovery_contour");
    contour.scale.x = contour.scale.y = .035;
    contour.points = out.contour_points; array.markers.push_back(contour);
    auto spans = make(1, Marker::LINE_LIST, .8F, .3F, 1, "recovery_contour");
    spans.scale.x = .02;
    auto current_rays = make(0, Marker::LINE_LIST, 1, .3F, 1, "recovery_current_sightlines");
    current_rays.scale.x = .01; current_rays.color.a = .25F;
    for (size_t i = 0; i < out.aim_points.size(); ++i) {
      auto aim = make(static_cast<int>(i), Marker::SPHERE, 1, .1F, 1, "recovery_aims");
      aim.pose.position = out.aim_points[i];
      aim.scale.x = aim.scale.y = aim.scale.z = .12; array.markers.push_back(aim);
      auto label_point = out.aim_points[i]; label_point.x += .12; label_point.y += .20;
      text(100 + static_cast<int>(i), label_point, out.aim_labels[i], 1, .3F, 1, "recovery_aims");
      spans.points.push_back(out.aim_endpoint_a[i]); spans.points.push_back(out.aim_endpoint_b[i]);
      current_rays.points.push_back(current_camera); current_rays.points.push_back(out.aim_points[i]);
    }
    array.markers.push_back(spans); array.markers.push_back(current_rays);
    auto hint = make(2, Marker::SPHERE, 1, 1, 0); hint.pose.position = out.vehicle_hint;
    hint.scale.x = hint.scale.y = hint.scale.z = .18; array.markers.push_back(hint);
    auto hint_label = out.vehicle_hint; hint_label.y -= .25;
    text(3, hint_label, "vehicle hint (heading unknown)", 1, 1, 0);
    arrow(4, out.approach_pose, .2F, .5F, 1);
    auto approach_label = out.approach_pose.position; approach_label.y -= .65;
    text(5, approach_label, "original approach", .2F, .5F, 1);
    arrow(6, out.current_pose, 1, 1, 1);
    auto current_label = out.current_pose.position; current_label.y += .70;
    text(7, current_label, "current base", 1, 1, 1);
    auto scope_position = out.current_pose.position;
    scope_position.x += 1.5; scope_position.y -= 1.7;
    const std::string source =
      (request_->source.find("mock") != std::string::npos ||
      request_->source.find("MOCK") != std::string::npos) ?
      "MOCK vehicle hint" : request_->source.substr(0, 64);
    text(8, scope_position, "PREVIEW: path/free space unchecked\n"
      "Aims = observed surfaces, not plate locations\nplate: " + out.plate_status +
      " | source: " + source + "\nXY aim at camera height; heading unknown", 1, .65F, 0);
    const int64_t requested_index = get_parameter("marker_aim_index").as_int();
    std::string legend;
    if (out.marker_aim_index < 0) {
      legend = "A" + std::to_string(requested_index + 1) + " unavailable in this scan\n" +
        std::to_string(out.aim_points.size()) + " observed aim(s); candidate arrows hidden";
    } else {
      legend = out.aim_labels[static_cast<size_t>(out.marker_aim_index)] +
        " displayed (not a selected goal)";
    }
    size_t displayed = 0;
    for (size_t i = 0; i < out.labels.size(); ++i) {
      if (out.marker_aim_index < 0 ||
        out.candidate_aim_indices[i] != static_cast<uint32_t>(out.marker_aim_index)) {continue;}
      const int id = static_cast<int>(displayed) * 4;
      ++displayed;
      const bool eligible = out.eligible[i];
      const float red = eligible ? .15F : 1.F, green = eligible ? 1.F : .15F;
      arrow(id, out.base_poses[i], red, green, .15F, "recovery_candidates");
      auto sight = make(id + 1, Marker::LINE_STRIP, 1, .65F, 0, "recovery_candidates");
      sight.scale.x = .012; sight.points = {out.camera_positions[i], out.look_at[i]}; array.markers.push_back(sight);
      auto label_position = out.base_poses[i].position;
      label_position.x += .08; label_position.y += .06;
      text(id + 2, label_position, std::to_string(displayed), red, green, .15F, "recovery_candidates");
      legend += "\n" + std::to_string(displayed) + " " + out.labels[i] + ": " + out.reasons[i];
      auto camera = make(id + 3, Marker::SPHERE, 1, .65F, 0, "recovery_candidates");
      camera.pose.position = out.camera_positions[i]; camera.scale.x = camera.scale.y = camera.scale.z = .06;
      array.markers.push_back(camera);
    }
    auto legend_position = out.current_pose.position;
    legend_position.x -= 1.35; legend_position.y -= 1.15;
    text(9, legend_position, legend, .9F, .9F, .9F);
    markers_->publish(array);
  }

  RecoveryPreviewConfig config_;
  RecoveryContourConfig contour_config_;
  RecoveryContourConfig approach_contour_config_;
  std::string fixed_frame_, base_frame_, camera_frame_, request_error_{"no_request"};
  double scan_timeout_, max_scan_age_, future_tolerance_, tf_wait_, request_timeout_;
  double association_radius_, range_min_, range_max_;
  std::optional<Request> request_, request_context_;
  std::map<std::pair<std::string, int64_t>, RequestRecord> request_records_;
  uint64_t epoch_{0};
  std::optional<msg::PlateObservation> plate_;
  Scan::ConstSharedPtr latest_scan_, pending_;
  Steady::time_point request_received_, scan_received_, plate_received_, pending_started_, last_publish_;
  int64_t last_now_{0}, previous_scan_stamp_{0};
  bool last_valid_{false};
  std::string last_status_, last_request_id_;
  std_msgs::msg::Header last_header_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<Output>::SharedPtr output_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Subscription<Request>::SharedPtr requests_;
  rclcpp::Subscription<Scan>::SharedPtr scans_;
  rclcpp::Subscription<msg::PlateObservation>::SharedPtr plates_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace amr_reobservation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {rclcpp::spin(std::make_shared<amr_reobservation::RecoveryPreviewNode>());}
  catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("recovery_preview"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
