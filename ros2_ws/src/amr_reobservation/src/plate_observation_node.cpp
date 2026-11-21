#include "amr_reobservation/plate_detector.hpp"
#include "amr_reobservation/msg/plate_observation.hpp"

#include "amr_reobservation/srv/inspect_plate_frame.hpp"
#include "amr_reobservation/srv/read_plate_crop.hpp"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <opencv2/imgproc.hpp>
#include "cv_bridge/cv_bridge.h"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace amr_reobservation
{
class PlateObservationNode : public rclcpp::Node
{
  using Output = msg::PlateObservation;
  using Inspect = srv::InspectPlateFrame;
  using ReadCrop = srv::ReadPlateCrop;
  using Steady = std::chrono::steady_clock;
public:
  PlateObservationNode() : Node("plate_observation")
  {
    const auto input = parameter<std::string>("input_topic", "/front_stereo_camera/left/image_raw");
    const auto result = parameter<std::string>("observation_topic", "/plate/observation");
    const auto debug = parameter<std::string>("debug_topic", "/debug/plate_observation");
    const auto model_path = parameter<std::string>("model_path", "");
    PlateDetectorConfig d;
    d.input_size = parameter("input_size", 640);
    d.rtmdet_score_threshold = parameter("rtmdet_score_threshold", 0.5);
    d.num_classes = parameter("num_classes", 4);
    d.plate_class_id = parameter("plate_class_id", 2);
    d.inference_threads = parameter("inference_threads", 2);
    d.score_threshold = parameter("detector_score_threshold", 0.25);
    d.nms_iou_threshold = parameter("nms_iou_threshold", 0.45);
    PlateObservationConfig q;
    q.min_confidence = parameter("min_confidence", 0.5);
    q.min_width_px = parameter("min_width_px", 80.0);
    q.min_height_px = parameter("min_height_px", 16.0);
    q.border_margin_ratio = parameter("border_margin_ratio", 0.02);
    q.min_tracking_iou = parameter("min_tracking_iou", 0.3);
    q.required_consecutive = parameter("required_consecutive", 3);
    quality_config_ = q;
    evaluator_ = PlateObservationEvaluator(q);
    timeout_ = parameter("image_timeout_sec", 1.0);
    max_age_ = parameter("max_image_age_sec", 0.5);
    future_tolerance_ = parameter("future_tolerance_sec", 0.1);
    const int threads = parameter("opencv_threads", 2);
    if (!std::isfinite(timeout_) || timeout_ <= 0 || timeout_ > 60 ||
      !std::isfinite(max_age_) || max_age_ <= 0 || max_age_ > 60 ||
      !std::isfinite(future_tolerance_) || future_tolerance_ < 0 || future_tolerance_ > 1 ||
      threads < 1 || threads > 16 || d.score_threshold > q.min_confidence ||
      input.empty() || result.empty() || debug.empty() || input == debug)
    {
      throw std::invalid_argument("Invalid timing/thread/topic settings or detector threshold > quality threshold.");
    }
    cv::setNumThreads(threads);
    detector_ = std::make_unique<PlateDetector>(model_path, d);
    publisher_ = create_publisher<Output>(result, rclcpp::QoS(1));
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(debug, rclcpp::SensorDataQoS().keep_last(1));
    // Keep ROS parameter services responsive while a camera frame is in inference.
    image_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions image_options;
    image_options.callback_group = image_group_;
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input, rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Image::ConstSharedPtr image) {on_image(image);}, image_options);
    // Dedicated callback group and a two-thread executor keep the watchdog runnable during inference.
    watchdog_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    inspect_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    ocr_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    ocr_ = create_client<ReadCrop>("/plate/read_crop", rmw_qos_profile_services_default, ocr_group_);
    inspect_service_ = create_service<Inspect>("/plate/inspect_frame",
      [this](const Inspect::Request::SharedPtr req, Inspect::Response::SharedPtr res) {
        inspect(*req, *res);
      }, rmw_qos_profile_services_default, inspect_group_);
    last_receive_ = Steady::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {watchdog();}, watchdog_group_);
    RCLCPP_INFO(get_logger(), "Plate detector ready: %s -> %s; CPU, %d consecutive frames",
      input.c_str(), result.c_str(), q.required_consecutive);
  }

private:
  template<typename T> T parameter(const std::string & name, const T & value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = "Startup setting; restart the node to apply changes.";
    return declare_parameter<T>(name, value, descriptor);
  }

  static double elapsed(Steady::time_point start)
  {
    return std::chrono::duration<double>(Steady::now() - start).count();
  }

  std::string time_error(const std_msgs::msg::Header & header)
  {
    if (header.stamp.sec < 0 || header.stamp.nanosec >= 1000000000U) {return "invalid_stamp";}
    const double age = (now() - rclcpp::Time(header.stamp, get_clock()->get_clock_type())).seconds();
    if (age > max_age_) {return "stale_image";}
    if (age < -future_tolerance_) {return "future_image";}
    return {};
  }

  Output base_message(const std_msgs::msg::Header & header, uint32_t width, uint32_t height)
  {
    Output output;
    output.header = header;
    output.image_width = width;
    output.image_height = height;
    return output;
  }

  Output observation(const std_msgs::msg::Header & header, int width, int height,
    const PlateObservationResult & r, double ms)
  {
    auto o = base_message(header, width, height);
    o.input_valid = true;
    o.detected = r.detected; o.usable = r.usable; o.status = r.status;
    o.x = r.box.x; o.y = r.box.y; o.width = r.box.width; o.height = r.box.height;
    o.confidence = r.box.confidence; o.border_margin_ratio = r.border_margin_ratio;
    o.consecutive_count = r.consecutive_count; o.processing_ms = ms;
    return o;
  }

  std::vector<PlateBox> detect(const cv::Mat & image)
  {
    std::lock_guard<std::mutex> guard(inference_mutex_);
    return detector_->detect(image);
  }

  void inspect(const Inspect::Request & req, Inspect::Response & res)
  {
    const auto & source = req.image;
    res.sample_id = req.sample_id;
    res.observation.header = source.header;
    res.status = "invalid_image";
    if (req.sample_id.empty() || source.header.frame_id.empty() || source.format != "png" ||
      source.data.empty() || source.data.size() > 24 * 1024 * 1024) {return;}
    try {
      const auto started = Steady::now();
      const auto image = cv::imdecode(source.data, cv::IMREAD_COLOR);
      if (image.empty() || image.cols > 8192 || image.rows > 8192) {return;}
      // Per-request detection never advances or resets the live tracking streak.
      PlateObservationEvaluator single_frame(quality_config_);
      const auto r = single_frame.evaluate(detect(image), image.cols, image.rows);
      res.observation = observation(source.header, image.cols, image.rows, r, elapsed(started)*1000);
      if (!r.detected) {res.status = "not_detected"; return;}
      const auto & b = r.box;
      const int x1 = std::clamp(static_cast<int>(std::floor(b.x)), 0, image.cols);
      const int y1 = std::clamp(static_cast<int>(std::floor(b.y)), 0, image.rows);
      const int x2 = std::clamp(static_cast<int>(std::ceil(b.x+b.width)), 0, image.cols);
      const int y2 = std::clamp(static_cast<int>(std::ceil(b.y+b.height)), 0, image.rows);
      if (x1 >= x2 || y1 >= y2) {res.status = "invalid_bbox"; return;}
      if (!ocr_->service_is_ready()) {res.status = "ocr_unavailable"; return;}
      auto request = std::make_shared<ReadCrop::Request>();
      request->sample_id = req.sample_id;
      request->crop = *cv_bridge::CvImage(source.header, "bgr8",
        image(cv::Rect(x1, y1, x2-x1, y2-y1))).toImageMsg();
      auto pending = ocr_->async_send_request(request);
      if (pending.wait_for(std::chrono::seconds(6)) != std::future_status::ready) {
        ocr_->remove_pending_request(pending);
        res.status = "ocr_timeout"; return;
      }
      const auto answer = pending.get();
      if (answer->sample_id != req.sample_id || answer->header != source.header) {
        res.status = "ocr_source_mismatch"; return;
      }
      res.status = answer->status;
      if (res.status == "recognized") {
        res.text = answer->text; res.ocr_confidence = answer->confidence;
        res.recognition_ms = answer->recognition_ms;
        res.crop_pixels_sha256 = answer->crop_pixels_sha256;
      }
    } catch (const std::exception & e) {
      res.status = "inspection_error";
      RCLCPP_WARN(get_logger(), "Sample %s: %s", req.sample_id.c_str(), e.what());
    }
  }

  // All publication/state commits occur under mutex_: timeout cannot be followed by
  // a late usable result from an inference that started before the timeout.
  void publish(Output output, const cv::Mat & source = {})
  {
    publisher_->publish(output);
    cv::Mat overlay = source.empty() ? cv::Mat(240, 640, CV_8UC3, cv::Scalar(0, 0, 0)) : source.clone();
    const cv::Scalar color = output.usable ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 180, 255);
    if (output.detected && !source.empty()) {
      const auto clamp_x = [&](double x) {return static_cast<int>(std::clamp(x, 0.0, double(overlay.cols - 1)));};
      const auto clamp_y = [&](double y) {return static_cast<int>(std::clamp(y, 0.0, double(overlay.rows - 1)));};
      cv::rectangle(overlay, {clamp_x(output.x), clamp_y(output.y)},
        {clamp_x(output.x + output.width), clamp_y(output.y + output.height)}, color, 2);
    }
    std::string text = output.status + " | consecutive=" + std::to_string(output.consecutive_count);
    if (output.detected) {text += " | score=" + std::to_string(output.confidence).substr(0, 4);}
    cv::putText(overlay, text, {10, 25}, cv::FONT_HERSHEY_SIMPLEX, .6, color, 2);
    image_publisher_->publish(*cv_bridge::CvImage(output.header, "bgr8", overlay).toImageMsg());
  }

  void fail(const std_msgs::msg::Header & header, uint32_t w, uint32_t h,
    const std::string & status, double processing_ms = 0)
  {
    evaluator_.reset();
    auto output = base_message(header, w, h);
    output.status = status;
    output.processing_ms = processing_ms;
    publish(output);
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & image)
  {
    const auto start = Steady::now();
    uint64_t generation;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (elapsed(last_receive_) > timeout_) {evaluator_.reset();}
      last_receive_ = start;
      timed_out_ = false;
      received_ = true;
      last_header_ = image->header;
      last_width_ = image->width;
      last_height_ = image->height;
      generation = generation_;
      if (image->header.frame_id.empty()) {fail(image->header, image->width, image->height, "empty_frame"); return;}
      if (!image->width || !image->height || image->width > 8192 || image->height > 8192 ||
        image->data.size() != static_cast<size_t>(image->height) * image->step)
      {
        fail(image->header, image->width, image->height, "invalid_image"); return;
      }
      const auto error = time_error(image->header);
      if (!error.empty()) {fail(image->header, image->width, image->height, error); return;}
      const auto stamp = rclcpp::Time(image->header.stamp).nanoseconds();
      if (have_stamp_ && (frame_ != image->header.frame_id || width_ != image->width || height_ != image->height)) {
        evaluator_.reset(); have_stamp_ = false;
      }
      if (have_stamp_ && stamp <= last_stamp_) {
        const auto reason = stamp == last_stamp_ ? "duplicate_stamp" : "time_reset";
        // Rebase after a backward jump so the next increasing frame can recover.
        last_stamp_ = stamp;
        fail(image->header, image->width, image->height, reason); return;
      }
      last_stamp_ = stamp; have_stamp_ = true;
      frame_ = image->header.frame_id; width_ = image->width; height_ = image->height;
    }
    cv_bridge::CvImageConstPtr converted;
    std::vector<PlateBox> boxes;
    std::string processing_error;
    try {converted = cv_bridge::toCvShare(image, "bgr8");}
    catch (const std::exception & e) {
      processing_error = "invalid_image";
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000, "Image conversion failed: %s", e.what());
    }
    if (processing_error.empty()) {
      try {boxes = detect(converted->image);}
      catch (const std::exception & e) {
        processing_error = "inference_error";
        RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000, "Inference failed: %s", e.what());
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const double ms = elapsed(start) * 1000;
    if (generation != generation_ || elapsed(start) > timeout_) {
      fail(image->header, image->width, image->height, "processing_timeout", ms); return;
    }
    if (!processing_error.empty()) {fail(image->header, image->width, image->height, processing_error, ms); return;}
    const auto error = time_error(image->header);
    if (!error.empty()) {fail(image->header, image->width, image->height, error, ms); return;}
    const auto result = evaluator_.evaluate(boxes, static_cast<int>(image->width), static_cast<int>(image->height));
    auto output = observation(image->header, image->width, image->height, result, ms);
    publish(output, converted->image);
    RCLCPP_INFO_THROTTLE(get_logger(), steady_clock_, 5000, "plate=%s score=%.3f size=%.0fx%.0f processing=%.1fms",
      output.status.c_str(), output.confidence, output.width, output.height, output.processing_ms);
  }

  void watchdog()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_) {
      fail(std_msgs::msg::Header{}, 0, 0, "no_image");
    } else if (elapsed(last_receive_) > timeout_) {
      if (!timed_out_) {++generation_; timed_out_ = true;}
      fail(last_header_, last_width_, last_height_, "input_timeout");
    } else {
      // Invalidate a previously usable result after ROS time advances or resets,
      // even if an image callback is currently busy doing inference.
      const auto error = time_error(last_header_);
      if (!error.empty()) {++generation_; fail(last_header_, last_width_, last_height_, error);}
    }
  }

  std::unique_ptr<PlateDetector> detector_;
  PlateObservationEvaluator evaluator_;
  PlateObservationConfig quality_config_;
  rclcpp::CallbackGroup::SharedPtr inspect_group_, ocr_group_;
  rclcpp::Client<ReadCrop>::SharedPtr ocr_;
  rclcpp::Service<Inspect>::SharedPtr inspect_service_;
  std::mutex inference_mutex_, mutex_;
  double timeout_{1}, max_age_{.5}, future_tolerance_{.1};
  Steady::time_point last_receive_;
  bool received_{false}, timed_out_{false}, have_stamp_{false};
  uint64_t generation_{0};
  int64_t last_stamp_{0};
  uint32_t width_{0}, height_{0}, last_width_{0}, last_height_{0};
  std::string frame_;
  std_msgs::msg::Header last_header_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  rclcpp::Publisher<Output>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr image_group_, watchdog_group_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace amr_reobservation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<amr_reobservation::PlateObservationNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("plate_observation"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
