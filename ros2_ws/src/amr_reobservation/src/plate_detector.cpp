#include "amr_reobservation/plate_detector.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <opencv2/imgproc.hpp>
#include <onnxruntime_cxx_api.h>
#include <array>

namespace amr_reobservation
{
namespace
{
void validate(const PlateDetectorConfig & c)
{
  if (c.input_size < 32 || c.input_size > 2048 || c.input_size % 32 != 0 ||
    c.num_classes < 1 || c.num_classes > 1000 || c.plate_class_id < 0 || c.plate_class_id >= c.num_classes ||
    c.inference_threads < 1 || c.inference_threads > 16 ||
    !std::isfinite(c.score_threshold) || c.score_threshold <= 0 || c.score_threshold > 1 ||
    !std::isfinite(c.rtmdet_score_threshold) || c.rtmdet_score_threshold <= 0 || c.rtmdet_score_threshold > 1 ||
    !std::isfinite(c.nms_iou_threshold) || c.nms_iou_threshold <= 0 || c.nms_iou_threshold >= 1)
  {
    throw std::invalid_argument("Invalid plate detector settings.");
  }
}

constexpr int kRtmdetSize = 960;
constexpr std::array<const char *, 6> kRtmdetOutputs{
  "cls_s8", "cls_s16", "cls_s32", "bbox_s8", "bbox_s16", "bbox_s32"};

std::vector<PlateBox> decode_rtmdet(
  const std::vector<Ort::Value> & outputs, double scale, double threshold)
{
  std::vector<cv::Rect2d> boxes;
  std::vector<float> scores;
  for (int level = 0; level < 3; ++level) {
    const int stride = 8 << level;
    const int side = kRtmdetSize / stride;
    const int count = side * side;
    auto logits = outputs[level].GetTensorData<float>();
    auto distances = outputs[level + 3].GetTensorData<float>();
    for (int i = 0; i < count; ++i) {
      const float score = 1.F / (1.F + std::exp(-logits[i]));
      if (!std::isfinite(score) || score < threshold) {continue;}
      const double x = (i % side) * stride, y = (i / side) * stride;
      const double left = distances[i], top = distances[count + i];
      const double width = (left + distances[2 * count + i]) / scale;
      const double height = (top + distances[3 * count + i]) / scale;
      if (!std::isfinite(width) || !std::isfinite(height) || width <= 0 || height <= 0) {continue;}
      boxes.emplace_back((x - left) / scale, (y - top) / scale, width, height);
      scores.push_back(score);
    }
  }
  std::vector<int> keep;
  cv::dnn::NMSBoxes(boxes, scores, 0.F, 0.6F, keep);
  std::vector<PlateBox> result;
  for (const int i : keep) {
    const auto & b = boxes[i];
    if (b.width >= 2.0 * b.height) {
      result.push_back({b.x, b.y, b.width, b.height, scores[i]});
    }
  }
  return result;
}
}  // namespace

LetterboxGeometry letterbox_geometry(int width, int height, int size)
{
  if (width <= 0 || height <= 0 || size <= 0) {throw std::invalid_argument("Invalid letterbox size.");}
  const double scale = std::min(static_cast<double>(size) / width, static_cast<double>(size) / height);
  const int w = std::max(1, static_cast<int>(std::round(width * scale)));
  const int h = std::max(1, static_cast<int>(std::round(height * scale)));
  return {scale, (size - w) / 2, (size - h) / 2, w, h};
}

std::vector<PlateBox> decode_plate_output(
  const cv::Mat & output, const LetterboxGeometry & g, const PlateDetectorConfig & config)
{
  validate(config);
  if (output.dims != 3 || output.size[0] != 1 || output.size[1] != 4 + config.num_classes ||
    output.size[2] <= 0 || output.type() != CV_32F || !output.isContinuous() ||
    !std::isfinite(g.scale) || g.scale <= 0)
  {
    throw std::runtime_error("Unsupported ONNX output; expected [1,4+classes,anchors] float32.");
  }
  const int n = output.size[2];
  const float * data = output.ptr<float>();
  std::vector<cv::Rect2d> boxes;
  std::vector<float> scores;
  for (int i = 0; i < n; ++i) {
    int best_class = 0;
    float score = -1;
    for (int c = 0; c < config.num_classes; ++c) {
      const float value = data[(4 + c) * n + i];
      if (!std::isfinite(value) || value < 0 || value > 1) {
        throw std::runtime_error("Nonfinite or invalid ONNX class score.");
      }
      if (value > score) {best_class = c; score = value;}
    }
    // Plate must win the class competition; Not_Plate is never interpreted as Plate.
    if (best_class != config.plate_class_id || score < config.score_threshold) {continue;}
    const double cx = data[i], cy = data[n + i], w = data[2 * n + i], h = data[3 * n + i];
    if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(w) || !std::isfinite(h) || w <= 0 || h <= 0) {
      throw std::runtime_error("Invalid ONNX bounding box.");
    }
    boxes.emplace_back((cx - w / 2 - g.left) / g.scale, (cy - h / 2 - g.top) / g.scale,
      w / g.scale, h / g.scale);
    scores.push_back(score);
  }
  std::vector<int> keep;
  cv::dnn::NMSBoxes(boxes, scores, static_cast<float>(config.score_threshold),
    static_cast<float>(config.nms_iou_threshold), keep);
  std::vector<PlateBox> result;
  result.reserve(keep.size());
  for (const int i : keep) {
    const auto & b = boxes[i];
    result.push_back({b.x, b.y, b.width, b.height, scores[i]});
  }
  return result;
}

struct PlateDetector::Impl
{
  Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "amr_plate"};
  Ort::SessionOptions options;
  Ort::Session session{nullptr};
  std::string input_name, output_name;
  bool rtmdet{false};
  int size;
  Impl(const std::string & path, const PlateDetectorConfig & config) : size(config.input_size)
  {
    options.SetIntraOpNumThreads(config.inference_threads);
    options.SetInterOpNumThreads(1);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session = Ort::Session(env, path.c_str(), options);
    Ort::AllocatorWithDefaultOptions allocator;
    rtmdet = session.GetOutputCount() == kRtmdetOutputs.size();
    if (rtmdet) {
      size = kRtmdetSize;
      for (size_t i = 0; i < kRtmdetOutputs.size(); ++i) {
        const int side = size / (8 << (i % 3));
        auto type = session.GetOutputTypeInfo(i);
        auto head = type.GetTensorTypeAndShapeInfo();
        const std::vector<int64_t> shape{1, i < 3 ? 1 : 4, side, side};
        if (std::string(session.GetOutputNameAllocated(i, allocator).get()) != kRtmdetOutputs[i] ||
          head.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || head.GetShape() != shape)
        {
          throw std::runtime_error("Unexpected KR RTMDet v4 output name, type or shape.");
        }
      }
    }
    if (session.GetInputCount() != 1 || (!rtmdet && session.GetOutputCount() != 1)) {
      throw std::runtime_error("Unsupported plate detector ONNX signature.");
    }
    auto info = session.GetInputTypeInfo(0);
    auto tensor = info.GetTensorTypeAndShapeInfo();
    const std::vector<int64_t> expected{1, 3, size, size};
    if (tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || tensor.GetShape() != expected) {
      throw std::runtime_error("Model must have static float32 input [1,3,input_size,input_size].");
    }
    input_name = session.GetInputNameAllocated(0, allocator).get();
    output_name = session.GetOutputNameAllocated(0, allocator).get();
  }
};

PlateDetector::~PlateDetector() = default;

PlateDetector::PlateDetector(const std::string & path, const PlateDetectorConfig & config)
: config_(config)
{
  validate(config);
  if (path.empty() || !std::filesystem::is_regular_file(path)) {
    throw std::invalid_argument("model_path must point to the exported .onnx file.");
  }
  impl_ = std::make_unique<Impl>(path, config);
  // Fail at startup for incompatible output contracts or unsupported operators.
  detect(cv::Mat(config.input_size, config.input_size, CV_8UC3, cv::Scalar(114, 114, 114)));
}

std::vector<PlateBox> PlateDetector::detect(const cv::Mat & bgr)
{
  if (bgr.empty() || bgr.type() != CV_8UC3) {throw std::invalid_argument("Expected nonempty BGR8 image.");}
  const int size = impl_->size;
  auto g = letterbox_geometry(bgr.cols, bgr.rows, size);
  if (impl_->rtmdet) {
    g.left = 0;
    g.top = 0;
    g.resized_width = std::max(1, cvRound(bgr.cols * g.scale));
    g.resized_height = std::max(1, cvRound(bgr.rows * g.scale));
  }
  cv::Mat resized, padded;
  cv::resize(bgr, resized, cv::Size(g.resized_width, g.resized_height));
  cv::copyMakeBorder(resized, padded, g.top, size - g.resized_height - g.top,
    g.left, size - g.resized_width - g.left, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
  cv::Mat blob;
  if (impl_->rtmdet) {
    padded.convertTo(padded, CV_32F);
    cv::subtract(padded, cv::Scalar(103.53, 116.28, 123.675), padded);
    cv::divide(padded, cv::Scalar(57.375, 57.12, 58.395), padded);
    blob = cv::dnn::blobFromImage(padded);
  } else {
    blob = cv::dnn::blobFromImage(padded, 1.0 / 255.0, {}, {}, true, false, CV_32F);
  }
  const std::array<int64_t, 4> shape{1, 3, size, size};
  auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  auto input = Ort::Value::CreateTensor<float>(memory, blob.ptr<float>(), blob.total(), shape.data(), shape.size());
  const char * input_names[] = {impl_->input_name.c_str()};
  if (impl_->rtmdet) {
    auto heads = impl_->session.Run(Ort::RunOptions{nullptr}, input_names, &input, 1,
      kRtmdetOutputs.data(), kRtmdetOutputs.size());
    return decode_rtmdet(heads, g.scale, config_.rtmdet_score_threshold);
  }
  const char * output_names[] = {impl_->output_name.c_str()};
  auto outputs = impl_->session.Run(Ort::RunOptions{nullptr}, input_names, &input, 1, output_names, 1);
  auto info = outputs.at(0).GetTensorTypeAndShapeInfo();
  const auto dimensions = info.GetShape();
  if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || dimensions.size() != 3 ||
    dimensions[0] != 1 || dimensions[1] != 4 + config_.num_classes || dimensions[2] <= 0 || dimensions[2] > 1000000)
  {
    throw std::runtime_error("Unsupported ONNX output shape/type.");
  }
  const int sizes[] = {1, static_cast<int>(dimensions[1]), static_cast<int>(dimensions[2])};
  cv::Mat output(3, sizes, CV_32F, outputs[0].GetTensorMutableData<float>());
  return decode_plate_output(output, g, config_);
}
}  // namespace amr_reobservation
