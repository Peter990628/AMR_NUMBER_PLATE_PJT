#pragma once

#include <string>
#include <memory>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include "amr_reobservation/plate_observation.hpp"

namespace amr_reobservation
{
struct PlateDetectorConfig
{
  int input_size{640};
  int num_classes{4};
  int plate_class_id{2};
  int inference_threads{2};
  double score_threshold{0.25};
  double nms_iou_threshold{0.45};
};

struct LetterboxGeometry
{
  double scale;
  int left;
  int top;
  int resized_width;
  int resized_height;
};

LetterboxGeometry letterbox_geometry(int width, int height, int input_size);
// Accept only this project's exported [1, 4+classes, anchors] float32 YOLO output.
std::vector<PlateBox> decode_plate_output(
  const cv::Mat & output, const LetterboxGeometry & geometry, const PlateDetectorConfig & config);

class PlateDetector
{
public:
  PlateDetector(const std::string & model_path, const PlateDetectorConfig & config = {});
  ~PlateDetector();
  std::vector<PlateBox> detect(const cv::Mat & bgr);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  PlateDetectorConfig config_;
};
}  // namespace amr_reobservation
