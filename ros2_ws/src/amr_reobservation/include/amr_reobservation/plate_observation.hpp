#pragma once

#include <string>
#include <vector>

namespace amr_reobservation
{
struct PlateBox
{
  double x{0}, y{0}, width{0}, height{0}, confidence{0};
};

struct PlateObservationConfig
{
  double min_confidence{0.5};
  double min_width_px{80};
  double min_height_px{16};
  double border_margin_ratio{0.02};
  double min_tracking_iou{0.3};
  int required_consecutive{3};
};

struct PlateObservationResult
{
  bool detected{false};
  bool usable{false};
  std::string status{"not_detected"};
  PlateBox box;
  double border_margin_ratio{0};
  int consecutive_count{0};
};

// Frame-to-frame evidence only. Caller must reject stale/duplicate frames and
// call reset on time, frame, resolution or stream discontinuities.
class PlateObservationEvaluator
{
public:
  explicit PlateObservationEvaluator(const PlateObservationConfig & config = {});
  PlateObservationResult evaluate(const std::vector<PlateBox> & boxes, int width, int height);
  void reset();
private:
  PlateObservationConfig config_;
  PlateBox previous_;
  int count_{0};
};
}  // namespace amr_reobservation
