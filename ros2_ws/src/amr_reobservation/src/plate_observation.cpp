#include "amr_reobservation/plate_observation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace amr_reobservation
{
namespace
{
bool valid(const PlateBox & b)
{
  return std::isfinite(b.x) && std::isfinite(b.y) && std::isfinite(b.width) &&
         std::isfinite(b.height) && std::isfinite(b.confidence) &&
         std::isfinite(b.x + b.width) && std::isfinite(b.y + b.height) &&
         std::isfinite(b.width * b.height) && b.width > 0 && b.height > 0 &&
         b.confidence >= 0 && b.confidence <= 1;
}

double iou(const PlateBox & a, const PlateBox & b)
{
  const double w = std::max(0.0, std::min(a.x + a.width, b.x + b.width) - std::max(a.x, b.x));
  const double h = std::max(0.0, std::min(a.y + a.height, b.y + b.height) - std::max(a.y, b.y));
  const double intersection = w * h;
  return intersection / (a.width * a.height + b.width * b.height - intersection);
}
}  // namespace

PlateObservationEvaluator::PlateObservationEvaluator(const PlateObservationConfig & config)
: config_(config)
{
  if (!std::isfinite(config.min_confidence) || config.min_confidence <= 0 || config.min_confidence > 1 ||
    !std::isfinite(config.min_width_px) || config.min_width_px <= 0 ||
    !std::isfinite(config.min_height_px) || config.min_height_px <= 0 ||
    !std::isfinite(config.border_margin_ratio) || config.border_margin_ratio < 0 ||
    config.border_margin_ratio >= 0.5 || !std::isfinite(config.min_tracking_iou) ||
    config.min_tracking_iou <= 0 || config.min_tracking_iou > 1 ||
    config.required_consecutive < 1 || config.required_consecutive > 1000)
  {
    throw std::invalid_argument("Invalid plate observation quality thresholds.");
  }
}

void PlateObservationEvaluator::reset()
{
  count_ = 0;
  previous_ = {};
}

PlateObservationResult PlateObservationEvaluator::evaluate(
  const std::vector<PlateBox> & boxes, int width, int height)
{
  PlateObservationResult result;
  auto fail = [&](const std::string & status) {
      reset();
      result.status = status;
      return result;
    };
  if (width <= 0 || height <= 0) {return fail("invalid_image");}
  if (boxes.empty()) {return fail("not_detected");}
  if (!std::all_of(boxes.begin(), boxes.end(), valid)) {return fail("invalid_detection");}
  const auto best = std::max_element(boxes.begin(), boxes.end(),
      [](const PlateBox & a, const PlateBox & b) {return a.confidence < b.confidence;});
  result.detected = true;
  result.box = *best;
  // Signed, axis-normalized distance from the original UNCLIPPED box to each edge.
  result.border_margin_ratio = std::min({best->x / width, best->y / height,
      (width - best->x - best->width) / width, (height - best->y - best->height) / height});
  if (best->confidence < config_.min_confidence) {return fail("low_confidence");}
  if (std::count_if(boxes.begin(), boxes.end(), [&](const PlateBox & b) {
      return b.confidence >= config_.min_confidence;
    }) > 1)
  {
    return fail("ambiguous");
  }
  // Crossing a boundary remains a risk even when configured margin is zero.
  if (result.border_margin_ratio <= 0 || result.border_margin_ratio < config_.border_margin_ratio) {
    return fail("border_risk");
  }
  if (best->width < config_.min_width_px || best->height < config_.min_height_px) {
    return fail("too_small");
  }
  if (count_ > 0 && iou(previous_, *best) < config_.min_tracking_iou) {reset();}
  count_ = std::min(count_ + 1, config_.required_consecutive);
  previous_ = *best;
  result.consecutive_count = count_;
  result.usable = count_ >= config_.required_consecutive;
  result.status = result.usable ? "usable" : "warming_up";
  return result;
}
}  // namespace amr_reobservation
