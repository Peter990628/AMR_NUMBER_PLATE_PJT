#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <string>
#include "amr_reobservation/srv/assess_plate_window.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
using Window = amr_reobservation::srv::AssessPlateWindow;
using Sample = amr_reobservation::msg::PlateWindowSample;
using Box = amr_reobservation::msg::PlateObservation;

bool simple_plate(const std::string & s)
{
  // The collector provides NFC text without whitespace. One Hangul syllable
  // (U+AC00..U+D7A3) is three UTF-8 bytes between the two digit groups.
  if (s.size() != 9 && s.size() != 10) {return false;}
  const size_t n = s.size() - 7;
  const auto digit = [](char c) {return c >= '0' && c <= '9';};
  if (!std::all_of(s.begin(), s.begin() + n, digit) ||
    !std::all_of(s.begin() + n + 3, s.end(), digit)) {return false;}
  const auto a = static_cast<unsigned char>(s[n]);
  const auto b = static_cast<unsigned char>(s[n+1]);
  const auto c = static_cast<unsigned char>(s[n+2]);
  const int cp = ((a & 15) << 12) | ((b & 63) << 6) | (c & 63);
  return (a & 240) == 224 && (b & 192) == 128 && (c & 192) == 128 &&
         cp >= 0xac00 && cp <= 0xd7a3;
}

double iou(const Box & a, const Box & b)
{
  const auto w = std::max(0., std::min(a.x+a.width,b.x+b.width)-std::max(a.x,b.x));
  const auto h = std::max(0., std::min(a.y+a.height,b.y+b.height)-std::max(a.y,b.y));
  const double total = a.width*a.height+b.width*b.height-w*h;
  return total > 0 ? w*h/total : 0.;
}

void assess(const Window::Request::SharedPtr req, Window::Response::SharedPtr out)
{
  out->consecutive_required = 3;
  out->min_ocr_confidence = .8;
  out->max_source_gap_sec = 1.5;
  out->min_box_iou = .3;
  if (!req->capture_valid) {out->reason = "capture_invalid"; return;}
  if (!req->expected_count || req->samples.size() != req->expected_count) {
    out->reason = "incomplete_slot_sequence"; return;
  }
  std::set<std::string> ids;
  const Sample * last_source = nullptr;
  int64_t last_ns = -1;
  bool any_recognized = false, any_valid = false;
  std::vector<uint32_t> streak;
  std::vector<uint32_t> confirmed;
  std::string confirmed_text;
  for (size_t i = 0; i < req->samples.size(); ++i) {
    const auto & s = req->samples[i];
    const auto & o = s.observation;
    if (s.slot != i || s.sample_id.empty() || !ids.insert(s.sample_id).second) {
      out->reason = "incomplete_slot_sequence"; return;
    }
    if (s.status == "missing_image" || s.status == "missing_pair") {streak.clear(); continue;}
    if (!o.input_valid || o.header.frame_id.empty() || o.header.stamp.sec < 0 ||
      o.header.stamp.nanosec >= 1000000000u || !o.image_width || !o.image_height ||
      (o.detected ? s.status != "recognized" : s.status != "not_detected")) {
      out->reason = "capture_invalid"; return;
    }
    const auto ns = rclcpp::Time(o.header.stamp).nanoseconds();
    if (ns <= last_ns) {out->reason = "duplicate_or_reordered_source"; return;}
    if (last_source && (last_source->observation.header.frame_id != o.header.frame_id ||
      last_source->observation.image_width != o.image_width ||
      last_source->observation.image_height != o.image_height)) {
      out->reason = "stream_changed"; return;
    }
    any_valid = true;
    any_recognized |= o.detected;
    const bool good = o.detected && s.ocr_confidence >= out->min_ocr_confidence &&
      s.ocr_confidence <= 1. && simple_plate(s.normalized_text);
    if (!good) {
      streak.clear();
    } else {
      if (!streak.empty() && (s.barrier_since_previous ||
        s.normalized_text != last_source->normalized_text ||
        (ns-last_ns)*1e-9 > out->max_source_gap_sec ||
        iou(o,last_source->observation) < out->min_box_iou)) {streak.clear();}
      streak.push_back(s.slot);
      if (confirmed.empty() && streak.size() == out->consecutive_required) {
        confirmed = streak;
        confirmed_text = s.normalized_text;
      }
    }
    last_ns = ns;
    last_source = &s;
  }
  out->success = !confirmed.empty();
  out->supporting_slots = out->success ? confirmed : streak;
  if (out->success) {
    out->text = confirmed_text;
    out->reason = "consistent_ocr";
  } else if (!any_valid) {out->reason = "no_valid_observations";}
  else {out->reason = any_recognized ? "no_consistent_read" : "not_detected";}
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("plate_window_decision");
  auto service = node->create_service<Window>("/plate/assess_window", assess);
  rclcpp::spin(node);
  rclcpp::shutdown();
}
