# Quality proxy for a detected plate, NOT OCR success or guaranteed full visibility.
# header is the source image header, including timeout/error notifications.
# Use input_valid AND usable AND freshness; never drive on a stale success.
std_msgs/Header header
bool input_valid
bool detected
bool usable
string status
uint32 image_width
uint32 image_height
# Unclipped pixel box. Zero on no candidate, image errors or timeout.
float64 x
float64 y
float64 width
float64 height
float64 confidence
# min(x/image_width,y/image_height,right_margin/image_width,bottom_margin/image_height)
float64 border_margin_ratio
uint32 consecutive_count
float64 processing_ms
