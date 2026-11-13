#include "amr_reobservation/plate_detector.hpp"
#include <iomanip>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char ** argv)
{
  if (argc < 3 || argc > 4) {
    std::cerr << "Usage: plate_image_probe MODEL.onnx IMAGE [ANNOTATED.png]\n";
    return 2;
  }
  try {
    cv::setNumThreads(2);
    amr_reobservation::PlateDetector detector(argv[1]);
    cv::Mat image = cv::imread(argv[2]);
    auto boxes = detector.detect(image);
    amr_reobservation::PlateObservationEvaluator evaluator;
    auto result = evaluator.evaluate(boxes, image.cols, image.rows);
    std::cout << std::setprecision(9) << "{\"status\":\"" << result.status << "\",\"detections\":[";
    for (size_t i = 0; i < boxes.size(); ++i) {
      const auto & b = boxes[i];
      if (i) {std::cout << ',';}
      std::cout << "{\"x\":" << b.x << ",\"y\":" << b.y << ",\"width\":" << b.width
                << ",\"height\":" << b.height << ",\"confidence\":" << b.confidence << '}';
      const cv::Rect area = cv::Rect2d(b.x, b.y, b.width, b.height) & cv::Rect2d(0, 0, image.cols, image.rows);
      cv::rectangle(image, area, {0, 255, 0}, 2);
    }
    std::cout << "]}\n";
    if (argc == 4 && !cv::imwrite(argv[3], image)) {throw std::runtime_error("Could not save annotation.");}
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n'; return 1;
  }
  return 0;
}
