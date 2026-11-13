// Offline adapter for production geometry: one whitespace-delimited record per
// input line, one JSON object per output line. No ROS or simulation side effects.
// estimate: sequence count angle_min angle_increment range_min range_max ranges...
// gate: row stamp_ns valid [position_xyz target_xyz yaw (twice)]
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "amr_reobservation/candidate_stability.hpp"
#include "amr_reobservation/scan_preprocessor.hpp"
#include "amr_reobservation/vehicle_side_estimator.hpp"

namespace
{
namespace amr = amr_reobservation;

std::string token(std::istream & input)
{
  std::string value;
  if (!(input >> value)) {throw std::runtime_error("Truncated input record");}
  return value;
}

std::int64_t integer(std::istream & input)
{
  const auto value = token(input);
  std::size_t consumed = 0;
  const auto result = std::stoll(value, &consumed);
  if (consumed != value.size()) {throw std::runtime_error("Invalid integer token");}
  return result;
}

double number(std::istream & input)
{
  const auto value = token(input);
  std::size_t consumed = 0;
  const double result = std::stod(value, &consumed);
  if (consumed != value.size()) {throw std::runtime_error("Invalid numeric token");}
  return result;
}

void end_record(std::istream & input)
{
  std::string extra;
  if (input >> extra) {throw std::runtime_error("Unexpected trailing input fields");}
}

void finite(std::ostream & output, double value)
{
  if (!std::isfinite(value)) {throw std::runtime_error("Nonfinite output value");}
  output << value;
}

template<typename Container>
void indices(std::ostream & output, const Container & values)
{
  output << '[';
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i) {output << ',';}
    output << values[i];
  }
  output << ']';
}

void vector2(std::ostream & output, const char * key, const std::array<double, 2> & value)
{
  output << '"' << key << "\":[";
  finite(output, value[0]);
  output << ',';
  finite(output, value[1]);
  output << "],";
}

void estimate(std::istream & input, std::ostream & output)
{
  const auto sequence = integer(input);
  const auto count = integer(input);
  if (count < 0 || count > 10000000) {throw std::runtime_error("Invalid scan size");}
  const amr::ScanGeometry geometry{
    number(input), number(input), number(input), number(input)};
  std::vector<float> ranges(static_cast<std::size_t>(count));
  for (auto & range : ranges) {
    const auto value = token(input);
    std::size_t consumed = 0;
    range = std::stof(value, &consumed);
    if (consumed != value.size()) {throw std::runtime_error("Invalid range token");}
  }
  end_record(input);
  const amr::ScanPreprocessor preprocessor;
  amr::SideEstimatorConfig config;
  config.refinement_limit = 20;
  const amr::VehicleSideEstimator estimator(config);
  const auto scan = preprocessor.process(ranges, geometry);
  const auto result = estimator.estimate(scan.points, geometry, ranges.size());
  output << "{\"scan_sequence\":" << sequence << ",\"status\":\"" <<
    amr::side_status_name(result.status) << "\",\"points\":[";
  for (std::size_t i = 0; i < scan.points.size(); ++i) {
    const auto & point = scan.points[i];
    if (i) {output << ',';}
    output << '[' << point.beam_index << ',';
    finite(output, point.x);
    output << ',';
    finite(output, point.y);
    output << ']';
  }
  output << "],\"clusters\":[";
  for (std::size_t i = 0; i < result.clusters.size(); ++i) {
    if (i) {output << ',';}
    indices(output, result.clusters[i]);
  }
  output << "],\"estimate\":";
  if (result.estimate) {
    const auto & e = *result.estimate;
    output << '{';
    vector2(output, "center", e.center);
    vector2(output, "tangent", e.tangent);
    vector2(output, "normal_toward_sensor", e.normal_toward_sensor);
    vector2(output, "endpoint_a", e.endpoint_a);
    vector2(output, "endpoint_b", e.endpoint_b);
    output << "\"observed_length_m\":";
    finite(output, e.observed_length_m);
    output << ",\"rmse_m\":";
    finite(output, e.rmse_m);
    output << ",\"inlier_ratio\":";
    finite(output, e.inlier_ratio);
    output << ",\"cluster_index\":" << e.cluster_index << ",\"inlier_indices\":";
    indices(output, e.inlier_indices);
    output << "},\"refinement_iterations\":" <<
      result.diagnostics.at(e.cluster_index).refinement_iterations;
  } else {
    output << "null,\"refinement_iterations\":null";
  }
  output << '}';
}

void gate(std::istream & input, std::ostream & output, amr::CandidateStabilityGate & filter)
{
  const auto row = integer(input);
  const auto stamp = integer(input);
  const auto valid = integer(input);
  if (valid != 0 && valid != 1) {throw std::runtime_error("valid must be 0 or 1");}
  amr::StabilityPair pair;
  if (valid) {
    for (auto & candidate : pair) {
      for (auto & coordinate : candidate.position) {coordinate = number(input);}
      for (auto & coordinate : candidate.target) {coordinate = number(input);}
      candidate.yaw = number(input);
    }
  }
  end_record(input);
  amr::CandidateStabilityDecision decision{false, "invalid_input", 0};
  if (valid) {decision = filter.update(pair, stamp);} else {filter.reset();}
  output << "{\"row\":" << row << ",\"valid\":" << (decision.valid ? 1 : 0) <<
    ",\"status\":\"" << decision.status << "\",\"consecutive\":" <<
    decision.consecutive << '}';
}
}  // namespace

int main(int argc, char ** argv)
{
  std::size_t line_number = 0;
  try {
    if (argc != 2 || (std::string(argv[1]) != "estimate" && std::string(argv[1]) != "gate")) {
      throw std::runtime_error("Usage: endpoint_support_replay estimate|gate < records.txt");
    }
    amr::CandidateStabilityGate filter;
    std::string line;
    while (std::getline(std::cin, line)) {
      ++line_number;
      if (line.find_first_not_of(" \t\r") == std::string::npos) {continue;}
      std::istringstream input(line);
      std::ostringstream output;
      output << std::setprecision(17);
      if (std::string(argv[1]) == "estimate") {estimate(input, output);}
      else {gate(input, output, filter);}
      // Buffer each record so an error cannot emit a partial JSON object.
      std::cout << output.str() << '\n';
    }
    if (std::cin.bad() || !std::cout) {throw std::runtime_error("Input/output failure");}
  } catch (const std::exception & error) {
    std::cerr << "endpoint_support_replay: line " << line_number << ": " << error.what() << '\n';
    return 1;
  }
  return 0;
}
