#include "amr_reobservation/observation_candidates.hpp"

#include <cmath>
#include <stdexcept>

namespace amr_reobservation
{
namespace
{
constexpr double kTolerance = 1e-3;
constexpr double kMinimumLength = 1e-9;

bool finite(const Vec2 & v)
{
  return std::isfinite(v[0]) && std::isfinite(v[1]);
}

double dot(const Vec2 & a, const Vec2 & b)
{
  return a[0] * b[0] + a[1] * b[1];
}

ObservationCandidate make_candidate(
  const Vec2 & endpoint, const Vec2 & tangent, const Vec2 & normal,
  double signed_extension, double standoff)
{
  ObservationCandidate candidate;
  candidate.target = endpoint;
  for (std::size_t axis = 0; axis < 2; ++axis) {
    candidate.position[axis] = endpoint[axis] + signed_extension * tangent[axis] +
      standoff * normal[axis];
  }
  const Vec2 ray{
    endpoint[0] - candidate.position[0], endpoint[1] - candidate.position[1]};
  if (!finite(candidate.position) || !finite(ray) ||
    !(std::hypot(ray[0], ray[1]) > 0.0))
  {
    throw std::invalid_argument("Candidate offset is not representable at the supplied coordinates");
  }
  candidate.yaw = std::atan2(ray[1], ray[0]);
  return candidate;
}
}  // namespace

std::array<ObservationCandidate, 2> make_observation_candidates(
  const CandidateSide & side, const CandidateConfig & config)
{
  if (!std::isfinite(config.extension_m) || config.extension_m <= 0.0 ||
    !std::isfinite(config.standoff_m) || config.standoff_m <= 0.0)
  {
    throw std::invalid_argument("Candidate extension and standoff must be finite and positive");
  }
  if (!finite(side.a) || !finite(side.b) ||
    !finite(side.tangent) || !finite(side.normal))
  {
    throw std::invalid_argument("Candidate side geometry must be finite");
  }

  const double tangent_norm = std::hypot(side.tangent[0], side.tangent[1]);
  const double normal_norm = std::hypot(side.normal[0], side.normal[1]);
  if (std::abs(tangent_norm - 1.0) > kTolerance ||
    std::abs(normal_norm - 1.0) > kTolerance ||
    std::abs(dot(side.tangent, side.normal)) > kTolerance)
  {
    throw std::invalid_argument("Candidate tangent and normal must be perpendicular unit vectors");
  }

  const Vec2 delta{side.b[0] - side.a[0], side.b[1] - side.a[1]};
  const double length = std::hypot(delta[0], delta[1]);
  if (!finite(delta) || !std::isfinite(length) || length <= kMinimumLength) {
    throw std::invalid_argument("Candidate endpoints must form a finite nondegenerate segment");
  }
  const Vec2 direction{delta[0] / length, delta[1] / length};
  const double cross = direction[0] * side.tangent[1] -
    direction[1] * side.tangent[0];
  if (dot(direction, side.tangent) <= 0.0 || std::abs(cross) > kTolerance ||
    std::abs(dot(direction, side.normal)) > kTolerance)
  {
    throw std::invalid_argument("Endpoint a-to-b direction must agree with the supplied tangent");
  }

  // Halve before adding so otherwise valid large same-sign coordinates do not
  // overflow solely while computing the midpoint.
  const Vec2 toward_sensor{
    -(0.5 * side.a[0] + 0.5 * side.b[0]),
    -(0.5 * side.a[1] + 0.5 * side.b[1])};
  const double sensor_side = dot(side.normal, toward_sensor);
  if (!std::isfinite(sensor_side) || sensor_side <= 0.0) {
    throw std::invalid_argument("Candidate normal must point from the line toward the sensor origin");
  }

  return {
    make_candidate(side.a, side.tangent, side.normal, -config.extension_m, config.standoff_m),
    make_candidate(side.b, side.tangent, side.normal, config.extension_m, config.standoff_m)};
}

}  // namespace amr_reobservation
