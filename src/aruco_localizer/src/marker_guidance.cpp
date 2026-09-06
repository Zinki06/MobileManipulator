#include "aruco_localizer/marker_guidance.hpp"

#include <cmath>

namespace aruco_localizer
{

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double predictTargetBearing(
  double observed_bearing, double observed_map_direction,
  double target_map_direction)
{
  return normalizeAngle(
    observed_bearing +
    normalizeAngle(target_map_direction - observed_map_direction));
}

double symmetricSearchTurn(double signed_step, int step_index)
{
  const int phase = ((step_index % 3) + 3) % 3;
  if (phase == 1) {
    return -2.0 * signed_step;
  }
  return signed_step;
}

}  // namespace aruco_localizer
