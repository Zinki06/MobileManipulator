#ifndef ARUCO_LOCALIZER__MARKER_GUIDANCE_HPP_
#define ARUCO_LOCALIZER__MARKER_GUIDANCE_HPP_

namespace aruco_localizer
{

double normalizeAngle(double angle);

double predictTargetBearing(
  double observed_bearing, double observed_map_direction,
  double target_map_direction);

// Return relative turns that sweep center -> one side -> the other side ->
// center. A negative signed_step starts the sweep to the robot's right.
double symmetricSearchTurn(double signed_step, int step_index);

}  // namespace aruco_localizer

#endif  // ARUCO_LOCALIZER__MARKER_GUIDANCE_HPP_
