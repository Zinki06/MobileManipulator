#ifndef PICK_AND_PLACE__PLACE_KINEMATICS_HPP_
#define PICK_AND_PLACE__PLACE_KINEMATICS_HPP_

#include "pick_and_place/grasp_kinematics.hpp"

namespace pick_and_place
{
struct PlacePlan
{
  double pitch_degrees{0.0};
  std::vector<double> above;
  std::vector<std::vector<double>> lower;
  std::vector<std::vector<double>> retreat;
};

inline bool planPlace(double x, double y, double floor, double release_height,
  const std::vector<double> & start, PlacePlan & result)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(floor) ||
    !std::isfinite(release_height) || release_height < 0.025 || release_height > 0.06 ||
    start.size() != 4) {return false;}
  for (double q : start) {if (!std::isfinite(q)) {return false;}}
  for (double pitch = -45.0; pitch <= -15.0; pitch += 2.5) {
    PlacePlan plan;
    const double release_z = floor + release_height;
    if (release_height + fingerOffsetZ(pitch * M_PI / 180.0) < 0.008 ||
      !solve4DofIK(x, y, release_z + 0.05, plan.above, pitch, false) ||
      !fingerPathClear(start, {plan.above}, floor)) {continue;}
    bool valid = true;
    for (int i = 1; i <= 10; ++i) {
      std::vector<double> q;
      if (!solve4DofIK(x, y, release_z + 0.05 * (1.0 - i / 10.0), q, pitch, false)) {
        valid = false; break;
      }
      plan.lower.push_back(q);
    }
    if (!valid || !fingerPathClear(plan.above, plan.lower, floor)) {continue;}
    for (int i = 8; i >= 0; --i) {plan.retreat.push_back(plan.lower[i]);}
    plan.retreat.push_back(plan.above);
    if (!fingerPathClear(plan.lower.back(), plan.retreat, floor)) {continue;}
    plan.pitch_degrees = pitch;
    result = plan;
    return true;
  }
  return false;
}
}  // namespace pick_and_place
#endif
