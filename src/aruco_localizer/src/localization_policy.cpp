#include "aruco_localizer/localization_policy.hpp"

#include <cmath>
#include <algorithm>

namespace aruco_localizer
{

namespace
{
Pose2D apply(const Pose2D & transform, double x, double y)
{
  return {transform.x + std::cos(transform.yaw) * x - std::sin(transform.yaw) * y,
    transform.y + std::sin(transform.yaw) * x + std::cos(transform.yaw) * y, transform.yaw};
}
}

double correctionDisplacement(const Pose2D & before, const Pose2D & after,
  double odom_x, double odom_y)
{
  const auto old_point = apply(before, odom_x, odom_y);
  const auto new_point = apply(after, odom_x, odom_y);
  return std::hypot(new_point.x - old_point.x, new_point.y - old_point.y);
}

Pose2D boundedCorrection(const Pose2D & before, const Pose2D & measured,
  double odom_x, double odom_y, double alpha, double translation_limit, double yaw_limit)
{
  const auto old_point = apply(before, odom_x, odom_y);
  const auto new_point = apply(measured, odom_x, odom_y);
  double dx = alpha * (new_point.x - old_point.x);
  double dy = alpha * (new_point.y - old_point.y);
  const double distance = std::hypot(dx, dy);
  if (distance > translation_limit) {
    dx *= translation_limit / distance;
    dy *= translation_limit / distance;
  }
  const double error = std::atan2(std::sin(measured.yaw - before.yaw),
    std::cos(measured.yaw - before.yaw));
  const double yaw = before.yaw + std::clamp(alpha * error, -yaw_limit, yaw_limit);
  return {old_point.x + dx - std::cos(yaw) * odom_x + std::sin(yaw) * odom_y,
    old_point.y + dy - std::sin(yaw) * odom_x - std::cos(yaw) * odom_y, yaw};
}

CorrectionDecision evaluateMarkerCorrection(
  double planar_distance, double spatial_distance, double angular_speed,
  const MarkerCorrectionPolicy & policy)
{
  if (!std::isfinite(planar_distance) || !std::isfinite(spatial_distance) ||
    !std::isfinite(angular_speed))
  {
    return CorrectionDecision::INVALID;
  }
  if (std::abs(angular_speed) > policy.max_angular_speed) {
    return CorrectionDecision::ROTATING;
  }
  if (planar_distance < policy.min_planar_distance) {
    return CorrectionDecision::TOO_CLOSE;
  }
  if (spatial_distance > policy.max_spatial_distance) {
    return CorrectionDecision::TOO_FAR;
  }
  return CorrectionDecision::ACCEPT;
}

bool markerAuthorizedForCorrection(int expected_marker_id, int observed_marker_id)
{
  return expected_marker_id < 0 || expected_marker_id == observed_marker_id;
}

const char * correctionDecisionName(CorrectionDecision decision)
{
  switch (decision) {
    case CorrectionDecision::ACCEPT:
      return "ACCEPT";
    case CorrectionDecision::TOO_CLOSE:
      return "TOO_CLOSE";
    case CorrectionDecision::TOO_FAR:
      return "TOO_FAR";
    case CorrectionDecision::ROTATING:
      return "ROTATING";
    case CorrectionDecision::INVALID:
      return "INVALID";
  }
  return "UNKNOWN";
}

}  // namespace aruco_localizer
