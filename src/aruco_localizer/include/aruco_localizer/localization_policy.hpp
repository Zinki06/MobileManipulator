#ifndef ARUCO_LOCALIZER__LOCALIZATION_POLICY_HPP_
#define ARUCO_LOCALIZER__LOCALIZATION_POLICY_HPP_

namespace aruco_localizer
{

struct Pose2D
{
  double x;
  double y;
  double yaw;
};

// Compare corrections at the same robot point, never at the arbitrary odom origin.
double correctionDisplacement(const Pose2D & before, const Pose2D & after,
  double odom_x, double odom_y);
Pose2D boundedCorrection(const Pose2D & before, const Pose2D & measured,
  double odom_x, double odom_y, double alpha, double translation_limit, double yaw_limit);

struct MarkerCorrectionPolicy
{
  double min_planar_distance;
  double max_spatial_distance;
  double max_angular_speed;
};

enum class CorrectionDecision
{
  ACCEPT,
  TOO_CLOSE,
  TOO_FAR,
  ROTATING,
  INVALID
};

CorrectionDecision evaluateMarkerCorrection(
  double planar_distance, double spatial_distance, double angular_speed,
  const MarkerCorrectionPolicy & policy);

bool markerAuthorizedForCorrection(int expected_marker_id, int observed_marker_id);

const char * correctionDecisionName(CorrectionDecision decision);

}  // namespace aruco_localizer

#endif  // ARUCO_LOCALIZER__LOCALIZATION_POLICY_HPP_
