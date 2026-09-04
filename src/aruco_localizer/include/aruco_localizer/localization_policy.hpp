#ifndef ARUCO_LOCALIZER__LOCALIZATION_POLICY_HPP_
#define ARUCO_LOCALIZER__LOCALIZATION_POLICY_HPP_

namespace aruco_localizer
{

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

const char * correctionDecisionName(CorrectionDecision decision);

}  // namespace aruco_localizer

#endif  // ARUCO_LOCALIZER__LOCALIZATION_POLICY_HPP_
