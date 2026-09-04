#include "aruco_localizer/localization_policy.hpp"

#include <cmath>

namespace aruco_localizer
{

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
