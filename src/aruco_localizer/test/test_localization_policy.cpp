#include "aruco_localizer/localization_policy.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace aruco_localizer
{
namespace
{

const MarkerCorrectionPolicy kPolicy{0.40, 2.50, 0.15};

TEST(LocalizationPolicy, AcceptsStableObservationInRange)
{
  EXPECT_EQ(
    evaluateMarkerCorrection(0.80, 0.95, 0.05, kPolicy),
    CorrectionDecision::ACCEPT);
}

TEST(LocalizationPolicy, UsesPlanarDistanceForNearFieldRejection)
{
  EXPECT_EQ(
    evaluateMarkerCorrection(0.25, 0.45, 0.0, kPolicy),
    CorrectionDecision::TOO_CLOSE);
}

TEST(LocalizationPolicy, RejectsCorrectionDuringRotation)
{
  EXPECT_EQ(
    evaluateMarkerCorrection(0.80, 0.95, -0.20, kPolicy),
    CorrectionDecision::ROTATING);
}

TEST(LocalizationPolicy, RejectsFarAndInvalidObservations)
{
  EXPECT_EQ(
    evaluateMarkerCorrection(2.0, 2.60, 0.0, kPolicy),
    CorrectionDecision::TOO_FAR);
  EXPECT_EQ(
    evaluateMarkerCorrection(
      std::numeric_limits<double>::quiet_NaN(), 1.0, 0.0, kPolicy),
    CorrectionDecision::INVALID);
}

}  // namespace
}  // namespace aruco_localizer
