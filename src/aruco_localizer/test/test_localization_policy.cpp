#include "aruco_localizer/localization_policy.hpp"

#include <gtest/gtest.h>
#include <cmath>

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

TEST(LocalizationPolicy, AuthorizesOnlyExpectedMarkerWhenRouteProvidesOne)
{
  EXPECT_TRUE(markerAuthorizedForCorrection(-1, 4));
  EXPECT_TRUE(markerAuthorizedForCorrection(3, 3));
  EXPECT_FALSE(markerAuthorizedForCorrection(3, 2));
  EXPECT_FALSE(markerAuthorizedForCorrection(3, 4));
}

}  // namespace
}  // namespace aruco_localizer
TEST(CorrectionGeometry, LoggedStationFourIsSixCmNotFortyFiveCm)
{
  using namespace aruco_localizer;
  const Pose2D before{1.972931, 0.769003, -2.669224};
  const Pose2D measured{2.213758, 1.155629, -2.509005};
  EXPECT_GT(std::hypot(measured.x - before.x, measured.y - before.y), 0.45);
  EXPECT_NEAR(correctionDisplacement(before, measured, 1.044807, -2.234214), 0.061584, 0.001);
  const auto filtered = boundedCorrection(before, measured, 1.044807, -2.234214,
    0.15, 0.03, 0.03);
  EXPECT_LT(correctionDisplacement(before, filtered, 1.044807, -2.234214), 0.01);
}

TEST(CorrectionGeometry, OriginIndependentAndRealJumpsRemainVisible)
{
  using namespace aruco_localizer;
  const Pose2D before{0, 0, 0};
  // Rotate 0.16rad about a robot standing ten meters from the odom origin.
  const Pose2D rotated{10 - 10 * std::cos(0.16), -10 * std::sin(0.16), 0.16};
  EXPECT_NEAR(correctionDisplacement(before, rotated, 10, 0), 0.0, 1e-12);
  const auto filtered = boundedCorrection(before, rotated, 10, 0, 0.15, 0.03, 0.03);
  EXPECT_NEAR(correctionDisplacement(before, filtered, 10, 0), 0.0, 1e-12);
  EXPECT_GT(correctionDisplacement(before, {0.5, 0, 0}, 10, 0), 0.4);
  const auto bounded = boundedCorrection(before, {0.5, 0, 0}, 10, 0, 0.15, 0.03, 0.03);
  EXPECT_NEAR(correctionDisplacement(before, bounded, 10, 0), 0.03, 1e-12);
}
