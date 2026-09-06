#include "aruco_localizer/marker_guidance.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace aruco_localizer
{
namespace
{

TEST(MarkerGuidance, PreviousMarkerPredictsOppositeDirectionOnStraightRoute)
{
  EXPECT_NEAR(predictTargetBearing(0.0, 0.0, M_PI), M_PI, 1e-9);
}

TEST(MarkerGuidance, RouteGeometryHandlesCornerInsteadOfBlindHalfTurn)
{
  EXPECT_NEAR(predictTargetBearing(0.0, 0.0, M_PI_2), M_PI_2, 1e-9);
}

TEST(MarkerGuidance, PredictionWrapsAcrossPiBoundary)
{
  EXPECT_NEAR(
    predictTargetBearing(-0.1, -3.0, 3.0), -0.383185307179586, 1e-9);
}

TEST(MarkerGuidance, SymmetricSearchReturnsToOriginalHeading)
{
  const double step = -M_PI / 12.0;
  EXPECT_NEAR(symmetricSearchTurn(step, 0), -M_PI / 12.0, 1e-9);
  EXPECT_NEAR(symmetricSearchTurn(step, 1), M_PI / 6.0, 1e-9);
  EXPECT_NEAR(symmetricSearchTurn(step, 2), -M_PI / 12.0, 1e-9);
}

TEST(MarkerGuidance, SymmetricSearchRepeatsItsThreeTurnCycle)
{
  EXPECT_DOUBLE_EQ(symmetricSearchTurn(0.2, 3), 0.2);
  EXPECT_DOUBLE_EQ(symmetricSearchTurn(0.2, 4), -0.4);
  EXPECT_DOUBLE_EQ(symmetricSearchTurn(0.2, 5), 0.2);
}

}  // namespace
}  // namespace aruco_localizer
