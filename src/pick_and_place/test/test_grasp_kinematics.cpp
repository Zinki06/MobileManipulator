#include <gtest/gtest.h>
#include "pick_and_place/grasp_kinematics.hpp"
#include "pick_and_place/candidate_grasp.hpp"
#include "pick_and_place/place_kinematics.hpp"
#include <limits>

TEST(PlaceKinematics, LowReleaseAndRetreatAtCollectionStandoff)
{
  pick_and_place::PlacePlan plan;
  ASSERT_TRUE(pick_and_place::planPlace(0.332, 0.0, -0.101, 0.035,
    {0.0, -0.523, -0.523, 1.5707}, plan));
  EXPECT_EQ(plan.lower.size(), 10U);
  EXPECT_EQ(plan.retreat.size(), 10U);
  EXPECT_EQ(plan.retreat.back(), plan.above);
  for (const auto & q : plan.lower) {
    EXPECT_GE(pick_and_place::fingerFloorClearance(q, -0.101), 0.00799);
  }
  EXPECT_FALSE(pick_and_place::planPlace(0.332, 0, -0.101, 0.005,
    {0.0, -0.523, -0.523, 1.5707}, plan));
  EXPECT_FALSE(pick_and_place::planPlace(0.6, 0, -0.101, 0.035,
    {0.0, -0.523, -0.523, 1.5707}, plan));
}

TEST(CandidateGrasp, DiagonalBodyHasCompleteFloorClearDescent)
{
  const std::vector<double> start{0, -0.514, -0.512, 1.772};
  pick_and_place::CandidateGrasp plan;
  ASSERT_TRUE(pick_and_place::planCandidateGrasp(
    0.333629, -0.053935, -0.068509, -0.101, start, plan));
  EXPECT_GE(plan.body_depth, 0.006);
  EXPECT_EQ(plan.descent.size(), 10U);
  EXPECT_NEAR(pick_and_place::fingerFloorClearance(plan.hover, -0.101),
    -0.068509 + 0.101 + 0.05, 1e-8);
  EXPECT_TRUE(pick_and_place::fingerPathClear(plan.hover, plan.descent, -0.101));
  EXPECT_GE(pick_and_place::fingerFloorClearance(plan.descent.back(), -0.101), 0.00799);
}

TEST(CandidateGrasp, RejectsUnreachableTargetAndInvalidStart)
{
  pick_and_place::CandidateGrasp plan;
  const std::vector<double> start{0, -0.514, -0.512, 1.772};
  EXPECT_FALSE(pick_and_place::planCandidateGrasp(0.6, 0, -0.07, -0.101, start, plan));
  EXPECT_FALSE(pick_and_place::planCandidateGrasp(0.3, 0, -0.098, -0.101, start, plan));
  EXPECT_FALSE(pick_and_place::planCandidateGrasp(0.3, 0, -0.07, -0.101, {}, plan));
  auto invalid = start;
  invalid[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(pick_and_place::planCandidateGrasp(0.3, 0, -0.07, -0.101, invalid, plan));
}

TEST(CandidateGrasp, RejectedLiveCaptureRetainsDiagnosticReasons)
{
  pick_and_place::CandidateGrasp plan;
  pick_and_place::CandidateDiagnostics counts;
  const std::vector<double> start{0, -0.514, -0.512, 1.772};
  EXPECT_FALSE(pick_and_place::planCandidateGrasp(
    0.337221, -0.047522, -0.069773, -0.101, start, plan, &counts));
  EXPECT_GT(counts.insufficient_body_depth, 0);
  EXPECT_GT(counts.descent_unreachable + counts.hover_unreachable, 0);
}

TEST(CandidateGrasp, RelaxedQualityAcceptsRecordedBodyWithoutReducingFloorClearance)
{
  const std::vector<double> start{0, -0.514, -0.512, 1.772};
  pick_and_place::GraspPlan plan;
  EXPECT_FALSE(pick_and_place::planCandidateExecution(
    0.337221, -0.047522, -0.069773, -0.101, start, plan, nullptr, 0.006));
  ASSERT_TRUE(pick_and_place::planCandidateExecution(
    0.337221, -0.047522, -0.069773, -0.101, start, plan, nullptr, 0.004));
  EXPECT_GE(plan.surface_to_body_depth, 0.004);
  EXPECT_LT(plan.surface_to_body_depth, 0.006);
  EXPECT_GE(pick_and_place::fingerFloorClearance(plan.descent.back(), -0.101), 0.00799);
  EXPECT_TRUE(pick_and_place::fingerPathClear(plan.descent.back(), plan.lift, -0.101));
  // The quality setting must not allow floor contact, zero support or invalid feedback.
  EXPECT_FALSE(pick_and_place::planCandidateExecution(
    0.322, 0, -0.098, -0.101, start, plan, nullptr, 0.004));
  EXPECT_FALSE(pick_and_place::planCandidateExecution(
    0.322, 0, -0.072, -0.101, {}, plan, nullptr, 0.004));
  EXPECT_FALSE(pick_and_place::planCandidateExecution(
    0.322, 0, -0.072, -0.101, start, plan, nullptr, 0.0));
}

TEST(CandidateGrasp, StationFloorBiasBlocksEveryApproachUntilHeightIsReobserved)
{
  const std::vector<double> start{0.001534, -0.513884, -0.497010, 1.575398};
  for (const auto & heights : {std::pair<double, double>{0.0213585, 0.0324071},
      {0.0231518, 0.0323211}})
  {
    for (double standoff = 0.30; standoff >= 0.179; standoff -= 0.02) {
      pick_and_place::GraspPlan plan;
      pick_and_place::CandidateDiagnostics counts;
      EXPECT_FALSE(pick_and_place::planCandidateExecution(
        standoff + 0.122, 0, heights.first - 0.101, -0.101, start, plan, &counts));
      EXPECT_EQ(counts.insufficient_body_depth, 27);
      EXPECT_EQ(counts.hover_unreachable, 0);
      EXPECT_NE(pick_and_place::candidateRejectionSummary(counts).find(
        "insufficient_body_depth=27"), std::string::npos);
    }
    // 20cm base standoff, including the existing 2cm arrival tolerance.
    for (double x : {0.322, 0.342}) {
      pick_and_place::GraspPlan plan;
      ASSERT_TRUE(pick_and_place::planCandidateExecution(
        x, 0, heights.second - 0.101, -0.101, start, plan));
      EXPECT_GE(plan.surface_to_body_depth, 0.006);
      EXPECT_GE(pick_and_place::fingerFloorClearance(plan.descent.back(), -0.101), 0.00799);
      EXPECT_TRUE(pick_and_place::fingerPathClear(plan.descent.back(), plan.lift, -0.101));
    }
  }
}

TEST(GraspKinematics, LoggedTargetRequiresBaseApproach)
{
  EXPECT_FALSE(pick_and_place::graspAndLiftReachable(0.434, -0.034, -0.053));
  // TF mounts link1 9.2cm behind base_link; verify a floor grasp after moving closer.
  bool found = false;
  for (double standoff = 0.30; standoff >= 0.179; standoff -= 0.02) {
    if (pick_and_place::graspAndLiftReachable(standoff + 0.092, 0.0, -0.053) &&
      pick_and_place::graspAndLiftReachable(standoff + 0.112, 0.0, -0.053))
    {
      found = true;
    }
  }
  EXPECT_TRUE(found);
}

TEST(GraspKinematics, RejectsNonFiniteAndImpossibleHeights)
{
  EXPECT_FALSE(pick_and_place::graspAndLiftReachable(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0));
  EXPECT_FALSE(pick_and_place::graspAndLiftReachable(0.20, 0.0, 1.0));
}

TEST(GraspKinematics, RejectsLoggedShallowGraspsAndAcceptsCloserCompletePlan)
{
  pick_and_place::GraspPlan plan;
  EXPECT_FALSE(pick_and_place::planGrasp(0.346613, -0.009366, -0.046276, -0.101, 0.012, plan));
  EXPECT_FALSE(pick_and_place::planGrasp(0.350701, 0.002826, -0.052212, -0.101, 0.012, plan));
  ASSERT_TRUE(pick_and_place::planGrasp(0.292, 0.0, -0.05, -0.101, 0.012, plan));
  EXPECT_LE(plan.pitch_degrees, -55.0);
  EXPECT_GE(plan.pitch_degrees, -65.0);
  EXPECT_LE(plan.insertion_depth, 0.012);
  EXPECT_EQ(plan.descent.size(), 10U);
  EXPECT_EQ(plan.insertion.size(), 4U);
  EXPECT_EQ(plan.lift.size(), 10U);
  EXPECT_NEAR(plan.surface_to_body_depth, 0.020, 1e-9);
  for (const auto & path : {plan.descent, plan.insertion, plan.lift}) {
    for (const auto & q : path) {
      EXPECT_NEAR(-(q[1] + q[2] + q[3]) * 180.0 / M_PI, plan.pitch_degrees, 1e-9);
    }
  }
}

TEST(GraspKinematics, InsertionNeverCrossesFloorOrExceedsBound)
{
  pick_and_place::GraspPlan plan;
  EXPECT_FALSE(pick_and_place::planGrasp(0.292, 0, -0.095, -0.101, 0.012, plan));
  EXPECT_FALSE(pick_and_place::planGrasp(0.292, 0, -0.05, -0.101, 0.1, plan));
}

TEST(GraspKinematics, ActualFingerMeshStaysAboveFloorAtLowSurface)
{
  pick_and_place::GraspPlan plan;
  ASSERT_TRUE(pick_and_place::planGrasp(0.292, 0, -0.071, -0.101, 0.012, plan));
  EXPECT_LT(plan.insertion_depth, 0.012);
  for (const auto & path : {plan.descent, plan.insertion, plan.lift}) {
    for (const auto & q : path) {
      EXPECT_GE(pick_and_place::fingerFloorClearance(q, -0.101), 0.00799);
    }
  }
  std::vector<double> unsafe;
  ASSERT_TRUE(pick_and_place::solve4DofIK(0.292, 0, -0.089, unsafe, -60, false));
  EXPECT_LT(pick_and_place::fingerFloorClearance(unsafe, -0.101), 0.0);
  EXPECT_FALSE(pick_and_place::fingerPathClear(plan.pregrasp, {unsafe}, -0.101));
}
