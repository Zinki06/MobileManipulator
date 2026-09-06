#include <gtest/gtest.h>
#include "aruco_localizer/depth_projection.hpp"
#include <limits>

TEST(DepthProjection, StridePaddingAndEndianness)
{
  // Two rows of three uint16 samples plus two padding bytes.
  std::vector<uint8_t> data{0xe8,3,0,0,0xb8,11,255,255, 0xe8,3,1,0,0xa1,15,255,255};
  std::vector<float> points;
  aruco_localizer::projectDepth(data.data(), data.size(), 3, 2, 8, false, false,
    1, 2, 2, 1, 0, points);
  ASSERT_EQ(points.size(), 9u);
  EXPECT_FLOAT_EQ(points[0], -0.5F);
  EXPECT_FLOAT_EQ(points[2], 1.F);
  EXPECT_FLOAT_EQ(points[3], 1.5F);
  EXPECT_FLOAT_EQ(points[5], 3.F);
  EXPECT_FLOAT_EQ(points[7], 0.5F);
  const auto expected = points;
  for (size_t i = 0; i < data.size(); i += 2) {std::swap(data[i], data[i + 1]);}
  aruco_localizer::projectDepth(data.data(), data.size(), 3, 2, 8, false, true,
    1, 2, 2, 1, 0, points);
  EXPECT_EQ(points, expected);
  aruco_localizer::projectDepth(data.data(), data.size(), 3, 2, 8, false, true,
    2, 2, 2, 1, 0, points);
  EXPECT_EQ(points.size(), 6u);
}

TEST(DepthProjection, InvalidValuesAndBufferFailClosed)
{
  const std::vector<float> depth{0.F, .1F, 3.F, 3.1F,
    std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()};
  const auto * data = reinterpret_cast<const uint8_t *>(depth.data());
  std::vector<float> points;
  aruco_localizer::projectDepth(data, 24, 6, 1, 24, true, false, 1, 1, 1, 0, 0, points);
  ASSERT_EQ(points.size(), 6u);
  EXPECT_FLOAT_EQ(points[2], .1F);
  EXPECT_FLOAT_EQ(points[5], 3.F);
  EXPECT_THROW(aruco_localizer::projectDepth(data, 23, 6, 1, 24, true, false,
    1, 1, 1, 0, 0, points), std::invalid_argument);
  EXPECT_THROW(aruco_localizer::projectDepth(data, 24, 6, 1, 24, true, false,
    1, 0, 1, 0, 0, points), std::invalid_argument);
  EXPECT_THROW(aruco_localizer::projectDepth(data, 24, 6, 1, 24, true, false,
    0, 1, 1, 0, 0, points), std::invalid_argument);
}
