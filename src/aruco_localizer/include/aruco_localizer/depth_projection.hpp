#ifndef ARUCO_LOCALIZER__DEPTH_PROJECTION_HPP_
#define ARUCO_LOCALIZER__DEPTH_PROJECTION_HPP_

#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace aruco_localizer
{
// Bytes are decoded explicitly: both padded rows and big-endian sensors are supported.
inline void projectDepth(const uint8_t * data, size_t bytes, uint32_t width,
  uint32_t height, uint32_t step, bool floating, bool big_endian, unsigned stride,
  double fx, double fy, double cx, double cy, std::vector<float> & points)
{
  const size_t pixel_bytes = floating ? 4 : 2;
  if (!data || !width || !height || !stride || step < size_t(width) * pixel_bytes ||
    bytes < size_t(height) * step || !std::isfinite(fx) || !std::isfinite(fy) ||
    !std::isfinite(cx) || !std::isfinite(cy) || fx <= 0 || fy <= 0)
  {
    throw std::invalid_argument("Invalid aligned depth layout/intrinsics");
  }
  points.clear();
  points.reserve(((size_t(width) + stride - 1) / stride) *
    ((size_t(height) + stride - 1) / stride) * 3);
  for (uint32_t row = 0; row < height; row += stride) {
    for (uint32_t col = 0; col < width; col += stride) {
      const auto * pixel = data + size_t(row) * step + size_t(col) * pixel_bytes;
      uint32_t raw = 0;
      for (size_t b = 0; b < pixel_bytes; ++b) {
        raw |= uint32_t(pixel[b]) << (8 * (big_endian ? pixel_bytes - b - 1 : b));
      }
      float z;
      if (floating) {std::memcpy(&z, &raw, sizeof(z));} else {z = raw / 1000.0F;}
      if (!std::isfinite(z) || z < 0.10F || z > 3.0F) {continue;}
      points.push_back(static_cast<float>((col - cx) * z / fx));
      points.push_back(static_cast<float>((row - cy) * z / fy));
      points.push_back(z);
    }
  }
}
}  // namespace aruco_localizer
#endif
