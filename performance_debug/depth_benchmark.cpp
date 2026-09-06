#include "aruco_localizer/depth_projection.hpp"
#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

int main(int argc, char ** argv)
{
  if (argc != 9) {return 2;}
  std::ifstream input(argv[1], std::ios::binary);
  std::vector<uint8_t> data{std::istreambuf_iterator<char>(input), {}};
  auto width = std::stoul(argv[2]), height = std::stoul(argv[3]);
  const double fx = std::stod(argv[4]), fy = std::stod(argv[5]);
  const double cx = std::stod(argv[6]), cy = std::stod(argv[7]);
  std::vector<float> points;
  std::vector<double> times, cpus;
  for (int i = 0; i < 300; ++i) {
    const auto start = std::chrono::steady_clock::now();
    const auto cpu = std::clock();
    aruco_localizer::projectDepth(data.data(), data.size(), width, height, width * 2,
      false, false, 8, fx, fy, cx, cy, points);
    const auto duration = std::chrono::steady_clock::now() - start;
    times.push_back(std::chrono::duration<double, std::milli>(duration).count());
    cpus.push_back(1000. * (std::clock() - cpu) / CLOCKS_PER_SEC);
  }
  std::sort(times.begin(), times.end());
  std::sort(cpus.begin(), cpus.end());
  std::ofstream output(argv[8], std::ios::binary);
  output.write(reinterpret_cast<const char *>(points.data()), points.size() * sizeof(float));
  std::cout << "{\"n\":300,\"wall_median_ms\":" << times[150]
    << ",\"wall_p95_ms\":" << times[285] << ",\"cpu_median_ms\":" << cpus[150]
    << ",\"points\":" << points.size() / 3 << "}\n";
}
