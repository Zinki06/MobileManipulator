#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include "pick_and_place/candidate_grasp.hpp"

namespace
{
void jointsJson(const std::vector<double> & joints)
{
  std::cout << '[';
  for (size_t i = 0; i < joints.size(); ++i) {
    if (i) {std::cout << ',';}
    std::cout << joints[i];
  }
  std::cout << ']';
}
}

// Read-only JSON-lines adapter; no ROS clients, publishers or hardware access.
int main(int argc, char ** argv)
{
  double min_body_depth = 0.006;
  if (argc != 1) {
    std::istringstream value(argc == 3 ? argv[2] : "");
    std::string extra;
    if (argc != 3 || std::string(argv[1]) != "--min-body-depth" ||
      !(value >> min_body_depth) || (value >> extra) || !std::isfinite(min_body_depth) ||
      min_body_depth < 0.004 || min_body_depth > 0.020)
    {
      std::cerr << "Usage: candidate_grasp_plan [--min-body-depth 0.004..0.020]\n";
      return 2;
    }
  }
  std::cout << std::setprecision(12);
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream input(line);
    double x, y, surface, floor;
    std::vector<double> start(4);
    bool valid = static_cast<bool>(input >> x >> y >> surface >> floor);
    for (auto & q : start) {valid = valid && static_cast<bool>(input >> q);}
    std::string extra;
    valid = valid && !(input >> extra);
    pick_and_place::CandidateGrasp plan;
    pick_and_place::CandidateDiagnostics counts;
    if (!valid || !pick_and_place::planCandidateGrasp(
        x, y, surface, floor, start, plan, &counts, min_body_depth)) {
      std::cout << "{\"feasible\":false,\"rejections\":{\"insufficient_body_depth\":"
        << counts.insufficient_body_depth << ",\"hover_unreachable\":"
        << counts.hover_unreachable << ",\"approach_clearance\":"
        << counts.approach_clearance << ",\"descent_unreachable\":"
        << counts.descent_unreachable << "}}\n";
      continue;
    }
    std::cout << "{\"feasible\":true,\"pitch_degrees\":" << plan.pitch_degrees
      << ",\"body_depth\":" << plan.body_depth
      << ",\"finger_height_above_surface\":0.05,\"path_floor_clearance\":"
      << plan.path_floor_clearance << ",\"joints\":";
    jointsJson(plan.hover);
    std::cout << ",\"descent\":[";
    for (size_t i = 0; i < plan.descent.size(); ++i) {
      if (i) {std::cout << ',';}
      jointsJson(plan.descent[i]);
    }
    std::cout << "]}\n";
  }
}
