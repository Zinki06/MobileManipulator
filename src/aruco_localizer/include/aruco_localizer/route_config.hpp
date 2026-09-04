#ifndef ARUCO_LOCALIZER__ROUTE_CONFIG_HPP_
#define ARUCO_LOCALIZER__ROUTE_CONFIG_HPP_

#include <string>
#include <unordered_map>
#include <vector>

namespace aruco_localizer
{

struct RouteWaypoint
{
  std::string name;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string role{"transit"};
};

class RouteConfig
{
public:
  static RouteConfig loadFromFile(const std::string & path);

  const RouteWaypoint & waypoint(const std::string & name) const;
  const std::vector<std::string> & route(const std::string & name) const;
  bool hasRoute(const std::string & name) const;

  const std::unordered_map<std::string, RouteWaypoint> & waypoints() const;

private:
  std::unordered_map<std::string, RouteWaypoint> waypoints_;
  std::unordered_map<std::string, std::vector<std::string>> routes_;
};

}  // namespace aruco_localizer

#endif  // ARUCO_LOCALIZER__ROUTE_CONFIG_HPP_
