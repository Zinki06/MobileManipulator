#include "aruco_localizer/route_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace aruco_localizer
{

namespace
{

double requiredFiniteDouble(
  const YAML::Node & node, const std::string & field,
  const std::string & context)
{
  if (!node[field]) {
    throw std::runtime_error(context + " is missing required field '" + field + "'");
  }

  const double value = node[field].as<double>();
  if (!std::isfinite(value)) {
    throw std::runtime_error(context + " field '" + field + "' must be finite");
  }
  return value;
}

}  // namespace

RouteConfig RouteConfig::loadFromFile(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  if (!root["waypoints"] || !root["waypoints"].IsMap()) {
    throw std::runtime_error("Route config must contain a 'waypoints' map");
  }
  if (!root["routes"] || !root["routes"].IsMap()) {
    throw std::runtime_error("Route config must contain a 'routes' map");
  }

  RouteConfig config;
  for (const auto & entry : root["waypoints"]) {
    const std::string name = entry.first.as<std::string>();
    const YAML::Node value = entry.second;
    if (name.empty()) {
      throw std::runtime_error("Waypoint names must not be empty");
    }
    if (!value.IsMap()) {
      throw std::runtime_error("Waypoint '" + name + "' must be a map");
    }

    RouteWaypoint waypoint;
    waypoint.name = name;
    waypoint.x = requiredFiniteDouble(value, "x", "Waypoint '" + name + "'");
    waypoint.y = requiredFiniteDouble(value, "y", "Waypoint '" + name + "'");
    waypoint.yaw = requiredFiniteDouble(value, "yaw", "Waypoint '" + name + "'");
    waypoint.role = value["role"] ? value["role"].as<std::string>() : "transit";

    if (!config.waypoints_.emplace(name, std::move(waypoint)).second) {
      throw std::runtime_error("Duplicate waypoint name: " + name);
    }
  }

  if (config.waypoints_.empty()) {
    throw std::runtime_error("Route config must define at least one waypoint");
  }

  for (const auto & entry : root["routes"]) {
    const std::string route_name = entry.first.as<std::string>();
    const YAML::Node value = entry.second;
    if (route_name.empty()) {
      throw std::runtime_error("Route names must not be empty");
    }
    if (!value.IsSequence() || value.size() == 0U) {
      throw std::runtime_error("Route '" + route_name + "' must be a non-empty sequence");
    }

    std::vector<std::string> waypoint_names;
    waypoint_names.reserve(value.size());
    for (const auto & waypoint_node : value) {
      const std::string waypoint_name = waypoint_node.as<std::string>();
      if (config.waypoints_.count(waypoint_name) == 0U) {
        throw std::runtime_error(
                "Route '" + route_name + "' references unknown waypoint '" +
                waypoint_name + "'");
      }
      waypoint_names.push_back(waypoint_name);
    }
    config.routes_.emplace(route_name, std::move(waypoint_names));
  }

  if (config.routes_.empty()) {
    throw std::runtime_error("Route config must define at least one route");
  }
  return config;
}

const RouteWaypoint & RouteConfig::waypoint(const std::string & name) const
{
  const auto iterator = waypoints_.find(name);
  if (iterator == waypoints_.end()) {
    throw std::out_of_range("Unknown waypoint: " + name);
  }
  return iterator->second;
}

const std::vector<std::string> & RouteConfig::route(const std::string & name) const
{
  const auto iterator = routes_.find(name);
  if (iterator == routes_.end()) {
    throw std::out_of_range("Unknown route: " + name);
  }
  return iterator->second;
}

bool RouteConfig::hasRoute(const std::string & name) const
{
  return routes_.count(name) != 0U;
}

const std::unordered_map<std::string, RouteWaypoint> & RouteConfig::waypoints() const
{
  return waypoints_;
}

}  // namespace aruco_localizer
