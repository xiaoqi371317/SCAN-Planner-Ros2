#ifndef PLAN_MANAGE_REFERENCE_PATH_UTILS_H
#define PLAN_MANAGE_REFERENCE_PATH_UTILS_H

#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <nav_msgs/msg/path.hpp>

namespace scan_planner
{

inline bool prepareReferenceWaypoints(
    const nav_msgs::msg::Path &path,
    double body_height,
    double min_distance,
    std::vector<Eigen::Vector3d> &waypoints,
    std::string *error = nullptr)
{
  waypoints.clear();
  if (path.poses.empty())
  {
    if (error) *error = "reference path is empty";
    return false;
  }
  if (!std::isfinite(body_height) || !std::isfinite(min_distance) || min_distance < 0.0)
  {
    if (error) *error = "body height and minimum distance must be finite and non-negative";
    return false;
  }

  waypoints.reserve(path.poses.size());
  Eigen::Vector3d final_waypoint;
  Eigen::Vector3d last_waypoint;
  bool first = true;

  for (const auto &pose_stamped : path.poses)
  {
    const auto &position = pose_stamped.pose.position;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z))
    {
      waypoints.clear();
      if (error) *error = "reference path contains a non-finite coordinate";
      return false;
    }

    Eigen::Vector3d waypoint(position.x, position.y, position.z + body_height);
    final_waypoint = waypoint;
    if (first || (waypoint - last_waypoint).norm() >= min_distance)
    {
      waypoints.push_back(waypoint);
      last_waypoint = waypoint;
      first = false;
    }
  }

  if ((waypoints.back() - final_waypoint).norm() > 1e-6)
    waypoints.push_back(final_waypoint);

  if (waypoints.size() < 2)
  {
    waypoints.clear();
    if (error) *error = "reference path requires at least two distinct points";
    return false;
  }
  return true;
}

}  // namespace scan_planner

#endif  // PLAN_MANAGE_REFERENCE_PATH_UTILS_H
