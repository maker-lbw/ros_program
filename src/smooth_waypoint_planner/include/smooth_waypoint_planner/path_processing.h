#ifndef SMOOTH_WAYPOINT_PLANNER__PATH_PROCESSING_H_
#define SMOOTH_WAYPOINT_PLANNER__PATH_PROCESSING_H_

#include <cstddef>
#include <vector>

#include <nav_msgs/Path.h>

#include "smooth_waypoint_planner/catmull_rom.h"

namespace smooth_waypoint_planner
{

enum class WaypointSource
{
  kNone,
  kLiveTopic,
  kFile,
};

struct WaypointSourcePolicy
{
  bool prefer_live_waypoints = true;
  bool live_waypoints_received = false;
  bool live_waypoints_available = false;
  bool waypoint_file_configured = false;
};

struct PathProcessingOptions
{
  double min_waypoint_spacing = 0.05;
  double corner_rounding_min_turn_deg = 30.0;
  double corner_blend_distance = 0.3;
};

WaypointSource SelectWaypointSource(const WaypointSourcePolicy &policy);
std::vector<Point2D> ConvertPathToPoints(const nav_msgs::Path &path);
std::vector<Point2D> BuildWaypointTraversal(
  const std::vector<Point2D> &points,
  std::size_t start_index,
  bool loop_waypoints);
double ComputeTurnAngleDegrees(const Point2D &previous, const Point2D &current, const Point2D &next);
std::vector<Point2D> PrepareWaypointsForSpline(
  const std::vector<Point2D> &points,
  const PathProcessingOptions &options);

}  // namespace smooth_waypoint_planner

#endif  // SMOOTH_WAYPOINT_PLANNER__PATH_PROCESSING_H_
