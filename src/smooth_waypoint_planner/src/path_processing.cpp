#include "smooth_waypoint_planner/path_processing.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace smooth_waypoint_planner
{
namespace
{
constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;

double Distance(const Point2D &a, const Point2D &b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::sqrt((dx * dx) + (dy * dy));
}

bool IsFinitePoint(const Point2D &point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

void AppendIfFarEnough(std::vector<Point2D> &points, const Point2D &candidate, double minimum_distance)
{
  if (points.empty() || Distance(points.back(), candidate) >= minimum_distance) {
    points.push_back(candidate);
  }
}

Point2D NormalizeVector(const Point2D &vector)
{
  const double length = std::sqrt((vector.x * vector.x) + (vector.y * vector.y));
  if (length <= std::numeric_limits<double>::epsilon()) {
    return Point2D{};
  }

  Point2D normalized;
  normalized.x = vector.x / length;
  normalized.y = vector.y / length;
  return normalized;
}

std::vector<Point2D> FilterPointsBySpacing(const std::vector<Point2D> &points, double minimum_distance)
{
  std::vector<Point2D> filtered;
  filtered.reserve(points.size());

  for (const Point2D &point : points) {
    if (!IsFinitePoint(point)) {
      continue;
    }

    AppendIfFarEnough(filtered, point, minimum_distance);
  }

  return filtered;
}

}  // namespace

WaypointSource SelectWaypointSource(const WaypointSourcePolicy &policy)
{
  if (policy.prefer_live_waypoints) {
    if (policy.live_waypoints_available) {
      return WaypointSource::kLiveTopic;
    }
    if (policy.waypoint_file_configured) {
      return WaypointSource::kFile;
    }
    return WaypointSource::kNone;
  }

  if (policy.waypoint_file_configured) {
    return WaypointSource::kFile;
  }
  if (policy.live_waypoints_available) {
    return WaypointSource::kLiveTopic;
  }
  return WaypointSource::kNone;
}

std::vector<Point2D> ConvertPathToPoints(const nav_msgs::Path &path)
{
  std::vector<Point2D> points;
  points.reserve(path.poses.size());

  for (const auto &pose : path.poses) {
    Point2D point;
    point.x = pose.pose.position.x;
    point.y = pose.pose.position.y;
    points.push_back(point);
  }

  return points;
}

std::vector<Point2D> BuildWaypointTraversal(
  const std::vector<Point2D> &points,
  std::size_t start_index,
  bool loop_waypoints)
{
  std::vector<Point2D> traversal;
  if (points.empty()) {
    return traversal;
  }

  const std::size_t clamped_start = std::min(start_index, points.size());
  traversal.reserve(points.size());

  if (clamped_start < points.size()) {
    traversal.insert(traversal.end(), points.begin() + static_cast<std::ptrdiff_t>(clamped_start), points.end());
  }

  if (!loop_waypoints) {
    return traversal;
  }

  if (clamped_start >= points.size()) {
    traversal = points;
    return traversal;
  }

  traversal.insert(
    traversal.end(),
    points.begin(),
    points.begin() + static_cast<std::ptrdiff_t>(clamped_start));
  return traversal;
}

double ComputeTurnAngleDegrees(const Point2D &previous, const Point2D &current, const Point2D &next)
{
  const Point2D incoming{current.x - previous.x, current.y - previous.y};
  const Point2D outgoing{next.x - current.x, next.y - current.y};

  const double incoming_length = std::sqrt((incoming.x * incoming.x) + (incoming.y * incoming.y));
  const double outgoing_length = std::sqrt((outgoing.x * outgoing.x) + (outgoing.y * outgoing.y));
  if (incoming_length <= std::numeric_limits<double>::epsilon() ||
      outgoing_length <= std::numeric_limits<double>::epsilon())
  {
    return 0.0;
  }

  const double dot = (incoming.x * outgoing.x) + (incoming.y * outgoing.y);
  const double cosine = std::max(-1.0, std::min(1.0, dot / (incoming_length * outgoing_length)));
  return std::acos(cosine) * kRadiansToDegrees;
}

std::vector<Point2D> PrepareWaypointsForSpline(
  const std::vector<Point2D> &points,
  const PathProcessingOptions &options)
{
  const double minimum_distance = std::max(0.0, options.min_waypoint_spacing);
  const std::vector<Point2D> filtered = FilterPointsBySpacing(points, minimum_distance);

  if (filtered.size() < 3U ||
      options.corner_blend_distance <= 0.0 ||
      options.corner_rounding_min_turn_deg <= 0.0)
  {
    return filtered;
  }

  std::vector<Point2D> rounded;
  rounded.reserve(filtered.size() * 2U);
  rounded.push_back(filtered.front());

  for (std::size_t index = 1; index + 1U < filtered.size(); ++index) {
    const Point2D &previous = filtered[index - 1U];
    const Point2D &current = filtered[index];
    const Point2D &next = filtered[index + 1U];

    const double turn_angle_deg = ComputeTurnAngleDegrees(previous, current, next);
    const double incoming_length = Distance(previous, current);
    const double outgoing_length = Distance(current, next);
    const double max_blend_distance = std::min(
      options.corner_blend_distance,
      std::min(incoming_length, outgoing_length) * 0.5);

    if (turn_angle_deg < options.corner_rounding_min_turn_deg ||
        turn_angle_deg > 175.0 ||
        max_blend_distance <= minimum_distance)
    {
      AppendIfFarEnough(rounded, current, minimum_distance);
      continue;
    }

    const Point2D incoming_direction = NormalizeVector(
      Point2D{current.x - previous.x, current.y - previous.y});
    const Point2D outgoing_direction = NormalizeVector(
      Point2D{next.x - current.x, next.y - current.y});

    const Point2D entry{
      current.x - (incoming_direction.x * max_blend_distance),
      current.y - (incoming_direction.y * max_blend_distance)};
    const Point2D exit{
      current.x + (outgoing_direction.x * max_blend_distance),
      current.y + (outgoing_direction.y * max_blend_distance)};

    AppendIfFarEnough(rounded, entry, minimum_distance);
    AppendIfFarEnough(rounded, exit, minimum_distance);
  }

  AppendIfFarEnough(rounded, filtered.back(), minimum_distance);
  return FilterPointsBySpacing(rounded, minimum_distance);
}

}  // namespace smooth_waypoint_planner
