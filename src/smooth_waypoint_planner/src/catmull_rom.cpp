#include "smooth_waypoint_planner/catmull_rom.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace smooth_waypoint_planner
{
namespace
{

double Distance(const Point2D &a, const Point2D &b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::sqrt((dx * dx) + (dy * dy));
}

Point2D Subtract(const Point2D &a, const Point2D &b)
{
  Point2D result;
  result.x = a.x - b.x;
  result.y = a.y - b.y;
  return result;
}

Point2D Scale(const Point2D &point, double factor)
{
  Point2D result;
  result.x = point.x * factor;
  result.y = point.y * factor;
  return result;
}

Point2D Interpolate(const Point2D &a, const Point2D &b, double ta, double tb, double t)
{
  const double denominator = tb - ta;
  if (std::fabs(denominator) < 1e-9) {
    return b;
  }

  const double wa = (tb - t) / denominator;
  const double wb = (t - ta) / denominator;
  Point2D result;
  result.x = (wa * a.x) + (wb * b.x);
  result.y = (wa * a.y) + (wb * b.y);
  return result;
}

Point2D SampleSegment(
  const Point2D &p0,
  const Point2D &p1,
  const Point2D &p2,
  const Point2D &p3,
  double u)
{
  const double alpha = 0.5;
  const double epsilon = 1e-6;

  const double t0 = 0.0;
  const double t1 = t0 + std::max(std::pow(Distance(p0, p1), alpha), epsilon);
  const double t2 = t1 + std::max(std::pow(Distance(p1, p2), alpha), epsilon);
  const double t3 = t2 + std::max(std::pow(Distance(p2, p3), alpha), epsilon);
  const double t = t1 + (u * (t2 - t1));

  const Point2D a1 = Interpolate(p0, p1, t0, t1, t);
  const Point2D a2 = Interpolate(p1, p2, t1, t2, t);
  const Point2D a3 = Interpolate(p2, p3, t2, t3, t);
  const Point2D b1 = Interpolate(a1, a2, t0, t2, t);
  const Point2D b2 = Interpolate(a2, a3, t1, t3, t);
  return Interpolate(b1, b2, t1, t2, t);
}

}  // namespace

void CatmullRomSpline::setControlPoints(const std::vector<Point2D> &points)
{
  control_points_ = points;
}

std::vector<Point2D> CatmullRomSpline::sample(double resolution) const
{
  std::vector<Point2D> sampled_points;
  if (control_points_.empty()) {
    return sampled_points;
  }

  if (control_points_.size() == 1U) {
    sampled_points.push_back(control_points_.front());
    return sampled_points;
  }

  if (resolution <= 0.0) {
    resolution = 0.05;
  }

  if (control_points_.size() == 2U) {
    const Point2D &start = control_points_.front();
    const Point2D &end = control_points_.back();
    const double segment_length = Distance(start, end);
    const std::size_t steps = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(segment_length / resolution)));

    sampled_points.reserve(steps + 1U);
    for (std::size_t step = 0; step < steps; ++step) {
      const double ratio = static_cast<double>(step) / static_cast<double>(steps);
      Point2D point;
      point.x = start.x + ((end.x - start.x) * ratio);
      point.y = start.y + ((end.y - start.y) * ratio);
      sampled_points.push_back(point);
    }
    sampled_points.push_back(end);
    return sampled_points;
  }

  std::vector<Point2D> extended_points;
  extended_points.reserve(control_points_.size() + 2U);
  extended_points.push_back(Subtract(Scale(control_points_[0], 2.0), control_points_[1]));
  extended_points.insert(
    extended_points.end(), control_points_.begin(), control_points_.end());
  extended_points.push_back(
    Subtract(
      Scale(control_points_.back(), 2.0),
      control_points_[control_points_.size() - 2U]));

  for (std::size_t segment = 0; segment + 1U < control_points_.size(); ++segment) {
    const Point2D &p0 = extended_points[segment];
    const Point2D &p1 = extended_points[segment + 1U];
    const Point2D &p2 = extended_points[segment + 2U];
    const Point2D &p3 = extended_points[segment + 3U];
    const double segment_length = Distance(p1, p2);
    const std::size_t steps = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(segment_length / resolution)));

    for (std::size_t step = 0; step < steps; ++step) {
      const double u = static_cast<double>(step) / static_cast<double>(steps);
      const Point2D point = SampleSegment(p0, p1, p2, p3, u);
      if (sampled_points.empty() || Distance(sampled_points.back(), point) > 1e-9) {
        sampled_points.push_back(point);
      }
    }
  }

  if (sampled_points.empty() || Distance(sampled_points.back(), control_points_.back()) > 1e-9) {
    sampled_points.push_back(control_points_.back());
  }

  return sampled_points;
}

}  // namespace smooth_waypoint_planner
