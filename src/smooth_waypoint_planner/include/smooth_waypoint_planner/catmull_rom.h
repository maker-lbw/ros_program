#ifndef SMOOTH_WAYPOINT_PLANNER__CATMULL_ROM_H_
#define SMOOTH_WAYPOINT_PLANNER__CATMULL_ROM_H_

#include <vector>

namespace smooth_waypoint_planner
{

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

class CatmullRomSpline
{
public:
  void setControlPoints(const std::vector<Point2D> &points);
  std::vector<Point2D> sample(double resolution) const;

private:
  std::vector<Point2D> control_points_;
};

}  // namespace smooth_waypoint_planner

#endif  // SMOOTH_WAYPOINT_PLANNER__CATMULL_ROM_H_
