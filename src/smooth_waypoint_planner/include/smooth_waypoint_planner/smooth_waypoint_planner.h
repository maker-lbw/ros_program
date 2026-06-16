#ifndef SMOOTH_WAYPOINT_PLANNER__SMOOTH_WAYPOINT_PLANNER_H_
#define SMOOTH_WAYPOINT_PLANNER__SMOOTH_WAYPOINT_PLANNER_H_

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>

#include "smooth_waypoint_planner/catmull_rom.h"
#include "smooth_waypoint_planner/path_processing.h"

namespace smooth_waypoint_planner
{

class SmoothWaypointPlanner : public nav_core::BaseGlobalPlanner
{
public:
  SmoothWaypointPlanner();
  SmoothWaypointPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros) override;
  bool makePlan(
    const geometry_msgs::PoseStamped &start,
    const geometry_msgs::PoseStamped &goal,
    std::vector<geometry_msgs::PoseStamped> &plan) override;
  bool makePlan(
    const geometry_msgs::PoseStamped &start,
    const geometry_msgs::PoseStamped &goal,
    std::vector<geometry_msgs::PoseStamped> &plan,
    double &cost) override;

private:
  bool refreshWaypoints(std::string *error_message = nullptr);
  bool applyWaypoints(
    const std::vector<Point2D> &source_waypoints,
    WaypointSource source,
    std::string *error_message = nullptr);
  void handleLiveWaypoints(const nav_msgs::Path::ConstPtr &path_msg);
  std::vector<Point2D> sanitizeWaypoints(const std::vector<Point2D> &waypoints) const;
  std::vector<Point2D> buildControlPoints(
    const geometry_msgs::PoseStamped &start,
    const geometry_msgs::PoseStamped &goal);
  std::size_t findClosestWaypointIndex(const Point2D &point, double *distance = nullptr) const;
  std::size_t findGoalWaypointIndex(
    const Point2D &point,
    double match_threshold,
    double *distance = nullptr) const;
  std::vector<Point2D> buildTraversalToGoalWaypoint(
    std::size_t start_index,
    std::size_t goal_index) const;
  std::vector<Point2D> prepareSplinePoints(const std::vector<Point2D> &waypoints) const;
  void advanceWaypointIndex(const Point2D &start_point);
  void publishWaypointMarkers() const;
  void publishPreviewPath() const;
  void publishPath(const std::vector<Point2D> &sampled_points, double z_value) const;
  bool reloadWaypoints(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);

  bool initialized_;
  std::string name_;
  std::string global_frame_;
  costmap_2d::Costmap2DROS *costmap_ros_;
  ros::Publisher plan_pub_;
  ros::Publisher marker_pub_;
  ros::ServiceServer reload_service_;
  ros::Subscriber live_waypoints_sub_;
  std::string waypoint_file_;
  std::string waypoints_topic_;
  double resolution_;
  double min_waypoint_spacing_;
  double waypoint_reached_threshold_;
  double corner_rounding_min_turn_deg_;
  double corner_blend_distance_;
  bool loop_waypoints_;
  bool use_waypoint_sequence_goal_;
  bool prefer_live_waypoints_;
  bool live_waypoints_received_;
  std::size_t current_waypoint_index_;
  WaypointSource active_waypoint_source_;
  std::vector<Point2D> live_waypoints_;
  std::vector<Point2D> waypoints_;
};

}  // namespace smooth_waypoint_planner

#endif  // SMOOTH_WAYPOINT_PLANNER__SMOOTH_WAYPOINT_PLANNER_H_
