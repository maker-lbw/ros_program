#include "smooth_waypoint_planner/smooth_waypoint_planner.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include "smooth_waypoint_planner/catmull_rom.h"
#include "smooth_waypoint_planner/path_processing.h"

namespace smooth_waypoint_planner
{
namespace
{

std::string Trim(const std::string &text)
{
  const auto first = std::find_if_not(text.begin(), text.end(), [](unsigned char c) { return std::isspace(c); });
  if (first == text.end()) {
    return {};
  }
  const auto last = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char c) { return std::isspace(c); }).base();
  return std::string(first, last);
}

std::vector<std::string> SplitCsvRow(const std::string &line)
{
  std::vector<std::string> columns;
  std::stringstream stream(line);
  std::string token;
  while (std::getline(stream, token, ',')) {
    columns.push_back(token);
  }
  return columns;
}

bool ParseDouble(const std::string &text, double &value)
{
  try {
    size_t processed = 0U;
    const std::string trimmed = Trim(text);
    value = std::stod(trimmed, &processed);
    return processed == trimmed.size();
  } catch (const std::exception &) {
    return false;
  }
}

double Distance(const Point2D &a, const Point2D &b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::sqrt((dx * dx) + (dy * dy));
}

double Dot(const Point2D &lhs, const Point2D &rhs)
{
  return (lhs.x * rhs.x) + (lhs.y * rhs.y);
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

struct SegmentProjection
{
  bool valid = false;
  double progress = 0.0;
  double lateral_distance = std::numeric_limits<double>::infinity();
};

SegmentProjection ProjectPointOntoSegment(
  const Point2D &point,
  const Point2D &segment_start,
  const Point2D &segment_end)
{
  const Point2D segment{segment_end.x - segment_start.x, segment_end.y - segment_start.y};
  const double segment_length_sq = Dot(segment, segment);
  if (segment_length_sq <= std::numeric_limits<double>::epsilon()) {
    return {};
  }

  const Point2D from_start{point.x - segment_start.x, point.y - segment_start.y};
  const double raw_progress = Dot(from_start, segment) / segment_length_sq;
  const double clamped_progress = std::max(0.0, std::min(1.0, raw_progress));
  const Point2D projected_point{
    segment_start.x + (segment.x * clamped_progress),
    segment_start.y + (segment.y * clamped_progress)};

  SegmentProjection projection;
  projection.valid = true;
  projection.progress = clamped_progress;
  projection.lateral_distance = Distance(point, projected_point);
  return projection;
}

Point2D ComputeWaypointPassDirection(const std::vector<Point2D> &waypoints, std::size_t waypoint_index)
{
  if (waypoints.empty() || waypoint_index >= waypoints.size()) {
    return {};
  }

  if (waypoint_index + 1U >= waypoints.size()) {
    if (waypoint_index == 0U) {
      return {};
    }

    return NormalizeVector(Point2D{
      waypoints[waypoint_index].x - waypoints[waypoint_index - 1U].x,
      waypoints[waypoint_index].y - waypoints[waypoint_index - 1U].y});
  }

  const Point2D outgoing = NormalizeVector(Point2D{
    waypoints[waypoint_index + 1U].x - waypoints[waypoint_index].x,
    waypoints[waypoint_index + 1U].y - waypoints[waypoint_index].y});
  if (waypoint_index == 0U) {
    return outgoing;
  }

  const Point2D incoming = NormalizeVector(Point2D{
    waypoints[waypoint_index].x - waypoints[waypoint_index - 1U].x,
    waypoints[waypoint_index].y - waypoints[waypoint_index - 1U].y});
  const Point2D blended = NormalizeVector(Point2D{incoming.x + outgoing.x, incoming.y + outgoing.y});
  if (std::abs(blended.x) <= std::numeric_limits<double>::epsilon() &&
      std::abs(blended.y) <= std::numeric_limits<double>::epsilon())
  {
    return outgoing;
  }

  return blended;
}

Point2D ToPoint2D(const geometry_msgs::PoseStamped &pose)
{
  Point2D point;
  point.x = pose.pose.position.x;
  point.y = pose.pose.position.y;
  return point;
}

void AppendIfFarEnough(std::vector<Point2D> &points, const Point2D &candidate, double minimum_distance)
{
  if (points.empty() || Distance(points.back(), candidate) >= minimum_distance) {
    points.push_back(candidate);
  }
}

double ComputeYaw(const std::vector<Point2D> &sampled_points, std::size_t index)
{
  if (sampled_points.size() < 2U) {
    return 0.0;
  }

  if (index == 0U) {
    return std::atan2(
      sampled_points[1U].y - sampled_points[0U].y,
      sampled_points[1U].x - sampled_points[0U].x);
  }

  if (index + 1U >= sampled_points.size()) {
    return std::atan2(
      sampled_points[index].y - sampled_points[index - 1U].y,
      sampled_points[index].x - sampled_points[index - 1U].x);
  }

  return std::atan2(
    sampled_points[index + 1U].y - sampled_points[index - 1U].y,
    sampled_points[index + 1U].x - sampled_points[index - 1U].x);
}

bool LoadWaypointsFromCsvFile(const std::string &path, std::vector<Point2D> &waypoints, std::string &error)
{
  std::ifstream input(path);
  if (!input.is_open()) {
    error = "Failed to open file for reading: " + path;
    return false;
  }

  std::string line;
  std::getline(input, line);

  std::vector<Point2D> parsed;
  while (std::getline(input, line)) {
    if (Trim(line).empty()) {
      continue;
    }

    const std::vector<std::string> columns = SplitCsvRow(line);
    if (columns.size() < 3U) {
      continue;
    }

    Point2D point;
    if (!ParseDouble(columns[1], point.x)) {
      continue;
    }
    if (!ParseDouble(columns[2], point.y)) {
      continue;
    }
    parsed.push_back(point);
  }

  waypoints = std::move(parsed);
  error.clear();
  return true;
}

}  // namespace

SmoothWaypointPlanner::SmoothWaypointPlanner()
: initialized_(false),
  costmap_ros_(nullptr),
  resolution_(0.05),
  min_waypoint_spacing_(0.05),
  waypoint_reached_threshold_(0.2),
  corner_rounding_min_turn_deg_(30.0),
  corner_blend_distance_(0.3),
  loop_waypoints_(false),
  use_waypoint_sequence_goal_(true),
  prefer_live_waypoints_(true),
  live_waypoints_received_(false),
  current_waypoint_index_(0U),
  active_waypoint_source_(WaypointSource::kNone)
{
}

SmoothWaypointPlanner::SmoothWaypointPlanner(
  std::string name,
  costmap_2d::Costmap2DROS *costmap_ros)
: SmoothWaypointPlanner()
{
  initialize(std::move(name), costmap_ros);
}

void SmoothWaypointPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
  if (initialized_) {
    ROS_WARN("SmoothWaypointPlanner is already initialized");
    return;
  }

  name_ = std::move(name);
  costmap_ros_ = costmap_ros;
  global_frame_ = costmap_ros_ != nullptr ? costmap_ros_->getGlobalFrameID() : "map";

  ros::NodeHandle private_nh("~");
  ros::NodeHandle planner_nh(private_nh, name_);
  planner_nh.param("waypoint_file", waypoint_file_, std::string());
  planner_nh.param("waypoints_topic", waypoints_topic_, std::string("/waypoint_editor/waypoints_path"));
  planner_nh.param("resolution", resolution_, 0.05);
  planner_nh.param("min_waypoint_spacing", min_waypoint_spacing_, 0.05);
  planner_nh.param("waypoint_reached_threshold", waypoint_reached_threshold_, 0.2);
  planner_nh.param("corner_rounding_min_turn_deg", corner_rounding_min_turn_deg_, 30.0);
  planner_nh.param("corner_blend_distance", corner_blend_distance_, 0.3);
  planner_nh.param("loop_waypoints", loop_waypoints_, false);
  planner_nh.param("use_waypoint_sequence_goal", use_waypoint_sequence_goal_, true);
  planner_nh.param("prefer_live_waypoints", prefer_live_waypoints_, true);

  if (min_waypoint_spacing_ < 0.0) {
    min_waypoint_spacing_ = 0.0;
  }
  if (resolution_ <= 0.0) {
    resolution_ = 0.05;
  }
  if (waypoint_reached_threshold_ < 0.0) {
    waypoint_reached_threshold_ = 0.0;
  }
  corner_rounding_min_turn_deg_ = std::max(0.0, std::min(corner_rounding_min_turn_deg_, 175.0));
  if (corner_blend_distance_ < 0.0) {
    corner_blend_distance_ = 0.0;
  }

  plan_pub_ = planner_nh.advertise<nav_msgs::Path>("smooth_plan", 1, true);
  marker_pub_ = planner_nh.advertise<visualization_msgs::MarkerArray>("waypoint_markers", 1, true);
  reload_service_ = planner_nh.advertiseService(
    "reload_waypoints", &SmoothWaypointPlanner::reloadWaypoints, this);
  if (!waypoints_topic_.empty()) {
    live_waypoints_sub_ = planner_nh.subscribe<nav_msgs::Path>(
      waypoints_topic_, 1, &SmoothWaypointPlanner::handleLiveWaypoints, this);
  }

  std::string error_message;
  if (!refreshWaypoints(&error_message)) {
    ROS_WARN_STREAM("SmoothWaypointPlanner failed to load waypoints: " << error_message);
  }

  initialized_ = true;
}

bool SmoothWaypointPlanner::makePlan(
  const geometry_msgs::PoseStamped &start,
  const geometry_msgs::PoseStamped &goal,
  std::vector<geometry_msgs::PoseStamped> &plan)
{
  double cost = 0.0;
  return makePlan(start, goal, plan, cost);
}

bool SmoothWaypointPlanner::makePlan(
  const geometry_msgs::PoseStamped &start,
  const geometry_msgs::PoseStamped &goal,
  std::vector<geometry_msgs::PoseStamped> &plan,
  double &cost)
{
  plan.clear();
  cost = 0.0;

  if (!initialized_) {
    ROS_ERROR("SmoothWaypointPlanner is not initialized");
    return false;
  }

  if (waypoints_.empty()) {
    std::string error_message;
    if (!refreshWaypoints(&error_message)) {
      if (waypoint_file_.empty() && live_waypoints_.empty()) {
        const std::vector<Point2D> direct_points{ToPoint2D(start), ToPoint2D(goal)};
        const std::vector<Point2D> sampled_points = direct_points;
        const std::string frame_id = global_frame_.empty() ? start.header.frame_id : global_frame_;
        const ros::Time stamp = ros::Time::now();
        plan.reserve(sampled_points.size());
        for (std::size_t index = 0; index < sampled_points.size(); ++index) {
          geometry_msgs::PoseStamped pose;
          pose.header.frame_id = frame_id;
          pose.header.stamp = stamp;
          pose.pose.position.x = sampled_points[index].x;
          pose.pose.position.y = sampled_points[index].y;
          pose.pose.position.z = start.pose.position.z;

          tf2::Quaternion orientation;
          orientation.setRPY(0.0, 0.0, ComputeYaw(sampled_points, index));
          pose.pose.orientation = tf2::toMsg(orientation);
          plan.push_back(pose);
        }

        publishPath(sampled_points, start.pose.position.z);
        cost = Distance(ToPoint2D(start), ToPoint2D(goal));
        return true;
      }

      ROS_ERROR_STREAM("SmoothWaypointPlanner cannot make plan: " << error_message);
      return false;
    }
  }

  const std::vector<Point2D> control_points = buildControlPoints(start, goal);
  if (control_points.size() < 2U) {
    ROS_WARN("SmoothWaypointPlanner does not have enough control points");
    return false;
  }

  CatmullRomSpline spline;
  spline.setControlPoints(control_points);
  const std::vector<Point2D> sampled_points = spline.sample(resolution_);
  if (sampled_points.size() < 2U) {
    ROS_WARN("SmoothWaypointPlanner generated an empty path");
    return false;
  }

  const std::string frame_id = global_frame_.empty() ? start.header.frame_id : global_frame_;
  const ros::Time stamp = ros::Time::now();
  plan.reserve(sampled_points.size());
  for (std::size_t index = 0; index < sampled_points.size(); ++index) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = stamp;
    pose.pose.position.x = sampled_points[index].x;
    pose.pose.position.y = sampled_points[index].y;
    pose.pose.position.z = start.pose.position.z;

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, ComputeYaw(sampled_points, index));
    pose.pose.orientation = tf2::toMsg(orientation);
    plan.push_back(pose);
  }

  publishPath(sampled_points, start.pose.position.z);
  cost = static_cast<double>(plan.size()) * resolution_;
  return true;
}

bool SmoothWaypointPlanner::refreshWaypoints(std::string *error_message)
{
  const WaypointSource source = SelectWaypointSource({
    prefer_live_waypoints_,
    live_waypoints_received_,
    !live_waypoints_.empty(),
    !waypoint_file_.empty()});

  if (source == WaypointSource::kLiveTopic) {
    return applyWaypoints(live_waypoints_, WaypointSource::kLiveTopic, error_message);
  }

  if (source == WaypointSource::kFile) {
    std::vector<Point2D> raw_waypoints;
    std::string io_error;
    if (!LoadWaypointsFromCsvFile(waypoint_file_, raw_waypoints, io_error)) {
      if (error_message != nullptr) {
        *error_message = io_error;
      }
      waypoints_.clear();
      current_waypoint_index_ = 0U;
      active_waypoint_source_ = WaypointSource::kNone;
      publishWaypointMarkers();
      publishPath({}, 0.0);
      return false;
    }
    return applyWaypoints(raw_waypoints, WaypointSource::kFile, error_message);
  }

  if (error_message != nullptr) {
    *error_message = live_waypoints_received_
      ? "live waypoint topic is empty and no waypoint_file is configured"
      : "no live waypoints received and waypoint_file is empty";
  }
  waypoints_.clear();
  current_waypoint_index_ = 0U;
  active_waypoint_source_ = WaypointSource::kNone;
  publishWaypointMarkers();
  publishPath({}, 0.0);
  return false;
}

bool SmoothWaypointPlanner::applyWaypoints(
  const std::vector<Point2D> &source_waypoints,
  WaypointSource source,
  std::string *error_message)
{
  waypoints_ = sanitizeWaypoints(source_waypoints);
  current_waypoint_index_ = 0U;
  active_waypoint_source_ = source;
  if (waypoints_.empty()) {
    if (error_message != nullptr) {
      *error_message = source == WaypointSource::kLiveTopic
        ? "live waypoint topic is empty"
        : "no valid waypoint found in CSV";
    }
    active_waypoint_source_ = WaypointSource::kNone;
    publishWaypointMarkers();
    publishPath({}, 0.0);
    return false;
  }

  publishWaypointMarkers();
  publishPreviewPath();
  return true;
}

void SmoothWaypointPlanner::handleLiveWaypoints(const nav_msgs::Path::ConstPtr &path_msg)
{
  if (path_msg == nullptr) {
    return;
  }

  live_waypoints_received_ = true;
  live_waypoints_ = ConvertPathToPoints(*path_msg);

  if (prefer_live_waypoints_ || waypoint_file_.empty()) {
    std::string error_message;
    if (!refreshWaypoints(&error_message) && !error_message.empty()) {
      ROS_WARN_STREAM("SmoothWaypointPlanner live waypoint update: " << error_message);
    }
  }
}

std::vector<Point2D> SmoothWaypointPlanner::sanitizeWaypoints(const std::vector<Point2D> &waypoints) const
{
  std::vector<Point2D> filtered_waypoints;
  filtered_waypoints.reserve(waypoints.size());

  for (const Point2D &point : waypoints) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }

    if (
      filtered_waypoints.empty() ||
      Distance(filtered_waypoints.back(), point) >= min_waypoint_spacing_)
    {
      filtered_waypoints.push_back(point);
    }
  }

  return filtered_waypoints;
}

std::vector<Point2D> SmoothWaypointPlanner::buildControlPoints(
  const geometry_msgs::PoseStamped &start,
  const geometry_msgs::PoseStamped &goal)
{
  std::vector<Point2D> control_points;
  const Point2D start_point = ToPoint2D(start);
  const Point2D goal_point = ToPoint2D(goal);
  const double goal_match_threshold = std::max(min_waypoint_spacing_, waypoint_reached_threshold_);

  advanceWaypointIndex(start_point);

  const bool enforce_sequence_goal = use_waypoint_sequence_goal_ && !loop_waypoints_ && !waypoints_.empty();
  double goal_waypoint_distance = std::numeric_limits<double>::infinity();
  std::size_t goal_waypoint_index = waypoints_.size();
  if (enforce_sequence_goal) {
    goal_waypoint_index = waypoints_.size() - 1U;
    goal_waypoint_distance = 0.0;
  } else {
    goal_waypoint_index =
      findGoalWaypointIndex(goal_point, goal_match_threshold, &goal_waypoint_distance);
  }

  std::size_t traversal_start_index = current_waypoint_index_;
  if (enforce_sequence_goal && traversal_start_index >= waypoints_.size()) {
    const double final_waypoint_distance = Distance(start_point, waypoints_.back());
    if (final_waypoint_distance > waypoint_reached_threshold_ && waypoints_.size() >= 2U) {
      traversal_start_index = waypoints_.size() - 2U;
      current_waypoint_index_ = traversal_start_index;
      ROS_WARN_STREAM_THROTTLE(
        1.0,
        "SmoothWaypointPlanner corrected over-advanced waypoint index to "
          << traversal_start_index << " while final waypoint is still "
          << final_waypoint_distance << " m away");
    } else if (!waypoints_.empty()) {
      traversal_start_index = waypoints_.size() - 1U;
    }
  }

  std::vector<Point2D> traversal;
  if (enforce_sequence_goal) {
    traversal = buildTraversalToGoalWaypoint(traversal_start_index, goal_waypoint_index);
  } else if (goal_waypoint_index < waypoints_.size() && goal_waypoint_distance <= goal_match_threshold) {
    traversal = buildTraversalToGoalWaypoint(traversal_start_index, goal_waypoint_index);
  } else {
    traversal = BuildWaypointTraversal(waypoints_, traversal_start_index, loop_waypoints_);
  }

  if (
    enforce_sequence_goal &&
    traversal.size() < 2U &&
    waypoints_.size() >= 2U &&
    Distance(start_point, waypoints_.back()) > waypoint_reached_threshold_)
  {
    const std::size_t fallback_start_index = std::min(traversal_start_index, waypoints_.size() - 2U);
    traversal = buildTraversalToGoalWaypoint(fallback_start_index, goal_waypoint_index);
    ROS_WARN_STREAM_THROTTLE(
      1.0,
      "SmoothWaypointPlanner expanded short sequence traversal from index "
        << fallback_start_index << " to preserve waypoint-route semantics");
  }

  control_points.reserve(traversal.size() + 2U);
  control_points.push_back(start_point);

  for (const Point2D &waypoint : traversal) {
    AppendIfFarEnough(control_points, waypoint, min_waypoint_spacing_);
  }

  AppendIfFarEnough(control_points, goal_point, min_waypoint_spacing_);

  if (control_points.size() == 1U && !waypoints_.empty()) {
    control_points.push_back(waypoints_.back());
  }

  ROS_INFO_STREAM_THROTTLE(
    1.0,
    "SmoothWaypointPlanner plan state: current_index=" << current_waypoint_index_
      << ", traversal_start=" << traversal_start_index
      << ", goal_index=" << goal_waypoint_index
      << ", traversal_points=" << traversal.size()
      << ", control_points=" << control_points.size());

  return prepareSplinePoints(control_points);
}

std::size_t SmoothWaypointPlanner::findClosestWaypointIndex(const Point2D &point, double *distance) const
{
  std::size_t best_index = waypoints_.size();
  double best_distance = std::numeric_limits<double>::infinity();

  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    const double candidate_distance = Distance(point, waypoints_[index]);
    if (candidate_distance < best_distance) {
      best_distance = candidate_distance;
      best_index = index;
    }
  }

  if (distance != nullptr) {
    *distance = best_distance;
  }
  return best_index;
}

std::size_t SmoothWaypointPlanner::findGoalWaypointIndex(
  const Point2D &point,
  const double match_threshold,
  double *distance) const
{
  const double tolerance = 1e-6;
  std::size_t best_index = waypoints_.size();
  double best_distance = std::numeric_limits<double>::infinity();

  if (!waypoints_.empty()) {
    const std::size_t start_index = loop_waypoints_
      ? 0U
      : std::min(current_waypoint_index_, waypoints_.size() - 1U);

    if (!loop_waypoints_ && match_threshold > 0.0) {
      for (std::size_t index = start_index; index < waypoints_.size(); ++index) {
        const double candidate_distance = Distance(point, waypoints_[index]);
        if (candidate_distance <= match_threshold + tolerance) {
          best_index = index;
          best_distance = candidate_distance;
        }
      }
    }

    if (best_index >= waypoints_.size()) {
      for (std::size_t index = start_index; index < waypoints_.size(); ++index) {
        const double candidate_distance = Distance(point, waypoints_[index]);
        if (
          candidate_distance + tolerance < best_distance ||
          (std::abs(candidate_distance - best_distance) <= tolerance && index > best_index))
        {
          best_distance = candidate_distance;
          best_index = index;
        }
      }
    }
  }

  if (best_index >= waypoints_.size()) {
    best_index = findClosestWaypointIndex(point, &best_distance);
  }

  if (distance != nullptr) {
    *distance = best_distance;
  }
  return best_index;
}

std::vector<Point2D> SmoothWaypointPlanner::buildTraversalToGoalWaypoint(
  std::size_t start_index,
  std::size_t goal_index) const
{
  if (waypoints_.empty() || goal_index >= waypoints_.size()) {
    return {};
  }

  if (!loop_waypoints_ && start_index >= waypoints_.size()) {
    return {waypoints_[goal_index]};
  }

  const std::size_t clamped_start = std::min(start_index, waypoints_.size() - 1U);

  if (!loop_waypoints_) {
    if (goal_index < clamped_start) {
      return {waypoints_[goal_index]};
    }

    std::vector<Point2D> segment;
    segment.reserve((goal_index - clamped_start) + 1U);
    segment.insert(
      segment.end(),
      waypoints_.begin() + static_cast<std::ptrdiff_t>(clamped_start),
      waypoints_.begin() + static_cast<std::ptrdiff_t>(goal_index + 1U));
    return segment;
  }

  std::vector<Point2D> segment;
  segment.reserve(waypoints_.size());
  std::size_t index = clamped_start;
  while (true) {
    segment.push_back(waypoints_[index]);
    if (index == goal_index) {
      break;
    }
    index = (index + 1U) % waypoints_.size();
  }
  return segment;
}

std::vector<Point2D> SmoothWaypointPlanner::prepareSplinePoints(const std::vector<Point2D> &waypoints) const
{
  return PrepareWaypointsForSpline(
    waypoints,
    {min_waypoint_spacing_, corner_rounding_min_turn_deg_, corner_blend_distance_});
}

void SmoothWaypointPlanner::advanceWaypointIndex(const Point2D &start_point)
{
  const double segment_capture_distance = std::max(waypoint_reached_threshold_ * 1.5, 0.3);
  const double pass_plane_margin = std::max(0.02, waypoint_reached_threshold_ * 0.25);

  while (current_waypoint_index_ < waypoints_.size()) {
    const Point2D &current_waypoint = waypoints_[current_waypoint_index_];
    const double current_distance = Distance(start_point, current_waypoint);
    if (current_distance <= waypoint_reached_threshold_) {
      ++current_waypoint_index_;
      continue;
    }

    if (current_waypoint_index_ + 1U >= waypoints_.size()) {
      break;
    }

    const Point2D &next_waypoint = waypoints_[current_waypoint_index_ + 1U];
    const double next_distance = Distance(start_point, next_waypoint);
    const double segment_length = Distance(current_waypoint, next_waypoint);
    const SegmentProjection outgoing_projection =
      ProjectPointOntoSegment(start_point, current_waypoint, next_waypoint);

    const double segment_progress_threshold = std::max(
      0.15,
      std::min(0.6, waypoint_reached_threshold_ / std::max(segment_length, waypoint_reached_threshold_)));
    const bool passed_on_outgoing_segment =
      outgoing_projection.valid &&
      outgoing_projection.progress >= segment_progress_threshold &&
      outgoing_projection.lateral_distance <= segment_capture_distance;

    const Point2D pass_direction = ComputeWaypointPassDirection(waypoints_, current_waypoint_index_);
    const Point2D from_current{start_point.x - current_waypoint.x, start_point.y - current_waypoint.y};
    const double pass_plane_progress = Dot(from_current, pass_direction);
    const bool crossed_pass_plane =
      (std::abs(pass_direction.x) > std::numeric_limits<double>::epsilon() ||
       std::abs(pass_direction.y) > std::numeric_limits<double>::epsilon()) &&
      pass_plane_progress >= pass_plane_margin &&
      next_distance <= current_distance + segment_capture_distance;

    if (!passed_on_outgoing_segment && !crossed_pass_plane) {
      break;
    }

    ROS_INFO_STREAM_THROTTLE(
      1.0,
      "SmoothWaypointPlanner advanced waypoint index from " << current_waypoint_index_
        << " to " << (current_waypoint_index_ + 1U)
        << " (current_distance=" << current_distance
        << ", next_distance=" << next_distance
        << ", segment_progress=" << outgoing_projection.progress
        << ", lateral_distance=" << outgoing_projection.lateral_distance
        << ", pass_plane_progress=" << pass_plane_progress << ")");
    ++current_waypoint_index_;
  }
}

void SmoothWaypointPlanner::publishWaypointMarkers() const
{
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker clear_marker;
  clear_marker.action = visualization_msgs::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  if (waypoints_.empty()) {
    marker_pub_.publish(marker_array);
    return;
  }

  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = global_frame_;
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = name_;
  point_marker.id = 0;
  point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.pose.orientation.w = 1.0;
  point_marker.scale.x = 0.15;
  point_marker.scale.y = 0.15;
  point_marker.scale.z = 0.15;
  point_marker.color.r = 1.0F;
  point_marker.color.g = 0.4F;
  point_marker.color.b = 0.0F;
  point_marker.color.a = 1.0F;

  visualization_msgs::Marker line_marker = point_marker;
  line_marker.id = 1;
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.scale.x = 0.05;
  line_marker.color.r = 0.0F;
  line_marker.color.g = 0.8F;
  line_marker.color.b = 1.0F;

  for (const Point2D &waypoint : waypoints_) {
    geometry_msgs::Point point;
    point.x = waypoint.x;
    point.y = waypoint.y;
    point.z = 0.0;
    point_marker.points.push_back(point);
    line_marker.points.push_back(point);
  }

  marker_array.markers.push_back(point_marker);
  marker_array.markers.push_back(line_marker);
  marker_pub_.publish(marker_array);
}

void SmoothWaypointPlanner::publishPreviewPath() const
{
  if (waypoints_.empty()) {
    publishPath({}, 0.0);
    return;
  }

  if (waypoints_.size() == 1U) {
    publishPath(waypoints_, 0.0);
    return;
  }

  std::vector<Point2D> preview_points = BuildWaypointTraversal(waypoints_, 0U, loop_waypoints_);
  if (loop_waypoints_ && !preview_points.empty()) {
    AppendIfFarEnough(preview_points, preview_points.front(), min_waypoint_spacing_);
  }

  CatmullRomSpline spline;
  preview_points = prepareSplinePoints(preview_points);
  if (preview_points.size() < 2U) {
    publishPath(preview_points, 0.0);
    return;
  }

  spline.setControlPoints(preview_points);
  publishPath(spline.sample(resolution_), 0.0);
}

void SmoothWaypointPlanner::publishPath(const std::vector<Point2D> &sampled_points, double z_value) const
{
  nav_msgs::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = ros::Time::now();

  for (std::size_t index = 0; index < sampled_points.size(); ++index) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = sampled_points[index].x;
    pose.pose.position.y = sampled_points[index].y;
    pose.pose.position.z = z_value;

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, ComputeYaw(sampled_points, index));
    pose.pose.orientation = tf2::toMsg(orientation);
    path.poses.push_back(pose);
  }

  plan_pub_.publish(path);
}

bool SmoothWaypointPlanner::reloadWaypoints(
  std_srvs::Trigger::Request & /*request*/,
  std_srvs::Trigger::Response &response)
{
  std::string error_message;
  response.success = refreshWaypoints(&error_message);
  response.message = response.success ? "waypoints reloaded" : error_message;
  return true;
}

}  // namespace smooth_waypoint_planner

PLUGINLIB_EXPORT_CLASS(
  smooth_waypoint_planner::SmoothWaypointPlanner,
  nav_core::BaseGlobalPlanner)
