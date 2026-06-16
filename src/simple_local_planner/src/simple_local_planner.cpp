#include <simple_local_planner.h>

#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

template <typename T>
T clampValue(T value, T min_value, T max_value) {
    return std::max(min_value, std::min(max_value, value));
}

double squaredDistance(const geometry_msgs::PoseStamped& lhs,
                       const geometry_msgs::PoseStamped& rhs) {
    const double dx = lhs.pose.position.x - rhs.pose.position.x;
    const double dy = lhs.pose.position.y - rhs.pose.position.y;
    return dx * dx + dy * dy;
}

}  // namespace

PLUGINLIB_EXPORT_CLASS(simple_local_planner::SimpleLocalPlannerROS,
                       nav_core::BaseLocalPlanner)

namespace simple_local_planner {

SimpleLocalPlannerROS::SimpleLocalPlannerROS()
    : tf_buffer_(nullptr),
      costmap_ros_(nullptr),
      initialized_(false),
      goal_reached_(false),
      plan_cursor_(0),
      yaw_tolerance_(0.08),
      xy_goal_tolerance_(0.08),
      lookahead_distance_(0.35),
      min_lookahead_distance_(0.15),
      closest_pose_search_window_(40),
      goal_slowdown_distance_(0.5),
      heading_error_slowdown_(1.5),
      heading_error_gain_(1.8),
      curvature_gain_(0.8),
      final_heading_gain_(1.5),
      min_linear_speed_(0.04),
      max_linear_speed_(0.22),
      max_angular_speed_(0.8),
      allow_final_rotation_(true),
      rotate_in_place_heading_threshold_(1.2) {}

SimpleLocalPlannerROS::~SimpleLocalPlannerROS() {}

void SimpleLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf,
                                       costmap_2d::Costmap2DROS* costmap_ros) {
    if (initialized_) {
        ROS_WARN("SimpleLocalPlannerROS has already been initialized.");
        return;
    }

    ros::NodeHandle private_nh("~/" + name);

    tf_buffer_ = tf;
    costmap_ros_ = costmap_ros;
    base_frame_ = costmap_ros_->getBaseFrameID();

    private_nh.param("yaw_tolerance", yaw_tolerance_, yaw_tolerance_);
    private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, xy_goal_tolerance_);
    private_nh.param("lookahead_distance", lookahead_distance_, lookahead_distance_);
    private_nh.param("min_lookahead_distance", min_lookahead_distance_, min_lookahead_distance_);
    private_nh.param("closest_pose_search_window", closest_pose_search_window_,
                     closest_pose_search_window_);
    private_nh.param("goal_slowdown_distance", goal_slowdown_distance_, goal_slowdown_distance_);
    private_nh.param("heading_error_slowdown", heading_error_slowdown_, heading_error_slowdown_);
    private_nh.param("heading_error_gain", heading_error_gain_, heading_error_gain_);
    private_nh.param("curvature_gain", curvature_gain_, curvature_gain_);
    private_nh.param("final_heading_gain", final_heading_gain_, final_heading_gain_);
    private_nh.param("min_linear_speed", min_linear_speed_, min_linear_speed_);
    private_nh.param("max_linear_speed", max_linear_speed_, max_linear_speed_);
    private_nh.param("max_angular_speed", max_angular_speed_, max_angular_speed_);
    private_nh.param("allow_final_rotation", allow_final_rotation_, allow_final_rotation_);
    private_nh.param("rotate_in_place_heading_threshold", rotate_in_place_heading_threshold_,
                     rotate_in_place_heading_threshold_);

    min_lookahead_distance_ = std::max(0.01, min_lookahead_distance_);
    lookahead_distance_ = std::max(min_lookahead_distance_, lookahead_distance_);
    goal_slowdown_distance_ = std::max(xy_goal_tolerance_, goal_slowdown_distance_);
    min_linear_speed_ = std::max(0.0, min_linear_speed_);
    max_linear_speed_ = std::max(min_linear_speed_, max_linear_speed_);
    max_angular_speed_ = std::max(0.0, max_angular_speed_);
    rotate_in_place_heading_threshold_ =
        std::max(0.0, rotate_in_place_heading_threshold_);

    plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1, true);

    initialized_ = true;
    ROS_INFO("Simple Local Planner initialized in pure path-tracking mode.");
}

bool SimpleLocalPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan) {
    if (!initialized_) {
        ROS_ERROR("SimpleLocalPlannerROS has not been initialized.");
        return false;
    }
    if (plan.empty()) {
        ROS_ERROR("SimpleLocalPlannerROS received an empty plan.");
        return false;
    }

    global_plan_ = plan;
    goal_reached_ = false;
    plan_cursor_ = 0;

    publishPlan(global_plan_);
    ROS_INFO("SimpleLocalPlannerROS received a new global plan with %zu poses.",
             global_plan_.size());
    return true;
}

bool SimpleLocalPlannerROS::isGoalReached() {
    return goal_reached_;
}

double SimpleLocalPlannerROS::normalizeAngle(double angle) const {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

bool SimpleLocalPlannerROS::getCurrentPose(
    geometry_msgs::PoseStamped& robot_pose) const {
    geometry_msgs::PoseStamped stamped_ident;
    stamped_ident.header.frame_id = base_frame_;
    stamped_ident.header.stamp = ros::Time(0);
    tf2::toMsg(tf2::Transform::getIdentity(), stamped_ident.pose);

    try {
        robot_pose = tf_buffer_->transform(stamped_ident,
                                           costmap_ros_->getGlobalFrameID());
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("Failed to get robot pose: %s", ex.what());
        return false;
    }
    return true;
}

void SimpleLocalPlannerROS::publishPlan(
    const std::vector<geometry_msgs::PoseStamped>& path) const {
    if (!initialized_) {
        return;
    }

    nav_msgs::Path gui_path;
    gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
    gui_path.header.stamp = ros::Time::now();
    gui_path.poses = path;
    plan_pub_.publish(gui_path);
}

std::size_t SimpleLocalPlannerROS::findClosestPlanIndex(
    const geometry_msgs::PoseStamped& robot_pose) const {
    if (global_plan_.empty()) {
        return 0;
    }

    const std::size_t start_index =
        std::min(plan_cursor_, global_plan_.size() - 1);
    std::size_t end_index = global_plan_.size() - 1;
    if (closest_pose_search_window_ > 0) {
        end_index = std::min(end_index,
                             start_index + static_cast<std::size_t>(closest_pose_search_window_));
    }

    std::size_t best_index = start_index;
    double best_distance = std::numeric_limits<double>::infinity();

    for (std::size_t i = start_index; i <= end_index; ++i) {
        const double current_distance = squaredDistance(robot_pose, global_plan_[i]);
        if (current_distance < best_distance) {
            best_distance = current_distance;
            best_index = i;
        }
    }

    return best_index;
}

std::size_t SimpleLocalPlannerROS::findLookaheadPlanIndex(
    std::size_t start_index, double lookahead_distance) const {
    if (global_plan_.empty()) {
        return 0;
    }

    std::size_t lookahead_index =
        std::min(start_index, global_plan_.size() - 1);
    double accumulated_distance = 0.0;

    for (std::size_t i = lookahead_index + 1; i < global_plan_.size(); ++i) {
        const double dx = global_plan_[i].pose.position.x -
                          global_plan_[i - 1].pose.position.x;
        const double dy = global_plan_[i].pose.position.y -
                          global_plan_[i - 1].pose.position.y;
        accumulated_distance += std::hypot(dx, dy);
        if (accumulated_distance >= lookahead_distance) {
            return i;
        }
    }

    return global_plan_.size() - 1;
}

double SimpleLocalPlannerROS::remainingPlanDistance(
    const geometry_msgs::PoseStamped& robot_pose, std::size_t start_index) const {
    if (global_plan_.empty()) {
        return 0.0;
    }

    const std::size_t clamped_start =
        std::min(start_index, global_plan_.size() - 1);
    double remaining_distance =
        std::hypot(global_plan_[clamped_start].pose.position.x - robot_pose.pose.position.x,
                   global_plan_[clamped_start].pose.position.y - robot_pose.pose.position.y);

    for (std::size_t i = clamped_start + 1; i < global_plan_.size(); ++i) {
        const double dx = global_plan_[i].pose.position.x -
                          global_plan_[i - 1].pose.position.x;
        const double dy = global_plan_[i].pose.position.y -
                          global_plan_[i - 1].pose.position.y;
        remaining_distance += std::hypot(dx, dy);
    }

    return remaining_distance;
}

bool SimpleLocalPlannerROS::computeVelocityCommands(
    geometry_msgs::Twist& cmd_vel) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    if (!initialized_) {
        ROS_ERROR("SimpleLocalPlannerROS has not been initialized.");
        return false;
    }
    if (global_plan_.empty()) {
        ROS_WARN_THROTTLE(1.0, "SimpleLocalPlannerROS has no global plan to track.");
        return true;
    }
    if (goal_reached_) {
        return true;
    }

    geometry_msgs::PoseStamped current_pose;
    if (!getCurrentPose(current_pose)) {
        return false;
    }

    plan_cursor_ = findClosestPlanIndex(current_pose);

    const geometry_msgs::PoseStamped& final_goal_pose = global_plan_.back();
    const double current_yaw = tf2::getYaw(current_pose.pose.orientation);
    const double dist_to_final_goal =
        std::hypot(final_goal_pose.pose.position.x - current_pose.pose.position.x,
                   final_goal_pose.pose.position.y - current_pose.pose.position.y);
    const double remaining_plan_distance =
        remainingPlanDistance(current_pose, plan_cursor_);
    const bool near_plan_end =
        (plan_cursor_ + 1U >= global_plan_.size()) ||
        remaining_plan_distance <= std::max(lookahead_distance_, xy_goal_tolerance_ * 2.0);

    if (dist_to_final_goal <= xy_goal_tolerance_ && near_plan_end) {
        const double final_yaw = tf2::getYaw(final_goal_pose.pose.orientation);
        const double final_yaw_error = normalizeAngle(final_yaw - current_yaw);

        if (!allow_final_rotation_ || std::abs(final_yaw_error) <= yaw_tolerance_) {
            goal_reached_ = true;
            plan_cursor_ = global_plan_.size() - 1;
            publishPlan(std::vector<geometry_msgs::PoseStamped>(1, final_goal_pose));
            ROS_INFO("SimpleLocalPlannerROS goal reached.");
            return true;
        }

        cmd_vel.angular.z = clampValue(final_heading_gain_ * final_yaw_error,
                                       -max_angular_speed_, max_angular_speed_);
        publishPlan(std::vector<geometry_msgs::PoseStamped>(1, final_goal_pose));
        return true;
    }

    std::vector<geometry_msgs::PoseStamped> remaining_plan(
        global_plan_.begin() + static_cast<std::ptrdiff_t>(plan_cursor_),
        global_plan_.end());
    publishPlan(remaining_plan);

    const double desired_lookahead =
        std::max(min_lookahead_distance_,
                 std::min(lookahead_distance_, dist_to_final_goal));
    const std::size_t lookahead_index =
        findLookaheadPlanIndex(plan_cursor_, desired_lookahead);

    geometry_msgs::PoseStamped target_pose_map = global_plan_[lookahead_index];
    target_pose_map.header.stamp = ros::Time(0);

    geometry_msgs::PoseStamped target_pose_base;
    try {
        target_pose_base = tf_buffer_->transform(target_pose_map, base_frame_);
    } catch (tf2::TransformException& ex) {
        ROS_ERROR("Failed to transform lookahead pose: %s", ex.what());
        return false;
    }

    const double target_x = target_pose_base.pose.position.x;
    const double target_y = target_pose_base.pose.position.y;
    const double target_distance = std::hypot(target_x, target_y);
    const double heading_error = std::atan2(target_y, target_x);

    double heading_scale = 1.0;
    if (heading_error_slowdown_ > 1e-3) {
        heading_scale = clampValue(1.0 - std::abs(heading_error) / heading_error_slowdown_,
                                   0.0, 1.0);
    } else if (std::abs(heading_error) > 1e-3) {
        heading_scale = 0.0;
    }

    double goal_scale = 1.0;
    if (goal_slowdown_distance_ > 1e-3) {
        goal_scale = clampValue(dist_to_final_goal / goal_slowdown_distance_, 0.0, 1.0);
    }

    double linear_speed = max_linear_speed_ * heading_scale * goal_scale;
    if (std::abs(heading_error) > rotate_in_place_heading_threshold_) {
        linear_speed = 0.0;
    } else if (linear_speed > 0.0) {
        linear_speed = std::max(min_linear_speed_, linear_speed);
    }

    double curvature = 0.0;
    if (target_distance > 1e-3) {
        curvature = 2.0 * target_y / (target_distance * target_distance);
    }

    const double angular_speed =
        heading_error_gain_ * heading_error + curvature_gain_ * curvature * linear_speed;

    cmd_vel.linear.x = clampValue(linear_speed, 0.0, max_linear_speed_);
    cmd_vel.angular.z =
        clampValue(angular_speed, -max_angular_speed_, max_angular_speed_);
    return true;
}

}  // namespace simple_local_planner
