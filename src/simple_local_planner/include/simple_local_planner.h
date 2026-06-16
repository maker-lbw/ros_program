#ifndef SIMPLE_LOCAL_PLANNER_H_
#define SIMPLE_LOCAL_PLANNER_H_

#include <cstddef>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

namespace simple_local_planner {

class SimpleLocalPlannerROS : public nav_core::BaseLocalPlanner {
public:
    SimpleLocalPlannerROS();
    ~SimpleLocalPlannerROS() override;

    void initialize(std::string name, tf2_ros::Buffer* tf,
                    costmap_2d::Costmap2DROS* costmap_ros) override;

    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    bool isGoalReached() override;

private:
    double normalizeAngle(double angle) const;
    bool getCurrentPose(geometry_msgs::PoseStamped& robot_pose) const;
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) const;
    std::size_t findClosestPlanIndex(const geometry_msgs::PoseStamped& robot_pose) const;
    std::size_t findLookaheadPlanIndex(std::size_t start_index, double lookahead_distance) const;
    double remainingPlanDistance(const geometry_msgs::PoseStamped& robot_pose,
                                 std::size_t start_index) const;

    tf2_ros::Buffer* tf_buffer_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    ros::Publisher plan_pub_;

    bool initialized_;
    bool goal_reached_;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    std::string base_frame_;
    std::size_t plan_cursor_;

    double yaw_tolerance_;
    double xy_goal_tolerance_;
    double lookahead_distance_;
    double min_lookahead_distance_;
    int closest_pose_search_window_;
    double goal_slowdown_distance_;
    double heading_error_slowdown_;
    double heading_error_gain_;
    double curvature_gain_;
    double final_heading_gain_;
    double min_linear_speed_;
    double max_linear_speed_;
    double max_angular_speed_;
    bool allow_final_rotation_;
    double rotate_in_place_heading_threshold_;
};

}  // namespace simple_local_planner

#endif  // SIMPLE_LOCAL_PLANNER_H_
