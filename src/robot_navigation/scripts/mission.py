#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger, TriggerRequest

from robot_navigation.srv import detect, detectRequest


class MissionController(object):
    def __init__(self):
        rospy.init_node("mission", anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.navigation_index = -1
        self.navigation_status = "IDLE"
        self.current_yaw = None
        self.current_position = None
        self.waypoint_positions = []
        self.waypoint_path_poses = []
        self.wp3_done = False
        self.wp7_done = False

        # 使用 odom 的平面 yaw 做绝对朝向控制，避免相对转 90 度时受停车朝向误差影响。
        self.turn_angular_speed = rospy.get_param("~turn_angular_speed", 0.6)
        self.turn_min_angular_speed = rospy.get_param("~turn_min_angular_speed", 0.18)
        self.turn_final_angular_speed = rospy.get_param("~turn_final_angular_speed", 0.06)
        self.turn_kp = rospy.get_param("~turn_kp", 1.8)
        self.turn_slowdown_deg = rospy.get_param("~turn_slowdown_deg", 20.0)
        self.turn_tolerance_deg = rospy.get_param("~turn_tolerance_deg", 2.0)
        self.turn_settle_time = rospy.get_param("~turn_settle_time", 0.5)
        self.task_waypoint_tolerance = rospy.get_param("~task_waypoint_tolerance", 0.1)

        self.start_navigation_srv = rospy.ServiceProxy(
            "/waypoint_editor/start_navigation", Trigger)
        self.pause_navigation_srv = rospy.ServiceProxy(
            "/waypoint_editor/pause_navigation", Trigger)
        self.resume_navigation_srv = rospy.ServiceProxy(
            "/waypoint_editor/resume_navigation", Trigger)
        self.stop_navigation_srv = rospy.ServiceProxy(
            "/waypoint_editor/stop_navigation", Trigger)
        self.person_client = rospy.ServiceProxy("/recognize_person", detect)
        self.plate_client = rospy.ServiceProxy("/recognize_plate", detect)

        rospy.loginfo("等待 waypoint_editor 导航服务...")
        rospy.wait_for_service("/waypoint_editor/start_navigation")
        rospy.wait_for_service("/waypoint_editor/pause_navigation")
        rospy.wait_for_service("/waypoint_editor/resume_navigation")
        rospy.wait_for_service("/waypoint_editor/stop_navigation")

        rospy.loginfo("等待识别服务...")
        rospy.wait_for_service("/recognize_person")
        rospy.wait_for_service("/recognize_plate")

        rospy.Subscriber(
            "/waypoint_editor/navigation_index",
            Int32,
            lambda msg: setattr(self, "navigation_index", msg.data),
            queue_size=10)
        rospy.Subscriber(
            "/waypoint_editor/navigation_status",
            String,
            lambda msg: setattr(self, "navigation_status", msg.data.strip().upper()),
            queue_size=10)
        rospy.Subscriber(
            "/waypoint_editor/waypoints_path",
            Path,
            self.waypoints_callback,
            queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~odom_topic", "/odom"),
            Odometry,
            self.odom_callback,
            queue_size=10)

        rospy.loginfo("任务控制器已启动")

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.current_position = (position.x, position.y)

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def waypoints_callback(self, msg):
        self.waypoint_path_poses = list(msg.poses)
        self.waypoint_positions = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def wait_for_yaw(self):
        rate = rospy.Rate(20)
        wait_count = 0
        while not rospy.is_shutdown() and self.current_yaw is None and wait_count < 60:
            rate.sleep()
            wait_count += 1
        return self.current_yaw is not None

    def normalize_degrees(self, angle_deg):
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg <= -180:
            angle_deg += 360
        return angle_deg

    def snap_axis_yaw_deg(self, yaw_deg):
        yaw_deg = self.normalize_degrees(yaw_deg)
        axis_candidates = [0, 90, 180, -90]
        return min(
            axis_candidates,
            key=lambda candidate: abs(self.normalize_degrees(yaw_deg - candidate)))

    def get_waypoint_yaw_deg(self, waypoint_index):
        if waypoint_index < 0 or waypoint_index >= len(self.waypoint_positions):
            return None

        path_msg_index = waypoint_index
        waypoint_path_pose = self.waypoint_path_poses[path_msg_index]
        q = waypoint_path_pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        return int(self.snap_axis_yaw_deg(raw_yaw_deg))

    def rotate_to_yaw(self, target_yaw_deg):
        if target_yaw_deg is None:
            return

        if not self.wait_for_yaw():
            return

        max_speed = abs(self.turn_angular_speed)
        min_speed = abs(self.turn_min_angular_speed)
        final_min_speed = abs(self.turn_final_angular_speed)
        if max_speed < 1e-3:
            return

        target_yaw_deg = int(round(self.normalize_degrees(target_yaw_deg)))
        target_yaw = math.radians(target_yaw_deg)
        tolerance = math.radians(self.turn_tolerance_deg)
        slowdown_error = math.radians(self.turn_slowdown_deg)
        

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            error = self.normalize_angle(target_yaw - self.current_yaw)
            if abs(error) <= tolerance:
                break

            current_max_speed = max_speed
            current_min_speed = min_speed
            if abs(error) <= slowdown_error:
                current_max_speed = max(final_min_speed, max_speed * 0.35)
                current_min_speed = final_min_speed

            angular_z = self.clamp(self.turn_kp * error, -current_max_speed, current_max_speed)
            if abs(angular_z) < current_min_speed and abs(error) > tolerance * 2.0:
                angular_z = current_min_speed if error > 0 else -current_min_speed

            twist = Twist()
            twist.angular.z = angular_z
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(self.turn_settle_time)
   

    def pause_navigation(self):
        if self.navigation_status != "FOLLOWING":
            return True

        pause_result = self.pause_navigation_srv(TriggerRequest())
        if not pause_result.success:
            rospy.logerr("暂停导航失败：%s", pause_result.message)
            return False

        wait_count = 0
        while not rospy.is_shutdown() and self.navigation_status != "PAUSED" and wait_count < 50:
            rospy.sleep(0.1)
            wait_count += 1

        return self.navigation_status == "PAUSED"

    def resume_navigation(self):
        resume_result = self.resume_navigation_srv(TriggerRequest())
        if not resume_result.success:
            rospy.logerr("恢复导航失败：%s", resume_result.message)
            return False

        return True

    def parse_counts(self, result_text):
        community_count = 0
        non_community_count = 0
        if not result_text:
            return community_count, non_community_count

        for item in result_text.split(','):
            item = item.strip()
            if ':' not in item:
                continue

            key, value = item.split(':', 1)
            key = key.strip().lower()
            try:
                count = int(value.strip())
            except ValueError:
                continue

            if key == 'community':
                community_count += count
            elif key == 'non-community':
                non_community_count += count

        return community_count, non_community_count

    def is_near_waypoint(self, waypoint_index):
        # 只有真正接近任务航点时才触发任务，避免被 navigation_index 提前推进影响。
        if self.current_position is None:
            return False
        if waypoint_index < 0 or waypoint_index >= len(self.waypoint_positions):
            return False

        robot_x, robot_y = self.current_position
        waypoint_x, waypoint_y = self.waypoint_positions[waypoint_index]
        distance = math.hypot(waypoint_x - robot_x, waypoint_y - robot_y)
        return distance <= self.task_waypoint_tolerance

    def run(self):
        rate = rospy.Rate(10)

        start_result = self.start_navigation_srv(TriggerRequest())
        if not start_result.success:
            rospy.logerr("启动航点导航失败：%s", start_result.message)
            return

        rospy.loginfo("航点导航已启动")

        while not rospy.is_shutdown():
            if self.navigation_status == "FINISHED":
                rospy.loginfo("全部导航任务完成")
                return

            if self.navigation_status == "ERROR":
                rospy.logerr("导航进入错误状态")
                return

            # navigation_index 从 0 开始，所以第3个航点对应 2，第7个航点对应 6。
            if not self.wp3_done and self.navigation_index >= 2 and self.is_near_waypoint(2):
                rospy.loginfo("到达敌友识别区域始识别")
                if not self.pause_navigation():
                    return

                wp3_yaw_deg = self.get_waypoint_yaw_deg(2)
                if wp3_yaw_deg is None:
                    return

                opposite_yaw_deg = int(round(self.normalize_degrees(wp3_yaw_deg + 180)))

                self.rotate_to_yaw(wp3_yaw_deg)
                result_a = self.person_client(detectRequest(detect_flag=1))

                self.rotate_to_yaw(opposite_yaw_deg)
                result_b = self.person_client(detectRequest(detect_flag=2))

                a_count, outsider_a = self.parse_counts(result_a.result)
                b_count, outsider_b = self.parse_counts(result_b.result)
                a_total = a_count + outsider_a
                b_total = b_count + outsider_b
                total_count = a_total + b_total

                rospy.loginfo(
                    "共检测到 %d 人；其中 A 街总人数 %d 人，B 街总人数 %d 人；发现 A 街非社区人员 %d 人，B 街非社区人员 %d 人。",
                    total_count,
                    a_total,
                    b_total,
                    outsider_a,
                    outsider_b)
                self.wp3_done = True

                if not self.resume_navigation():
                    return

            if not self.wp7_done and self.navigation_index >= 6 and self.is_near_waypoint(6):
                rospy.loginfo("到达停车场开始识别")
                if not self.pause_navigation():
                    return

                wp7_yaw_deg = self.get_waypoint_yaw_deg(6)
                if wp7_yaw_deg is None:

                    return

                self.rotate_to_yaw(wp7_yaw_deg)
                plate_result = self.plate_client(detectRequest(detect_flag=3))
                plate_number = plate_result.result.strip() if plate_result.result else "识别失败"
                rospy.loginfo("1 号停车场车牌号为 %s。", plate_number)
                self.wp7_done = True

                if not self.resume_navigation():
                    return

            rate.sleep()

    def shutdown(self):
        try:
            if self.navigation_status in ["FOLLOWING", "PAUSED"]:
                self.stop_navigation_srv(TriggerRequest())
        except rospy.ServiceException:
            pass

        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        MissionController().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("任务结束")
