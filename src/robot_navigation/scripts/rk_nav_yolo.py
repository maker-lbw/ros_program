#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
导航任务控制器（带识别和旋转）
功能：
- 启动/暂停/恢复/停止 waypoint_editor 导航
- 在指定航点暂停，旋转到目标朝向，执行YOLO识别（通过服务调用）
- 在仅旋转航点（如最后一个）仅旋转，不识别
- 格式化输出识别结果（战区1/战区2）
- 所有可调参数均从参数服务器读取
"""

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger, TriggerRequest
from robot_navigation.srv import RecognizeWaypoint, RecognizeWaypointRequest


class MissionController(object):
    def __init__(self):
        rospy.init_node("mission_controller", anonymous=False)
        rospy.on_shutdown(self.shutdown)

        # ---------- 发布器 ----------
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # ---------- 状态变量 ----------
        self.navigation_status = "IDLE"
        self.current_index = -1
        self.total_waypoints = 0
        self.last_printed_index = -1

        self.current_position = None
        self.current_yaw = None

        self.waypoint_path_poses = []
        self.waypoint_positions = []

        # ---------- 从参数服务器读取任务相关参数 ----------
        # 需要识别并打印结果的航点索引（列表，如 [4, 13]）
        self.recognition_wp_indices = rospy.get_param("~recognition_wp_indices", [4, 13])
        self.recognized_flags = {idx: False for idx in self.recognition_wp_indices}

        # 仅需要旋转到预设朝向的航点索引（列表，如 [18]）
        self.rotation_only_wp_indices = rospy.get_param("~rotation_only_wp_indices", [18])
        self.rotated_flags = {idx: False for idx in self.rotation_only_wp_indices}

        # 到达判断距离阈值
        self.task_waypoint_tolerance = rospy.get_param("~task_waypoint_tolerance", 0.2)

        # ---------- 旋转控制参数 ----------
        self.turn_angular_speed = rospy.get_param("~turn_angular_speed", 0.6)
        self.turn_min_angular_speed = rospy.get_param("~turn_min_angular_speed", 0.18)
        self.turn_final_angular_speed = rospy.get_param("~turn_final_angular_speed", 0.06)
        self.turn_kp = rospy.get_param("~turn_kp", 1.8)
        self.turn_slowdown_deg = rospy.get_param("~turn_slowdown_deg", 20.0)
        self.turn_tolerance_deg = rospy.get_param("~turn_tolerance_deg", 2.0)
        self.turn_settle_time = rospy.get_param("~turn_settle_time", 0.5)

        # ---------- 导航服务代理 ----------
        rospy.loginfo("等待 waypoint_editor 导航服务...")
        self.start_navigation_srv = rospy.ServiceProxy("/waypoint_editor/start_navigation", Trigger)
        self.stop_navigation_srv = rospy.ServiceProxy("/waypoint_editor/stop_navigation", Trigger)
        self.pause_navigation_srv = rospy.ServiceProxy("/waypoint_editor/pause_navigation", Trigger)
        self.resume_navigation_srv = rospy.ServiceProxy("/waypoint_editor/resume_navigation", Trigger)

        rospy.wait_for_service("/waypoint_editor/start_navigation")
        rospy.wait_for_service("/waypoint_editor/stop_navigation")
        rospy.wait_for_service("/waypoint_editor/pause_navigation")
        rospy.wait_for_service("/waypoint_editor/resume_navigation")

        # ---------- 识别服务客户端 ----------
        rospy.loginfo("等待识别服务 /recognize_waypoint ...")
        rospy.wait_for_service("/recognize_waypoint")
        self.recognize_client = rospy.ServiceProxy("/recognize_waypoint", RecognizeWaypoint)

        # ---------- 话题订阅 ----------
        rospy.Subscriber("/waypoint_editor/navigation_status", String, self.status_callback, queue_size=10)
        rospy.Subscriber("/waypoint_editor/navigation_index", Int32, self.index_callback, queue_size=10)
        rospy.Subscriber("/waypoint_editor/waypoints_path", Path, self.path_callback, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=10)

        rospy.loginfo("任务控制器已启动")
        rospy.loginfo("识别航点索引: %s", self.recognition_wp_indices)
        rospy.loginfo("仅旋转航点索引: %s", self.rotation_only_wp_indices)

    # ---------- 回调函数 ----------
    def status_callback(self, msg):
        self.navigation_status = msg.data.strip().upper()

    def index_callback(self, msg):
        self.current_index = msg.data
        if self.total_waypoints > 0 and self.current_index >= 0:
            if self.current_index != self.last_printed_index:
                self.last_printed_index = self.current_index
                rospy.loginfo("导航进度: 正在前往第 %d / %d 个航点",
                              self.current_index + 1, self.total_waypoints)

    def path_callback(self, msg):
        self.waypoint_path_poses = list(msg.poses)
        self.waypoint_positions = [
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
        ]
        self.total_waypoints = len(msg.poses)
        rospy.loginfo("成功加载航点路径，共 %d 个航点", self.total_waypoints)

    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ---------- 导航控制 ----------
    def pause_navigation(self):
        if self.navigation_status == "PAUSED":
            return True
        rospy.loginfo("尝试暂停导航，当前状态: %s", self.navigation_status)
        try:
            res = self.pause_navigation_srv(TriggerRequest())
        except rospy.ServiceException as e:
            rospy.logerr("调用暂停服务异常: %s", e)
            return False
        if not res.success:
            rospy.logerr("暂停导航失败: %s", res.message)
            return False
        for _ in range(50):
            if self.navigation_status == "PAUSED":
                rospy.loginfo("导航已成功暂停")
                return True
            rospy.sleep(0.1)
        rospy.logerr("暂停导航超时，当前状态: %s", self.navigation_status)
        return False

    def resume_navigation(self):
        try:
            res = self.resume_navigation_srv(TriggerRequest())
        except rospy.ServiceException as e:
            rospy.logerr("恢复服务异常: %s", e)
            return False
        if not res.success:
            rospy.logerr("恢复导航失败: %s", res.message)
            return False
        rospy.loginfo("导航已恢复")
        return True

    # ---------- 旋转辅助 ----------
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

    def get_waypoint_yaw_deg(self, waypoint_index):
        """提取航点的目标偏航角（度），不吸附"""
        if waypoint_index < 0 or waypoint_index >= len(self.waypoint_path_poses):
            return None
        pose = self.waypoint_path_poses[waypoint_index].pose
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        # 归一化到 [-180, 180]
        while raw_yaw_deg > 180:
            raw_yaw_deg -= 360
        while raw_yaw_deg <= -180:
            raw_yaw_deg += 360
        return raw_yaw_deg

    def rotate_to_yaw(self, target_yaw_deg):
        if target_yaw_deg is None:
            rospy.logwarn("目标偏航角无效，跳过旋转")
            return
        if not self.wait_for_yaw():
            rospy.logwarn("无法获取当前偏航角，跳过旋转")
            return

        max_speed = abs(self.turn_angular_speed)
        min_speed = abs(self.turn_min_angular_speed)
        final_min_speed = abs(self.turn_final_angular_speed)
        if max_speed < 1e-3:
            return

        target_yaw = math.radians(target_yaw_deg)
        tolerance = math.radians(self.turn_tolerance_deg)
        slowdown_error = math.radians(self.turn_slowdown_deg)

        rospy.loginfo("旋转到目标角度 %.1f° (当前 %.1f°)", target_yaw_deg, math.degrees(self.current_yaw))

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
        rospy.loginfo("旋转完成")

    # ---------- 辅助判断 ----------
    def is_near_waypoint(self, index):
        if self.current_position is None or index >= len(self.waypoint_positions):
            return False
        dx = self.current_position[0] - self.waypoint_positions[index][0]
        dy = self.current_position[1] - self.waypoint_positions[index][1]
        return math.hypot(dx, dy) <= self.task_waypoint_tolerance

    # ---------- 执行识别任务（含旋转） ----------
    def do_recognition(self, wp_index):
        rospy.loginfo("到达指定航点 %d，暂停导航进行识别...", wp_index + 1)

        # 1. 暂停
        if not self.pause_navigation():
            rospy.logerr("暂停导航失败，跳过识别")
            return

        # 2. 旋转到目标朝向
        target_yaw = self.get_waypoint_yaw_deg(wp_index)
        if target_yaw is not None:
            self.rotate_to_yaw(target_yaw)
        else:
            rospy.logwarn("无法获取航点 %d 的目标朝向", wp_index + 1)

        # 3. 调用识别服务
        req = RecognizeWaypointRequest()
        req.waypoint_id = str(wp_index + 1)
        try:
            res = self.recognize_client(req)
            if res.success:
                # 根据航点索引确定战区编号
                if wp_index == 4:
                    zone_name = "战区1"
                elif wp_index == 13:
                    zone_name = "战区2"
                else:
                    zone_name = f"航点{wp_index+1}"
                rospy.loginfo("%s - 敌军：%d  友军：%d  人质：%d",
                              zone_name,
                              res.enemy_count,
                              res.friend_count,
                              res.hostage_count)
                rospy.loginfo("图像已保存至: %s", res.saved_image_path)
            else:
                rospy.logerr("识别服务返回失败: %s", res.message)
        except rospy.ServiceException as e:
            rospy.logerr("调用识别服务失败: %s", e)

        # 4. 恢复导航
        if not self.resume_navigation():
            rospy.logerr("恢复导航失败，任务可能中断")

    # ---------- 执行仅旋转任务 ----------
    def do_rotation_only(self, wp_index):
        rospy.loginfo("到达航点 %d，暂停导航进行旋转（不识别）...", wp_index + 1)

        if not self.pause_navigation():
            rospy.logerr("暂停导航失败，跳过旋转")
            return

        target_yaw = self.get_waypoint_yaw_deg(wp_index)
        if target_yaw is not None:
            self.rotate_to_yaw(target_yaw)
        else:
            rospy.logwarn("无法获取航点 %d 的目标朝向", wp_index + 1)

        if not self.resume_navigation():
            rospy.logerr("恢复导航失败")

    # ---------- 主循环 ----------
    def run(self):
        rate = rospy.Rate(10)

        start_res = self.start_navigation_srv(TriggerRequest())
        if not start_res.success:
            rospy.logerr("启动航点导航失败：%s", start_res.message)
            return
        rospy.loginfo("航点导航已启动，等待任务完成...")

        while not rospy.is_shutdown():
            # 检查识别航点
            if self.current_index in self.recognition_wp_indices:
                idx = self.current_index
                if not self.recognized_flags.get(idx, False) and self.is_near_waypoint(idx):
                    self.recognized_flags[idx] = True
                    self.do_recognition(idx)

            # 检查仅旋转航点
            if self.current_index in self.rotation_only_wp_indices:
                idx = self.current_index
                if not self.rotated_flags.get(idx, False) and self.is_near_waypoint(idx):
                    self.rotated_flags[idx] = True
                    self.do_rotation_only(idx)

            # 状态检查
            if self.navigation_status == "FINISHED":
                rospy.loginfo("导航任务完成")
                return
            if self.navigation_status == "ERROR":
                rospy.logerr("导航进入错误状态")
                return

            rate.sleep()

    # ---------- 关闭回调 ----------
    def shutdown(self):
        if self.navigation_status in ["FOLLOWING", "PAUSED"]:
            try:
                self.stop_navigation_srv(TriggerRequest())
            except Exception:
                pass
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        MissionController().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("任务结束")