#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger, TriggerRequest
from robot_navigation.srv import RecognizeWaypoint, RecognizeWaypointRequest  # 新建的服务

class MissionController(object):
    def __init__(self):
        rospy.init_node("mission", anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.navigation_status = "IDLE"
        self.current_index = -1
        self.total_waypoints = 0
        self.last_printed_index = -1

        # 指定需要识别的航点索引（从0开始）
        self.recognition_wp_indices = [2, 6]   # 第3和第7个航点
        self.recognized_flags = {idx: False for idx in self.recognition_wp_indices}

        # 导航服务
        self.start_navigation_srv = rospy.ServiceProxy("/waypoint_editor/start_navigation", Trigger)
        self.stop_navigation_srv = rospy.ServiceProxy("/waypoint_editor/stop_navigation", Trigger)
        self.pause_navigation_srv = rospy.ServiceProxy("/waypoint_editor/pause_navigation", Trigger)
        self.resume_navigation_srv = rospy.ServiceProxy("/waypoint_editor/resume_navigation", Trigger)

        # 识别服务客户端
        rospy.wait_for_service("/recognize_waypoint")
        self.recognize_client = rospy.ServiceProxy("/recognize_waypoint", RecognizeWaypoint)

        # 订阅话题
        rospy.Subscriber("/waypoint_editor/navigation_status", String, self.status_callback, queue_size=10)
        rospy.Subscriber("/waypoint_editor/navigation_index", Int32, self.index_callback, queue_size=10)
        rospy.Subscriber("/waypoint_editor/waypoints_path", Path, self.path_callback, queue_size=1)

        rospy.loginfo("任务控制器已启动")

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
        self.total_waypoints = len(msg.poses)
        rospy.loginfo("成功加载航点路径，共 %d 个航点", self.total_waypoints)

    def pause_navigation(self):
        if self.navigation_status != "FOLLOWING":
            return True
        res = self.pause_navigation_srv(TriggerRequest())
        if not res.success:
            rospy.logerr("暂停导航失败：%s", res.message)
            return False
        # 等待状态变为 PAUSED
        for _ in range(50):
            if self.navigation_status == "PAUSED":
                return True
            rospy.sleep(0.1)
        return False

    def resume_navigation(self):
        res = self.resume_navigation_srv(TriggerRequest())
        if not res.success:
            rospy.logerr("恢复导航失败：%s", res.message)
            return False
        return True

    def do_recognition(self, wp_index):
        """在航点 wp_index 处执行识别"""
        rospy.loginfo("到达指定航点 %d，暂停导航进行识别...", wp_index+1)
        if not self.pause_navigation():
            rospy.logerr("无法暂停导航，跳过识别")
            return

        # 调用识别服务，传入航点编号（从1开始，更直观）
        req = RecognizeWaypointRequest()
        req.waypoint_id = str(wp_index + 1)   # 或者自定义名称
        try:
            res = self.recognize_client(req)
            if res.success:
                rospy.loginfo("识别结果 - 敌方: %d, 友军: %d, 人质: %d",
                              res.enemy_count, res.friend_count, res.hostage_count)
                # 可选：打印保存路径等
                rospy.loginfo("识别图像已保存至: %s", res.saved_image_path)
            else:
                rospy.logerr("识别服务返回失败: %s", res.message)
        except rospy.ServiceException as e:
            rospy.logerr("调用识别服务失败: %s", e)

        # 恢复导航
        if not self.resume_navigation():
            rospy.logerr("无法恢复导航，任务可能中断")

    def run(self):
        rate = rospy.Rate(10)
        start_res = self.start_navigation_srv(TriggerRequest())
        if not start_res.success:
            rospy.logerr("启动航点导航失败：%s", start_res.message)
            return
        rospy.loginfo("航点导航已启动，等待任务完成...")

        while not rospy.is_shutdown():
            # 检查是否到达需要识别的航点
            if self.current_index in self.recognition_wp_indices:
                idx = self.current_index
                if not self.recognized_flags.get(idx, False):
                    self.recognized_flags[idx] = True
                    self.do_recognition(idx)

            if self.navigation_status == "FINISHED":
                rospy.loginfo("导航任务完成")
                return
            if self.navigation_status == "ERROR":
                rospy.logerr("导航进入错误状态")
                return
            rate.sleep()

    def shutdown(self):
        if self.navigation_status in ["FOLLOWING", "PAUSED"]:
            try:
                self.stop_navigation_srv(TriggerRequest())
            except:
                pass
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        MissionController().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("任务结束")