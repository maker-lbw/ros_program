#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger, TriggerRequest


class MissionController(object):
    def __init__(self):
        rospy.init_node("mission", anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.navigation_status = "IDLE"
        self.current_index = -1          # 当前正在前往的航点索引（0开始）
        self.total_waypoints = 0         # 总航点数
        self.last_printed_index = -1     # 上次打印的索引，避免重复输出

        # 导航服务代理
        self.start_navigation_srv = rospy.ServiceProxy(
            "/waypoint_editor/start_navigation", Trigger)
        self.stop_navigation_srv = rospy.ServiceProxy(
            "/waypoint_editor/stop_navigation", Trigger)

        rospy.loginfo("等待 waypoint_editor 导航服务...")
        rospy.wait_for_service("/waypoint_editor/start_navigation")
        rospy.wait_for_service("/waypoint_editor/stop_navigation")

        # 订阅导航状态
        rospy.Subscriber(
            "/waypoint_editor/navigation_status",
            String,
            self.status_callback,
            queue_size=10)

        # 订阅当前航点索引
        rospy.Subscriber(
            "/waypoint_editor/navigation_index",
            Int32,
            self.index_callback,
            queue_size=10)

        # 订阅航点路径（获取总航点数）
        rospy.Subscriber(
            "/waypoint_editor/waypoints_path",
            Path,
            self.path_callback,
            queue_size=1)

        rospy.loginfo("任务控制器已启动")

    def status_callback(self, msg):
        self.navigation_status = msg.data.strip().upper()

    def index_callback(self, msg):
        self.current_index = msg.data
        # 当总航点数已知且索引有效且索引发生变化时，输出进度
        if self.total_waypoints > 0 and self.current_index >= 0:
            if self.current_index != self.last_printed_index:
                self.last_printed_index = self.current_index
                # 注意：导航索引表示当前正在前往的航点，通常从0开始
                rospy.loginfo("导航进度: 正在前往第 %d / %d 个航点",
                              self.current_index + 1, self.total_waypoints)

    def path_callback(self, msg):
        self.total_waypoints = len(msg.poses)
        rospy.loginfo("成功加载航点路径，共 %d 个航点", self.total_waypoints)

    def run(self):
        rate = rospy.Rate(10)

        # 启动航点导航
        start_result = self.start_navigation_srv(TriggerRequest())
        if not start_result.success:
            rospy.logerr("启动航点导航失败：%s", start_result.message)
            return
        rospy.loginfo("航点导航已启动，等待任务完成...")

        # 等待导航结束
        while not rospy.is_shutdown():
            if self.navigation_status == "FINISHED":
                rospy.loginfo("导航任务完成")
                return
            if self.navigation_status == "ERROR":
                rospy.logerr("导航错误状态")
                return
            rate.sleep()

    def shutdown(self):
        # 如果导航仍在进行，尝试停止
        if self.navigation_status in ["FOLLOWING", "PAUSED"]:
            try:
                self.stop_navigation_srv(TriggerRequest())
                rospy.loginfo("停止导航")
            except rospy.ServiceException:
                pass
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        MissionController().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("任务结束")