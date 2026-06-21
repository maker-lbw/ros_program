#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
YOLO 识别节点（ROS服务版）
功能：
- 订阅摄像头图像话题（默认 /image_raw），缓存最新一帧
- 提供服务 /recognize_waypoint，接收航点ID（字符串）
- 对当前图像进行YOLO推理，统计三类（enemy, friend_army, hostage）的数量
- 保存带标注的图像到 yolo_ros/results/ 目录
- 保存原始摄像头图像到 yolo_ros/picture/camera/ 目录（便于后续分析）
- 返回识别结果（各类数量、图像保存路径）

依赖：
- ultralytics, opencv-python, cv_bridge
- 服务类型 RecognizeWaypoint（由 robot_navigation 包提供）
"""

import os
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

# 导入自定义服务（若服务定义在 yolo_ros 包，请相应修改）
from robot_navigation.srv import RecognizeWaypoint, RecognizeWaypointResponse


class RecognizerNode:
    """YOLO识别节点类"""

    def __init__(self):
        rospy.init_node("recognizer_node", anonymous=True)

        # ---------- 自动定位功能包根目录 ----------
        # 获取当前脚本所在目录（假设为 .../yolo_ros/scripts/）
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # 功能包根目录（scripts 的父目录）
        self.pkg_dir = os.path.dirname(script_dir)

        # ---------- 硬编码路径（相对于功能包根目录） ----------
        self.model_path = os.path.join(self.pkg_dir, "weights", "best.pt")
        # 带标注的结果图像保存目录
        self.annotated_dir = os.path.join(self.pkg_dir, "results")
        # 原始摄像头图像保存目录
        self.original_dir = os.path.join(self.pkg_dir, "picture", "camera")

        # 类别名称（必须与训练时 classes.txt 顺序一致）
        self.class_names = ["enemy", "friend_army", "hostage"]

        # ---------- 初始化ROS组件 ----------
        self.bridge = CvBridge()
        self.latest_image = None   # 存储最新一帧（BGR格式）

        # 加载YOLO模型（若路径不存在则报错）
        if not os.path.exists(self.model_path):
            rospy.logerr("模型文件不存在: %s", self.model_path)
            rospy.signal_shutdown("模型文件缺失")
        rospy.loginfo("加载模型: %s", self.model_path)
        self.model = YOLO(self.model_path)

        # 订阅图像话题（默认改为 /image_raw，可通过参数调整）
        image_topic = rospy.get_param("~image_topic", "/image_raw")
        rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)
        rospy.loginfo("订阅图像话题: %s", image_topic)

        # 提供服务
        self.service = rospy.Service(
            "/recognize_waypoint",
            RecognizeWaypoint,
            self.handle_recognize
        )
        rospy.loginfo("识别节点已启动，等待识别请求...")

    def image_callback(self, msg):
        """将ROS图像转为OpenCV BGR并缓存"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn("图像转换失败: %s", e)

    def handle_recognize(self, req):
        """
        服务回调：执行识别，返回结果
        req.waypoint_id: 航点标识（字符串）
        """
        response = RecognizeWaypointResponse()

        # 检查是否有图像
        if self.latest_image is None:
            response.success = False
            response.message = "未收到图像，请检查相机话题"
            return response

        # ---------- 保存原始图像 ----------
        # 确保目录存在
        os.makedirs(self.original_dir, exist_ok=True)
        # 生成文件名（基于航点ID）
        wp_id = req.waypoint_id if req.waypoint_id else "unknown"
        original_filename = f"waypoint_{wp_id}_original.jpg"
        original_path = os.path.join(self.original_dir, original_filename)
        # 保存原始图像（BGR）
        cv2.imwrite(original_path, self.latest_image)
        rospy.loginfo("原始图像已保存至: %s", original_path)

        # ---------- YOLO推理 ----------
        results = self.model(self.latest_image)[0]   # 取第一张图

        # 统计各类数量
        counts = {cls: 0 for cls in self.class_names}
        for cls_id in results.boxes.cls.int().tolist():
            if cls_id < len(self.class_names):
                counts[self.class_names[cls_id]] += 1
            else:
                rospy.logwarn("发现未知类别id: %d", cls_id)

        # ---------- 保存带标注的图像 ----------
        os.makedirs(self.annotated_dir, exist_ok=True)
        annotated_filename = f"waypoint_{wp_id}.jpg"
        annotated_path = os.path.join(self.annotated_dir, annotated_filename)
        annotated_img = results.plot()   # 绘制检测框
        cv2.imwrite(annotated_path, annotated_img)

        # ---------- 保存文本计数 ----------
        txt_path = os.path.join(self.annotated_dir, f"waypoint_{wp_id}.txt")
        with open(txt_path, "w") as f:
            f.write(f"Waypoint {wp_id}\n")
            for cls, cnt in counts.items():
                f.write(f"{cls}: {cnt}\n")
            f.write(f"Saved image: {annotated_path}\n")
            f.write(f"Original image: {original_path}\n")

        # 填充响应
        response.success = True
        response.message = "识别完成"
        response.enemy_count = counts["enemy"]
        response.friend_count = counts["friend_army"]
        response.hostage_count = counts["hostage"]
        response.saved_image_path = annotated_path  # 保留标注图路径（也可改为原始图）

        rospy.loginfo("航点 %s 识别完成，结果保存至 %s", wp_id, self.annotated_dir)
        return response


if __name__ == "__main__":
    try:
        node = RecognizerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass