#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
YOLO 识别节点（ROS服务版）
功能：
- 订阅摄像头图像，缓存最新帧
- 提供服务 /recognize_waypoint，接收航点ID
- 对图像进行YOLO推理，统计三类数量
- 保存原始图像到 raw_image_dir
- 保存标注图像和文本计数到 results_dir
- 所有路径均通过参数服务器配置
"""

import os
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from robot_navigation.srv import RecognizeWaypoint, RecognizeWaypointResponse


class RecognizerNode:
    def __init__(self):
        rospy.init_node("recognizer_node", anonymous=True)

        # ---------- 从参数服务器读取配置 ----------
        # 功能包根目录（自动定位）
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.pkg_dir = os.path.dirname(script_dir)  # yolo_ros 根目录

        # 模型路径（相对或绝对）
        model_rel = rospy.get_param("~model_path", "weights/best.pt")
        self.model_path = os.path.join(self.pkg_dir, model_rel)

        # 图像话题
        self.image_topic = rospy.get_param("~image_topic", "/image_raw")

        # 结果保存目录（相对功能包根目录）
        results_rel = rospy.get_param("~results_dir", "results")
        self.results_dir = os.path.join(self.pkg_dir, results_rel)

        # 原始图像保存目录
        raw_rel = rospy.get_param("~raw_image_dir", "picture/camera")
        self.raw_image_dir = os.path.join(self.pkg_dir, raw_rel)

        # 类别名称（必须与训练时一致）
        self.class_names = ["enemy", "friend_army", "hostage"]

        # ---------- 初始化ROS ----------
        self.bridge = CvBridge()
        self.latest_image = None

        # 加载模型
        if not os.path.exists(self.model_path):
            rospy.logerr("模型文件不存在: %s", self.model_path)
            rospy.signal_shutdown("模型文件缺失")
        rospy.loginfo("加载模型: %s", self.model_path)
        self.model = YOLO(self.model_path)

        # 订阅图像
        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        rospy.loginfo("订阅图像话题: %s", self.image_topic)

        # 提供服务
        self.service = rospy.Service("/recognize_waypoint", RecognizeWaypoint, self.handle_recognize)
        rospy.loginfo("识别节点已启动，等待识别请求...")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn("图像转换失败: %s", e)

    def handle_recognize(self, req):
        response = RecognizeWaypointResponse()

        if self.latest_image is None:
            response.success = False
            response.message = "未收到图像，请检查相机话题"
            return response

        wp_id = req.waypoint_id if req.waypoint_id else "unknown"

        # ---------- 1. 保存原始图像 ----------
        os.makedirs(self.raw_image_dir, exist_ok=True)
        raw_filename = f"raw_waypoint_{wp_id}.jpg"
        raw_path = os.path.join(self.raw_image_dir, raw_filename)
        cv2.imwrite(raw_path, self.latest_image)
        rospy.loginfo("原始图像已保存至: %s", raw_path)

        # ---------- 2. YOLO推理 ----------
        results = self.model(self.latest_image)[0]

        # 统计各类数量
        counts = {cls: 0 for cls in self.class_names}
        for cls_id in results.boxes.cls.int().tolist():
            if cls_id < len(self.class_names):
                counts[self.class_names[cls_id]] += 1
            else:
                rospy.logwarn("发现未知类别id: %d", cls_id)

        # ---------- 3. 保存标注图像和文本 ----------
        os.makedirs(self.results_dir, exist_ok=True)

        annotated_img = results.plot()
        ann_filename = f"annotated_waypoint_{wp_id}.jpg"
        ann_path = os.path.join(self.results_dir, ann_filename)
        cv2.imwrite(ann_path, annotated_img)

        txt_path = os.path.join(self.results_dir, f"waypoint_{wp_id}.txt")
        with open(txt_path, "w") as f:
            f.write(f"Waypoint {wp_id}\n")
            for cls, cnt in counts.items():
                f.write(f"{cls}: {cnt}\n")
            f.write(f"原始图像: {raw_path}\n")
            f.write(f"标注图像: {ann_path}\n")

        # ---------- 4. 填充响应 ----------
        response.success = True
        response.message = "识别完成"
        response.enemy_count = counts["enemy"]
        response.friend_count = counts["friend_army"]
        response.hostage_count = counts["hostage"]
        response.saved_image_path = ann_path

        rospy.loginfo("航点 %s 识别完成，结果保存至 %s", wp_id, self.results_dir)
        return response


if __name__ == "__main__":
    try:
        node = RecognizerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass