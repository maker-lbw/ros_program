#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from robot_navigation.srv import RecognizeWaypoint, RecognizeWaypointResponse

class RecognizerNode:
    def __init__(self):
        rospy.init_node("recognizer_node", anonymous=True)
        self.bridge = CvBridge()
        self.latest_image = None

        # 加载YOLO模型（请修改路径为你的模型文件）
        model_path = rospy.get_param("~model_path", "/path/to/yolo11n.pt")
        self.model = YOLO(model_path)

        # 类别名称（与classes.txt一致）
        self.class_names = ["enemy", "friend_army", "hostage"]

        # 订阅相机图像（根据实际话题修改）
        image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)

        # 服务
        self.service = rospy.Service("/recognize_waypoint", RecognizeWaypoint, self.handle_recognize)
        rospy.loginfo("识别节点已启动，等待识别请求...")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn("图像转换失败: %s", e)

    def handle_recognize(self, req):
        """服务回调：执行识别并返回结果"""
        response = RecognizeWaypointResponse()
        if self.latest_image is None:
            response.success = False
            response.message = "未收到图像"
            return response

        # 推理
        results = self.model(self.latest_image)[0]  # 取第一张图的结果

        # 统计各类别数量
        counts = {cls: 0 for cls in self.class_names}
        for cls_id in results.boxes.cls.int().tolist():
            cls_name = self.class_names[cls_id] if cls_id < len(self.class_names) else "unknown"
            if cls_name in counts:
                counts[cls_name] += 1

        # 保存带标注的图像
        save_dir = os.path.expanduser("~/detection_results")
        os.makedirs(save_dir, exist_ok=True)
        waypoint_id = req.waypoint_id if req.waypoint_id else "unknown"
        img_path = os.path.join(save_dir, f"waypoint_{waypoint_id}.jpg")
        annotated_img = results.plot()   # 获取绘制了边界框的图像
        cv2.imwrite(img_path, annotated_img)

        # 保存文本记录
        txt_path = os.path.join(save_dir, f"waypoint_{waypoint_id}.txt")
        with open(txt_path, "w") as f:
            f.write(f"Waypoint {waypoint_id}\n")
            for cls, cnt in counts.items():
                f.write(f"{cls}: {cnt}\n")

        # 填充响应
        response.success = True
        response.message = "识别完成"
        response.enemy_count = counts["enemy"]
        response.friend_count = counts["friend_army"]
        response.hostage_count = counts["hostage"]
        response.saved_image_path = img_path

        rospy.loginfo("航点 %s 识别完成，结果已保存", waypoint_id)
        return response

if __name__ == "__main__":
    try:
        node = RecognizerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass