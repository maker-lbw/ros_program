#!/usr/bin/env python3

import rospy
import cv2
import torch
import numpy as np
import threading
import os
import sys
import pathlib

# ========== 修复 WindowsPath 问题 ==========
if sys.platform == 'linux' and not hasattr(pathlib, 'WindowsPath'):
    class WindowsPath(pathlib.Path):
        __slots__ = ()
    pathlib.WindowsPath = WindowsPath
# ==========================================

# 导入 yolov5 包
import yolov5

from cv_bridge import CvBridge
from rostopic import get_topic_type
from robot_navigation.srv import * 
from sensor_msgs.msg import Image, CompressedImage
from yolo_ros.msg import BoundingBox, BoundingBoxes
from std_msgs.msg import Header

class YoloDetector:
    def __init__(self):
        # --- 从 ROS 参数服务器加载参数 ---
        weights_path = rospy.get_param("~weights")
        self.conf_thres = rospy.get_param("~confidence_threshold", 0.45)
        self.iou_thres = rospy.get_param("~iou_threshold", 0.5)
        self.max_det = rospy.get_param("~maximum_detections", 300)
        self.classes = rospy.get_param("~classes", None)
        self.device = str(rospy.get_param("~device", "cpu"))
        self.img_size = rospy.get_param("~inference_size", 640)
        
        # 可视化参数
        self.view_image = rospy.get_param("~view_image", True)
        self.publish_image = rospy.get_param("~publish_image", True)
        self.line_thickness = rospy.get_param("~line_thickness", 2)
        
        # --- 加载 YOLOv5 模型 ---
        try:
            self.model = yolov5.load(weights_path, device=self.device)
            self.model.conf = self.conf_thres
            self.model.iou = self.iou_thres
            self.model.max_det = self.max_det
            self.model_names = self.model.names
            rospy.loginfo("YOLOv5 model loaded successfully from %s", weights_path)
        except Exception as e:
            rospy.logerr(f"Failed to load YOLOv5 model: {e}")
            rospy.signal_shutdown(f"Model loading error: {e}")
            return
            
        # --- 存储路径和最新图像 ---
        self.save_path = '/home/lbw/workspace/ros_program/rk_ws/src/robot_navigation/scripts/pic/'
        self.latest_image = None
        self.latest_header = Header()
        self.image_lock = threading.Lock()
        
        self.bridge = CvBridge()

        # --- 初始化 ROS 订阅者 ---
        input_image_topic = rospy.get_param("~input_image_topic", "/camera/color/image_raw")
        input_image_type, _, _ = get_topic_type(input_image_topic, blocking=True)
        self.compressed_input = "CompressedImage" in input_image_type

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic, CompressedImage, self.image_callback, queue_size=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                input_image_topic, Image, self.image_callback, queue_size=1
            )
        
        # --- 初始化 ROS 发布者 ---
        self.pred_pub = rospy.Publisher(
            rospy.get_param("~output_topic", "/yolo/detections"), BoundingBoxes, queue_size=10
        )
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic", "/yolo/detection_image"), Image, queue_size=10
            )

        # --- 初始化 ROS 服务 ---
        self.service = rospy.Service('/recognize_person', detect, self.detect_callback)
        rospy.loginfo("YOLO Detector Service '/recognize_person' is ready.")

    def image_callback(self, msg):
        """存储最新的图像和其Header"""
        try:
            if self.compressed_input:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            with self.image_lock:
                self.latest_image = cv_image.copy()
                self.latest_header = msg.header
                
            if self.view_image:
                cv2.imshow("YOLO Live View", cv_image)
                cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Image conversion error: {str(e)}")

    def detect_callback(self, req):
        """ROS 服务回调函数，执行目标检测并返回每个类别的计数"""
        if req.detect_flag == 1 or req.detect_flag == 2:
            rospy.loginfo("Received detection request, starting YOLO inference...")
            
            # 获取最新图像
            with self.image_lock:
                if self.latest_image is not None:
                    im0 = self.latest_image.copy()
                    current_header = self.latest_header
                else:
                    rospy.logwarn("No image available for detection.")
                    return detectResponse("ERROR: No image available")

            # --- YOLOv5 推理 ---
            results = self.model(im0, size=self.img_size)
            detections = results.xyxy[0].cpu().numpy()  # shape (N, 6): x1,y1,x2,y2,conf,cls

            if len(detections) == 0:
                boxes = []
                confs = []
                class_ids = []
            else:
                boxes = detections[:, :4].astype(int)
                confs = detections[:, 4].tolist()
                class_ids = detections[:, 5].astype(int).tolist()

            # 手动绘制标注图像
            annotated_frame = im0.copy()
            for box, conf, cls_id in zip(boxes, confs, class_ids):
                x1, y1, x2, y2 = box
                label = f"{self.model_names[cls_id]}: {conf:.2f}"
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), self.line_thickness)
                cv2.putText(annotated_frame, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

            # 创建消息并统计
            bounding_boxes_msg = BoundingBoxes()
            bounding_boxes_msg.header = current_header
            bounding_boxes_msg.image_header = current_header
            class_counts = {}

            for box, conf, cls_id in zip(boxes, confs, class_ids):
                class_name = self.model_names[cls_id]
                class_counts[class_name] = class_counts.get(class_name, 0) + 1
                
                bbox_msg = BoundingBox()
                bbox_msg.Class = class_name
                bbox_msg.probability = conf
                bbox_msg.xmin = box[0]
                bbox_msg.ymin = box[1]
                bbox_msg.xmax = box[2]
                bbox_msg.ymax = box[3]
                bounding_boxes_msg.bounding_boxes.append(bbox_msg)

            self.pred_pub.publish(bounding_boxes_msg)

            if self.publish_image:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8"))

            if self.view_image:
                cv2.imshow("YOLO Detection Result", annotated_frame)
                cv2.waitKey(1)
                
            # 保存图像
            try:
                save_dir = os.path.dirname(self.save_path)
                if not os.path.exists(save_dir):
                    os.makedirs(save_dir)
                filename = f"{req.detect_flag}.jpg"
                full_path = os.path.join(self.save_path, filename)
                cv2.imwrite(full_path, annotated_frame)
                rospy.loginfo(f"Detection result saved to: {full_path}")
            except Exception as e:
                rospy.logerr(f"Failed to save image: {e}")

            response_items = [f"{cls}:{count}" for cls, count in class_counts.items()]
            response_str = ", ".join(sorted(response_items)) if response_items else "None"
            rospy.loginfo(f"Detection finished. Counts: {response_str}")
            return detectResponse(response_str)

if __name__ == "__main__":
    rospy.init_node("yolo_detector_node", anonymous=True)
    detector = None
    try:
        detector = YoloDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if detector and detector.view_image:
            cv2.destroyAllWindows()