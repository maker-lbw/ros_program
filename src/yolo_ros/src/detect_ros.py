#!/usr/bin/env python3

import rospy
import cv2
import torch
import numpy as np
import threading
import os

from ultralytics import YOLO
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
        self.classes = rospy.get_param("~classes", None) # 格式: [0, 1, 2] or None
        self.device = str(rospy.get_param("~device", "cpu")) # e.g., "cpu", "0", "1"
        self.img_size = rospy.get_param("~inference_size", 640)
        
        # 可视化参数
        self.view_image = rospy.get_param("~view_image", True)
        self.publish_image = rospy.get_param("~publish_image", True)
        self.line_thickness = rospy.get_param("~line_thickness", 2)#绘制检测结果的线宽
        
        # --- 初始化 YOLO 模型 ---
        try:
            self.model = YOLO(weights_path) #载入.pt文件
            self.model.to(self.device)
            rospy.loginfo("YOLO model loaded successfully from %s", weights_path)
        except Exception as e:
            rospy.logerr(f"Error loading YOLO model: {e}")
            rospy.signal_shutdown(f"Error loading YOLO model: {e}")
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
                # 仅显示原始图像，检测后的图像在服务回调中显示
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

            # --- 使用 ultralytics YOLO 进行推理 ---
            results = self.model(
                im0,
                conf=self.conf_thres,
                iou=self.iou_thres,
                imgsz=self.img_size,
                classes=self.classes,
                max_det=self.max_det,
                verbose=False
            )
            
            result = results[0]

            # --- 创建并填充 BoundingBoxes 消息 ---
            bounding_boxes_msg = BoundingBoxes()
            bounding_boxes_msg.header = current_header
            bounding_boxes_msg.image_header = current_header

            # =使用字典来统计每个检测到的类别数量 ===
            class_counts = {}


            if len(result.boxes) > 0:
                rospy.loginfo(f"Detected {len(result.boxes)} objects.")
                for box in result.boxes:
                    bbox_msg = BoundingBox()
                    
                    cls_id = int(box.cls.item())
                    class_name = self.model.names[cls_id]
                    
                    # =更新类别计数 ===
                    class_counts[class_name] = class_counts.get(class_name, 0) + 1
                    
                    xyxy = box.xyxy.cpu().numpy().flatten().astype(int)
                    conf = box.conf.item()
                    
                    bbox_msg.Class = class_name
                    bbox_msg.probability = conf
                    bbox_msg.xmin = xyxy[0]
                    bbox_msg.ymin = xyxy[1]
                    bbox_msg.xmax = xyxy[2]
                    bbox_msg.ymax = xyxy[3]
                    
                    bounding_boxes_msg.bounding_boxes.append(bbox_msg)
            else:
                rospy.loginfo("No objects detected.")

            self.pred_pub.publish(bounding_boxes_msg)

            # --- 可视化和发布带标注的图像 ---
            annotated_frame = result.plot(line_width=self.line_thickness)
            
            if self.publish_image:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8"))

            if self.view_image:
                cv2.imshow("YOLO Detection Result", annotated_frame)
                cv2.waitKey(1)
                
            # --- 保存图像 ---
            try:
                save_dir = os.path.dirname(self.save_path)
                if not os.path.exists(save_dir):
                    os.makedirs(save_dir)
                filename = f"{req.detect_flag}.jpg"     #区分是第一处识别还是第二处
                cv2.imwrite(self.save_path+ filename , annotated_frame)
                rospy.loginfo(f"Detection result saved to: {self.save_path}")
            except Exception as e:
                rospy.logerr(f"Failed to save image: {e}")

            # 准备并返回包含计数的服务响应字符串 ===
            # 将字典 {'person': 2, 'car': 1} 转换为 "car:1, person:2"
            response_items = [f"{cls}:{count}" for cls, count in class_counts.items()]
            response_str = ", ".join(sorted(response_items)) #排序以保证输出顺序一致
            
            rospy.loginfo(f"Detection finished. Counts: {response_str if response_str else 'None'}")
            
            return detectResponse(response_str)

if __name__ == "__main__":
    rospy.init_node("yolo_detector_node", anonymous=True)
    try:
        detector = YoloDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 确保程序退出时关闭所有OpenCV窗口
        cv2.destroyAllWindows()