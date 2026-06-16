#!/usr/bin/env python3

import rospy
import cv2
import base64
import requests
import threading
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from robot_navigation.srv import *
from PIL import Image as PILImage, ImageDraw, ImageFont 
# 百度API配置
API_KEY = "35g5qewe6ZLy0WRQyOmMs4Q0"
SECRET_KEY = "tOnoRh5lotAH09TK617C54BK16S0KFl4"
ACCESS_TOKEN_URL = "https://aip.baidubce.com/oauth/2.0/token"
PLATE_API_URL = "https://aip.baidubce.com/rest/2.0/ocr/v1/license_plate"

# --- 中文绘制配置 ---
# !!! 关键：请确认这个路径在你系统上真实存在，并且是一个支持中文的字体文件 (.ttf 或 .ttc) !!!
# 如果你不确定，可以尝试使用 /usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf
FONT_PATH = "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc" 
FONT_SIZE = 30 

class PlateRecognizer:
    def __init__(self):
        rospy.init_node('plate_recognition_service')
        
        # ROS服务
        self.service = rospy.Service('/recognize_plate', detect, self.handle_recognition)
        
        # 图像订阅和发布
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_callback)
        # 结果图像发布
        self.result_pub = rospy.Publisher("/recognized_image", Image, queue_size=1) 
        self.save_path = '/home/lbw/ws_com/src/robot_navigation/scripts/pic/plate.jpg'
        # 获取访问令牌
        self.access_token = self.get_access_token()
        if not self.access_token:
            rospy.logerr("Failed to obtain Baidu API access token. Check your API credentials.")
            rospy.signal_shutdown("API authentication failed")
            
        rospy.loginfo("Plate recognition service is ready")

    def image_callback(self, msg):
        """存储最新的图像"""
        try:
            # 确保图像是BGR格式，适合OpenCV处理
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") 
            with self.image_lock:
                self.latest_image = cv_image.copy() # 使用copy确保线程安全
        except Exception as e:
            rospy.logerr(f"Image conversion error: {str(e)}")

    def get_access_token(self):
        """获取百度API访问令牌 (保持不变)"""
        params = {
            "grant_type": "client_credentials",
            "client_id": API_KEY,
            "client_secret": SECRET_KEY
        }
        try:
            response = requests.post(ACCESS_TOKEN_URL, params=params)
            response.raise_for_status()
            return response.json().get("access_token")
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"Access token request failed: {str(e)}")
            return None

    def recognize_plate(self, cv_image):
        """
        修改：使用百度API识别车牌。
        现在返回完整的API响应JSON，以便提取坐标。
        """
        if cv_image is None:
            return None
            
        # 将OpenCV图像转换为JPEG格式的base64
        _, buffer = cv2.imencode('.jpg', cv_image)
        img_base64 = base64.b64encode(buffer).decode('utf-8')
        
        # 准备API请求
        payload = {
            'image': img_base64,
            'access_token': self.access_token
        }
        headers = {'Content-Type': 'application/x-www-form-urlencoded'}
        
        try:
            response = requests.post(PLATE_API_URL, headers=headers, data=payload)
            response.raise_for_status()
            # 返回完整的JSON结果
            return response.json()
        except Exception as e:
            rospy.logerr(f"Plate recognition API error: {str(e)}")
            return None

    def draw_plate_info(self, image, result_json):
        """
        在图像上绘制车牌号和锚框，使用 PIL 解决中文绘制问题。
        """
        draw_image = image.copy()
        
        if 'words_result' not in result_json:
            return draw_image, "RECOGNITION_FAILED_NO_WORDS"

        words_result = result_json['words_result']
        plate_number = words_result.get('number', 'N/A')
        location = words_result.get('vertexes_location', None)

        if location and len(location) == 4:
            # 1. 绘制锚框（使用 OpenCV）
            points = np.array([[p['x'], p['y']] for p in location], np.int32)
            cv2.polylines(draw_image, [points], isClosed=True, color=(0, 255, 0), thickness=2)
            
            # 确定文本绘制位置
            x, y = location[0]['x'], location[0]['y']
            text_y = max(20, y - 10) 
                
            # 转换 OpenCV BGR 图像到 PIL RGB 图像
            pil_image = PILImage.fromarray(cv2.cvtColor(draw_image, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(pil_image)
            
            try:
                # 加载中文字体
                font = ImageFont.truetype(FONT_PATH, FONT_SIZE)
            except IOError:
                rospy.logerr(f"Could not load font: {FONT_PATH}. Check font path! Using default font.")
                font = ImageFont.load_default() # 如果加载失败，使用默认字体 (可能仍无法显示中文)
            
            # 绘制文本
            # PIL使用RGB：(255, 255, 0) 是黄色
            draw.text((x, text_y), plate_number, font=font, fill=(255, 255, 0))
            
            # 转换回 OpenCV BGR 格式
            draw_image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
            
            rospy.loginfo(f"Recognized plate: {plate_number}. Bounding box drawn.")
        else:
            rospy.logwarn("Plate number recognized but vertexes_location not found or invalid.")

        return draw_image, plate_number

    def handle_recognition(self, req):
        """处理服务请求"""
        if req.detect_flag==3:
            rospy.loginfo("I GOT Request:%d,Starting car plate detection",req.detect_flag)
            # 获取最新图像
            with self.image_lock:
                current_image = self.latest_image.copy() # 再次copy，确保在处理过程中不被订阅回调修改
                
            # 检查是否有可用图像
            if current_image is None:
                rospy.logwarn("No image available for recognition")
                return detectResponse("ERROR: No image available")
                
            # 1. 执行车牌识别，获取完整JSON结果
            result_json = self.recognize_plate(current_image)
            
            if result_json:
                # 2. 在图像上绘制识别信息
                result_image, plate_number = self.draw_plate_info(current_image, result_json)

                # 3. 保存绘制结果的图像
                cv2.imwrite(self.save_path, result_image)
                rospy.loginfo(f"Successfully saved image to: {self.save_path}")
                    
                # 4. 返回车牌号给服务调用方
                if plate_number and plate_number != 'N/A':
                    return detectResponse(plate_number)
                else:
                    return detectResponse("RECOGNITION_FAILED")

            else:
                rospy.logwarn("Plate recognition failed (API returned no valid result)")
                return detectResponse("RECOGNITION_FAILED")

if __name__ == '__main__':
    try:
        recognizer = PlateRecognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass