#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from PIL import Image as PILImage, ImageDraw, ImageFont
from sensor_msgs.msg import Image

from robot_navigation.srv import detect, detectResponse

FONT_PATH = "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc"
FONT_SIZE = 30
DEFAULT_SAVE_PATH = "/home/lbw/ws_com/src/robot_navigation/scripts/pic/plate.jpg"


class PlateRecognizer:
    def __init__(self):
        rospy.init_node('plate_recognition_service')

        self.service = rospy.Service('/recognize_plate', detect, self.handle_recognition)
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
        self.result_pub = rospy.Publisher('/recognized_image', Image, queue_size=1)
        self.save_path = rospy.get_param('~save_path', DEFAULT_SAVE_PATH)

        self.det_model_dir = rospy.get_param('~det_model_dir', '')
        self.rec_model_dir = rospy.get_param('~rec_model_dir', '')
        self.cls_model_dir = rospy.get_param('~cls_model_dir', '')
        self.use_angle_cls = rospy.get_param('~use_angle_cls', False)
        self.paddle_lang = rospy.get_param('~paddle_lang', 'ch')
        self.ocr = None
        self.ocr_checked = False

        rospy.loginfo('车牌识别服务已启动，识别内核为 PaddleOCR')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.image_lock:
                self.latest_image = cv_image.copy()
        except Exception as exc:
            rospy.logerr('图像转换失败: %s', exc)

    def get_ocr(self):
        if self.ocr_checked:
            return self.ocr

        self.ocr_checked = True
        try:
            from paddleocr import PaddleOCR

            kwargs = {
                'use_angle_cls': self.use_angle_cls,
                'lang': self.paddle_lang,
                'show_log': False,
            }
            if self.det_model_dir:
                kwargs['det_model_dir'] = self.det_model_dir
            if self.rec_model_dir:
                kwargs['rec_model_dir'] = self.rec_model_dir
            if self.cls_model_dir:
                kwargs['cls_model_dir'] = self.cls_model_dir

            self.ocr = PaddleOCR(**kwargs)
        except Exception as exc:
            rospy.logerr('初始化 PaddleOCR 失败: %s', exc)
            self.ocr = None

        return self.ocr

    def extract_plate_roi(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        color_masks = [
            cv2.inRange(hsv, np.array([90, 70, 70]), np.array([140, 255, 255])),
            cv2.inRange(hsv, np.array([35, 40, 40]), np.array([95, 255, 255])),
            cv2.inRange(hsv, np.array([10, 60, 60]), np.array([40, 255, 255])),
        ]
        color_mask = color_masks[0]
        for extra_mask in color_masks[1:]:
            color_mask = cv2.bitwise_or(color_mask, extra_mask)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, kernel, iterations=1)

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 80, 200)
        merged_mask = cv2.bitwise_or(color_mask, edges)
        merged_mask = cv2.morphologyEx(merged_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(merged_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_rect = None
        best_score = -1.0
        image_area = float(cv_image.shape[0] * cv_image.shape[1])

        for contour in sorted(contours, key=cv2.contourArea, reverse=True)[:40]:
            x, y, w, h = cv2.boundingRect(contour)
            area = float(w * h)
            if area < image_area * 0.005 or area > image_area * 0.25:
                continue

            aspect_ratio = float(w) / float(max(h, 1))
            if aspect_ratio < 2.2 or aspect_ratio > 6.5:
                continue

            roi_mask = color_mask[y:y + h, x:x + w]
            color_ratio = float(cv2.countNonZero(roi_mask)) / float(max(w * h, 1))
            if color_ratio < 0.08:
                continue

            roi_edges = edges[y:y + h, x:x + w]
            edge_ratio = float(cv2.countNonZero(roi_edges)) / float(max(w * h, 1))
            score = area * (1.0 + color_ratio * 4.0 + edge_ratio * 2.0)
            if score > best_score:
                best_score = score
                best_rect = (x, y, w, h)

        if best_rect is None:
            return cv_image.copy(), None

        x, y, w, h = best_rect
        pad_x = int(w * 0.10)
        pad_y = int(h * 0.25)
        x0 = max(0, x - pad_x)
        y0 = max(0, y - pad_y)
        x1 = min(cv_image.shape[1], x + w + pad_x)
        y1 = min(cv_image.shape[0], y + h + pad_y)
        return cv_image[y0:y1, x0:x1].copy(), (x0, y0, x1, y1)

    def normalize_plate_text(self, text):
        if not text:
            return ''

        text = str(text).upper().replace(' ', '')
        return re.sub(r'[^0-9A-Z\u4e00-\u9fff]', '', text)

    def recognize_plate(self, cv_image):
        if cv_image is None:
            return None

        ocr = self.get_ocr()
        if ocr is None:
            return None

        plate_roi, bbox = self.extract_plate_roi(cv_image)
        if plate_roi is None or plate_roi.size == 0:
            return None

        try:
            results = ocr.ocr(plate_roi, cls=self.use_angle_cls)
        except Exception as exc:
            rospy.logerr('PaddleOCR 识别失败: %s', exc)
            return None

        best_text = ''
        best_score = -1.0
        for line_group in results or []:
            if not line_group:
                continue
            for line in line_group:
                if not line or len(line) < 2:
                    continue
                text_info = line[1]
                if not isinstance(text_info, (list, tuple)) or len(text_info) < 2:
                    continue
                text = self.normalize_plate_text(text_info[0])
                score = float(text_info[1])
                if text and score > best_score:
                    best_text = text
                    best_score = score

        if not best_text:
            return None

        return {
            'number': best_text,
            'bbox': bbox,
        }

    def draw_plate_info(self, image, result_dict):
        draw_image = image.copy()
        plate_number = result_dict.get('number', 'N/A')
        bbox = result_dict.get('bbox')

        if bbox is not None:
            x0, y0, x1, y1 = bbox
            cv2.rectangle(draw_image, (x0, y0), (x1, y1), (0, 255, 0), 2)
            text_x = x0
            text_y = max(20, y0 - 10)
        else:
            text_x = 10
            text_y = 30

        pil_image = PILImage.fromarray(cv2.cvtColor(draw_image, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(pil_image)
        try:
            font = ImageFont.truetype(FONT_PATH, FONT_SIZE)
        except IOError:
            rospy.logwarn('中文字体加载失败，使用默认字体')
            font = ImageFont.load_default()

        draw.text((text_x, text_y), plate_number, font=font, fill=(255, 255, 0))
        draw_image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
        return draw_image, plate_number

    def handle_recognition(self, req):
        if req.detect_flag != 3:
            return detectResponse('UNSUPPORTED_FLAG')

        rospy.loginfo('收到车牌识别请求，开始使用 PaddleOCR 识别')
        with self.image_lock:
            current_image = None if self.latest_image is None else self.latest_image.copy()

        if current_image is None:
            rospy.logwarn('当前没有可用图像')
            return detectResponse('ERROR: No image available')

        result_dict = self.recognize_plate(current_image)
        if result_dict is None:
            rospy.logwarn('车牌识别失败')
            return detectResponse('RECOGNITION_FAILED')

        result_image, plate_number = self.draw_plate_info(current_image, result_dict)

        save_dir = os.path.dirname(self.save_path)
        if save_dir and not os.path.exists(save_dir):
            os.makedirs(save_dir)
        cv2.imwrite(self.save_path, result_image)

        try:
            self.result_pub.publish(self.bridge.cv2_to_imgmsg(result_image, 'bgr8'))
        except Exception as exc:
            rospy.logwarn('发布车牌识别结果图失败: %s', exc)

        rospy.loginfo('车牌识别完成，结果: %s，图片已保存到: %s', plate_number, self.save_path)
        return detectResponse(plate_number)


if __name__ == '__main__':
    try:
        recognizer = PlateRecognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
