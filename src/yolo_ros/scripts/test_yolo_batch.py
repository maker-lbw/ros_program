#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
YOLO 批量测试脚本（基于功能包路径）
- 自动定位功能包根目录（假设脚本位于 yolo_ros/scripts/ 下）
- 模型、输入图片、输出目录均相对于功能包根目录
- 无需命令行参数，直接 rosrun 运行
"""

import os
import sys
from ultralytics import YOLO
import cv2

# ---------- 自动定位功能包根目录 ----------
# 获取当前脚本所在目录（假设为 .../yolo_ros/scripts/）
script_dir = os.path.dirname(os.path.abspath(__file__))
# 功能包根目录（scripts 的父目录）
pkg_dir = os.path.dirname(script_dir)

# ---------- 定义路径（全部相对于 pkg_dir） ----------
MODEL_PATH = os.path.join(pkg_dir, "weights", "best.pt")    # 模型文件
INPUT_DIR  = os.path.join(pkg_dir, "picture")              # 输入图片文件夹
OUTPUT_DIR = os.path.join(pkg_dir, "results")              # 输出标注图文件夹

# 类别名称（必须与训练时 classes.txt 顺序一致）
CLASS_NAMES = ["enemy", "friend_army", "hostage"]
# -------------------------------------------------


def process_image(model, image_path, output_dir, class_names):
    """推理单张图片，保存标注图，返回各类统计"""
    results = model(image_path)[0]
    counts = {name: 0 for name in class_names}
    for cls_id in results.boxes.cls.int().tolist():
        if cls_id < len(class_names):
            counts[class_names[cls_id]] += 1
        else:
            print(f"  警告: 未知类别 id {cls_id}")

    annotated = results.plot()
    out_name = os.path.basename(image_path)
    out_path = os.path.join(output_dir, out_name)
    cv2.imwrite(out_path, annotated)
    return counts, out_path


def main():
    # 检查必要路径
    if not os.path.exists(MODEL_PATH):
        print(f"错误: 模型文件不存在: {MODEL_PATH}")
        sys.exit(1)
    if not os.path.exists(INPUT_DIR):
        print(f"错误: 图片目录不存在: {INPUT_DIR}")
        sys.exit(1)

    # 加载模型
    print(f"加载模型: {MODEL_PATH}")
    model = YOLO(MODEL_PATH)

    # 获取图片列表（支持常见扩展名）
    exts = {'.jpg', '.jpeg', '.png', '.bmp', '.tif', '.tiff'}
    image_list = [
        os.path.join(INPUT_DIR, f) for f in os.listdir(INPUT_DIR)
        if os.path.splitext(f)[1].lower() in exts
    ]
    if not image_list:
        print(f"在 {INPUT_DIR} 中未找到任何图片")
        return

    # 创建输出目录
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    print(f"共找到 {len(image_list)} 张图片\n")
    for img_path in image_list:
        print(f"处理: {img_path}")
        counts, saved_path = process_image(model, img_path, OUTPUT_DIR, CLASS_NAMES)
        print(f"  统计: enemy={counts['enemy']}, friend_army={counts['friend_army']}, hostage={counts['hostage']}")
        print(f"  标注图保存至: {saved_path}\n")

    print("全部完成！")


if __name__ == "__main__":
    main()