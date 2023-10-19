#!/usr/bin/env python3

import cv2
import torch
import glob
import os
import sys
from numpy import asarray
import PIL.Image as Image
import pandas
import csv
from ultralytics import YOLO

import time
import math
import pathlib
import datetime

import rospy

# callback function to show the image using opencv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

get_path = pathlib.Path.cwd()
date = datetime.datetime.now().strftime("%Y%m%d")

classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]
model = YOLO("YoloWeights/yolov8n.pt")

# to get the parameter:
# yolo task=detect mode=train model=yolov8n.pt data=AVITAGS_NAVLAB20230930-1/data.yaml epochs=30 imgsz=640
# classNames = ["Wildfire Spot"]
# parameter_path = 'yolo-Weights/'
# parameter = '20231016best.pt'
# model = YOLO(parameter_path + parameter)

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
# params for recording the yolo processed video
frame_size = (640, 480)  # width, height
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
# print("[INFO]", date)
save_path = 'home/nav/dev/datasets/videos/20231016/'


def callback(image):
    try:
        frame = CvBridge().imgmsg_to_cv2(image, "bgr8")
        cv2.imshow("original", frame)
        cv2.waitKey(1)
        # results = model(frame, stream=True)
        # for result in results:
        #     boxes = result.boxes
        #     for box in boxes:
        #         x1, y1, x2, y2 = box.xyxy[0]
        #         x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        #         cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 1)
        #         print(f"======> The Coordinate x_min y_min x_max y_x=max\n", x1, y1, x2, y2)
        #
        #         # confidence
        #         confidence = math.ceil((box.conf[0] * 100)) / 100
        #         print("======> Confidence", confidence)
        #
        #         # class_name
        #         cls = int(box.cls[0])
        #         print("======> Class Name", classNames[cls])
        #
        #         # object detials
        #         org = [x1, y1]
        #         font = cv2.FONT_HERSHEY_SIMPLEX
        #         fontScale = 1
        #         color = (255, 0, 0)
        #         thickness = 1
        #
        #         cv2.putText(frame, classNames[cls], org, font, fontScale, color, thickness)
        #         cv2.imshow('processed', frame)
    except CvBridgeError as e:
        print(e)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/dji_osdk_ros/main_wide_RGB", Image, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
    cv2.destroyAllWindows()
