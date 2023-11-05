#!/usr/bin/env python3

import cv2

# check python version
import sys

print("Python version: " + sys.version)

import torch
import glob
import os
from numpy import asarray
import PIL.Image as Image

# qiao: using yolov5, no need for ultralytics
# import pandas
# import csv
# from ultralytics import YOLO

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

# classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
#               "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
#               "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
#               "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
#               "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
#               "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
#               "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
#               "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
#               "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
#               "teddy bear", "hair drier", "toothbrush"
#               ]
# model = YOLO("YoloWeights/yolov8n.pt")

# to get the parameter:
# yolo task=detect mode=train model=yolov8n.pt data=AVITAGS_NAVLAB20230930-1/data.yaml epochs=30 imgsz=640
classNames = ["Wildfire Spot"]

weight_path = "/home/qin/m300_ws/src/forest_fire_geopositioning/scripts/YoloWeights/v5l.pt"
model = torch.hub.load('ultralytics/yolov5', 'custom',
                       weight_path, force_reload = True)
# model = torch.hub.load('ultralytics/yolov5', 'yolov5l', force_reload=True)
# model = YOLO("/home/qin/m300_ws/src/forest_fire_geopositioning/scripts/YoloWeights/best.pt")
# model = YOLO("/home/qin/m300_ws/src/forest_fire_fighting/scripts/YoloWeights/yolov8n.pt")

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
# params for recording the yolo processed video
# frame_size = (640, 480)  # width, height
# fourcc = cv2.VideoWriter_fourcc(*"mp4v")
# print("[INFO]", date)
# save_path = 'home/nav/dev/datasets/videos/20231016/'

# import vision_msgs
from vision_msgs.msg import Detection2D, Detection2DArray


def callback(image, pub):
    try:
        frame = CvBridge().imgmsg_to_cv2(image, "bgr8")
        # convert color
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # cv2.imshow("original", frame)
        # cv2.waitKey(1)
        results = model(frame)
        ros_boxes = Detection2DArray()
        ros_boxes.header = image.header

        # ###########################################################
        # qiao: changing yolov8 bbboxes into yolov5
        # bboxes
        # for result in results:
        #     boxes = result.boxes
        #     for box in boxes:
        #         x1, y1, x2, y2 = box.xyxy[0]
        #         x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        #         cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 1)
        for box in results.xyxy[0]:
            x1 = int(box[0])
            x2 = int(box[2])
            y1 = int(box[1])
            y2 = int(box[3])

            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
            cv2.imshow('processed', frame)
            cv2.waitKey(1)

            # add results to the Detection2DArray
            # qiao20231031: maybe need modification
            ros_box = Detection2D()
            ros_box.header = image.header
            ros_box.bbox.size_x = x2 - x1
            ros_box.bbox.size_y = y2 - y1
            ros_box.bbox.center.x = (x1 + x2) / 2
            ros_box.bbox.center.y = (y1 + y2) / 2
            ros_box.bbox.center.theta = 0
            ros_boxes.detections.append(ros_box)
            pub.publish(ros_boxes)
            # print how many boxes are detected
        print("======> Number of boxes detected: ", len(ros_boxes.detections))
    except CvBridgeError as e:
        print(e)


def listener():
    rospy.init_node('listener', anonymous=True)
    # publish the detection bounding boxes using jsk_recognition_msgs/Detection2DArray
    pub = rospy.Publisher('/bounding_boxes/fire_spots', Detection2DArray, queue_size=10)
    # rospy.Subscriber("/dji_osdk_ros/main_wide_RGB", Image, callback, pub)
    rospy.Subscriber("/dji_osdk_ros/main_wide_RGB", Image, callback, pub)
    rospy.spin()


if __name__ == '__main__':
    listener()
    cv2.destroyAllWindows()
