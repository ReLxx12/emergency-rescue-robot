#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import yaml
import numpy as np


def nothing(x):
    pass


yaml_data = yaml.safe_load(open("/home/jetson/rescure_ws/src/mission_params/params/mission.yaml", 'r'))
usb_cam_config = yaml_data["usb_cam"]

cap = cv2.VideoCapture(usb_cam_config["video_port"])
cap.set(cv2.CAP_PROP_FRAME_WIDTH, usb_cam_config["width"])
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, usb_cam_config["height"])
if usb_cam_config["exposure"]:
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # 0~2.6手动曝光  2.6~4自动曝光
    cap.set(cv2.CAP_PROP_EXPOSURE, usb_cam_config["exposure"])
else:
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

cv2.namedWindow("Result")

cv2.createTrackbar("HL", "Result", 0, 255, nothing)
cv2.createTrackbar("SL", "Result", 0, 255, nothing)
cv2.createTrackbar("VL", "Result", 0, 255, nothing)
cv2.createTrackbar("HU", "Result", 255, 255, nothing)
cv2.createTrackbar("SU", "Result", 255, 255, nothing)
cv2.createTrackbar("VU", "Result", 255, 255, nothing)

while True:
    _, frame = cap.read()
    
    frame = cv2.resize(frame, (320, 240))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    l_h = cv2.getTrackbarPos("HL", "Result")
    l_s = cv2.getTrackbarPos("SL", "Result")
    l_v = cv2.getTrackbarPos("VL", "Result")
    u_h = cv2.getTrackbarPos("HU", "Result")
    u_s = cv2.getTrackbarPos("SU", "Result")
    u_v = cv2.getTrackbarPos("VU", "Result")

    thresh_lower = np.array([l_h, l_s, l_v])
    thresh_upper = np.array([u_h, u_s, u_v])
    mask = cv2.inRange(hsv, thresh_lower, thresh_upper)

    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow("Orgin", frame)
    #    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)

    key = cv2.waitKey(150)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
print(f"HSV_thresh_lower [{thresh_lower[0]}, {thresh_lower[1]}, {thresh_lower[2]}]")
print(f"HSV_thresh_upper [{thresh_upper[0]}, {thresh_upper[1]}, {thresh_upper[2]}]")
