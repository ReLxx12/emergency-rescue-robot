#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from user_msgs.msg import ObjDets, ObjDet, ObjBbox
from geometry_msgs.msg import Pose, Point, Quaternion

bridge = CvBridge()

# 全局变量
enable_detection = True
img_raw = None
camera_matrix = None

# HSV颜色阈值
color_ranges = {
    "red": [
        ([0, 50, 50], [5, 255, 255]),
        ([175, 50, 50], [180, 255, 255])
    ],
    "blue": [
        ([90, 50, 50], [120, 255, 255])
    ]
}

# 滑动条参数
erode_iterations = 1
dilate_iterations = 1
min_area = 500
min_circularity = 0.6
aspect_ratio_min = 0.3
aspect_ratio_max = 3.0

win_name = "Cylinder Detection Params"

def nothing(x):
    pass

def create_trackbars():
    cv2.namedWindow(win_name)
    cv2.createTrackbar("erode_iterations", win_name, erode_iterations, 5, nothing)
    cv2.createTrackbar("dilate_iterations", win_name, dilate_iterations, 5, nothing)
    cv2.createTrackbar("min_area", win_name, min_area, 5000, nothing)
    cv2.createTrackbar("min_circularity", win_name, int(min_circularity*100), 100, nothing)
    cv2.createTrackbar("aspect_ratio_min", win_name, int(aspect_ratio_min*100), 200, nothing)
    cv2.createTrackbar("aspect_ratio_max", win_name, int(aspect_ratio_max*100), 500, nothing)

def enable_cylindar_det_cb(msg):
    global enable_detection
    enable_detection = msg.data

def image_raw_cb(msg):
    global img_raw
    img_raw = bridge.imgmsg_to_cv2(msg, "bgr8")

def detect_cylinders(img, color):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = np.zeros_like(hsv[:, :, 0])
    for (lower, upper) in color_ranges[color]:
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)
        mask |= cv2.inRange(hsv, lower, upper)

    kernel = np.ones((3, 3), np.uint8)
    mask_erode = cv2.erode(mask, kernel, iterations=erode_iterations)
    mask_morph = cv2.dilate(mask_erode, kernel, iterations=dilate_iterations)

    contours, _ = cv2.findContours(mask_morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    dets = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        perimeter = cv2.arcLength(cnt, True)
        circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0

        x, y, w, h = cv2.boundingRect(cnt)
        aspect_ratio = float(w) / h

        if circularity > min_circularity or (aspect_ratio >= aspect_ratio_min and aspect_ratio <= aspect_ratio_max):
            cx = x + w // 2
            cy = y + h // 2
            dets.append((cx, cy, w, h, area, circularity))

    # 返回中间过程图像
    return dets, hsv, mask, mask_erode, mask_morph

def pixel_to_3d(u, v, z=0.5):
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    X = (u - cx) * z / fx
    Y = (v - cy) * z / fy
    return (X, Y, z)

def main():
    global erode_iterations, dilate_iterations, min_area, min_circularity, aspect_ratio_min, aspect_ratio_max
    rospy.init_node("cylinder_detector")

    rospy.Subscriber('/enable_target_detect', Bool, enable_cylindar_det_cb)
    rospy.Subscriber('/ascamera_hp60c/rgb0/image', Image, image_raw_cb)

    cam_info = rospy.wait_for_message('/ascamera_hp60c/rgb0/camera_info', CameraInfo)
    global camera_matrix
    camera_matrix = np.array([[cam_info.K[0], 0, cam_info.K[2]],
                              [0, cam_info.K[4], cam_info.K[5]],
                              [0, 0, 1]])

    obj_pub = rospy.Publisher('/usb_cam/obj_dets_with_pose', ObjDets, queue_size=3)

    create_trackbars()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if img_raw is None or not enable_detection:
            rate.sleep()
            continue

        # 读取滑动条参数
        erode_iterations = cv2.getTrackbarPos("erode_iterations", win_name)
        dilate_iterations = cv2.getTrackbarPos("dilate_iterations", win_name)
        min_area = cv2.getTrackbarPos("min_area", win_name)
        min_circularity = cv2.getTrackbarPos("min_circularity", win_name) / 100.0
        aspect_ratio_min = cv2.getTrackbarPos("aspect_ratio_min", win_name) / 100.0
        aspect_ratio_max = cv2.getTrackbarPos("aspect_ratio_max", win_name) / 100.0

        # 检测（获取中间过程图像）
        red_dets, hsv, mask_red, mask_red_erode, mask_red_morph = detect_cylinders(img_raw, "red")
        blue_dets, _, mask_blue, mask_blue_erode, mask_blue_morph = detect_cylinders(img_raw, "blue")

        # 发布消息
        obj_msg = ObjDets()
        obj_msg.header.stamp = rospy.Time.now()
        obj_msg.header.frame_id = "camera_link"
        obj_msg.fps = 30
        dets_list = []

        for idx, (cx, cy, w, h, area, circularity) in enumerate(red_dets):
            det = ObjDet()
            det.index = idx
            det.class_id = 0
            det.label = "red_cylinder"
            det.score = circularity

            bbox = ObjBbox()
            bbox.x1 = cx - w//2
            bbox.y1 = cy - h//2
            bbox.x2 = cx + w//2
            bbox.y2 = cy + h//2
            bbox.x = cx
            bbox.y = cy
            bbox.w = w
            bbox.h = h
            det.bbox = bbox

            X, Y, Z = pixel_to_3d(cx, cy, z=0.22)
            det.pose = Pose(Point(X, Y, Z), Quaternion(0, 0, 0, 1))
            dets_list.append(det)

        for idx, (cx, cy, w, h, area, circularity) in enumerate(blue_dets):
            det = ObjDet()
            det.index = idx + len(red_dets)
            det.class_id = 1
            det.label = "blue_cylinder"
            det.score = circularity

            bbox = ObjBbox()
            bbox.x1 = cx - w//2
            bbox.y1 = cy - h//2
            bbox.x2 = cx + w//2
            bbox.y2 = cy + h//2
            bbox.x = cx
            bbox.y = cy
            bbox.w = w
            bbox.h = h
            det.bbox = bbox

            X, Y, Z = pixel_to_3d(cx, cy, z=0.22)
            det.pose = Pose(Point(X, Y, Z), Quaternion(0, 0, 0, 1))
            dets_list.append(det)

        obj_msg.num = len(dets_list)
        obj_msg.dets = dets_list
        obj_pub.publish(obj_msg)

        # 可视化最终结果
        vis_img = img_raw.copy()
        for (cx, cy, w, h, _, _) in red_dets:
            cv2.rectangle(vis_img, (cx - w//2, cy - h//2), (cx + w//2, cy + h//2), (0, 0, 255), 2)
            cv2.circle(vis_img, (cx, cy), 3, (0, 0, 255), -1)
        for (cx, cy, w, h, _, _) in blue_dets:
            cv2.rectangle(vis_img, (cx - w//2, cy - h//2), (cx + w//2, cy + h//2), (255, 0, 0), 2)
            cv2.circle(vis_img, (cx, cy), 3, (255, 0, 0), -1)

#        # 显示中间过程
#        # 将mask转为3通道，方便拼接显示
#        mask_red_3ch = cv2.cvtColor(mask_red, cv2.COLOR_GRAY2BGR)
#        mask_red_morph_3ch = cv2.cvtColor(mask_red_morph, cv2.COLOR_GRAY2BGR)
#        mask_blue_3ch = cv2.cvtColor(mask_blue, cv2.COLOR_GRAY2BGR)
#        mask_blue_morph_3ch = cv2.cvtColor(mask_blue_morph, cv2.COLOR_GRAY2BGR)

#        # 拼接图像
#        top_row = np.hstack((img_raw, hsv))
#        mid_row1 = np.hstack((mask_red_3ch, mask_red_morph_3ch))
#        mid_row2 = np.hstack((mask_blue_3ch, mask_blue_morph_3ch))
#        bottom_row = np.hstack((vis_img, np.zeros_like(vis_img))) # 占位

#        # 整体拼接
#        process_img = np.vstack((top_row, mid_row1, mid_row2, bottom_row))

#        cv2.imshow("Process", process_img)
        cv2.imshow("Cylinder Detection", vis_img)

        # 检测按键
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            print("\n===== 调参完成，当前参数 =====")
            params = {
                "erode_iterations": erode_iterations,
                "dilate_iterations": dilate_iterations,
                "min_area": min_area,
                "min_circularity": min_circularity,
                "aspect_ratio_min": aspect_ratio_min,
                "aspect_ratio_max": aspect_ratio_max
            }
            print(params)
            cv2.destroyAllWindows()
            rospy.signal_shutdown("用户按ESC退出")
            break

        rate.sleep()

if __name__ == "__main__":
    main()
