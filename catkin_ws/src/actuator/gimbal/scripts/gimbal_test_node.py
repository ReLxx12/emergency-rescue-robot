#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-
import cv2
import numpy as np

import rospy
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from user_msgs.msg import GimbalPoseDeg


frame = None
bridge = CvBridge()


def nothing(x):
    pass


def image_raw_cb(msg:Image):
    global frame
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')


def main():
    try:
        rospy.init_node('gimbal_test_node', anonymous=True)
    except rospy.ROSInterruptException:
        rospy.logerr("云台测试节点初始化失败...")
        exit(0)
    
    global frame
    
    gimbal_pose = GimbalPoseDeg()
    gimbal_pose.roll = 0
    gimbal_pose.pitch = 0
    gimbal_pose.yaw = 0

    image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_raw_cb)
    frame = bridge.imgmsg_to_cv2(rospy.wait_for_message('/usb_cam/image_raw', Image, timeout=None), 'bgr8')
    (h, w, _) = frame.shape
    img_center = (w//2, h//2)

    task_pub = rospy.Publisher("/task", Bool, queue_size=1) 
    gimbal_pose_ctrl_pub = rospy.Publisher("gimbal_pose_ctrl", GimbalPoseDeg, queue_size=10) 

    cv2.imshow('Servo Degree', frame)

    cv2.createTrackbar("Pitch", "Servo Degree", 900, 1800, nothing)
    cv2.createTrackbar("Yaw", "Servo Degree", 900, 1800, nothing)
    cv2.createTrackbar("Reset", "Servo Degree", 0, 1, nothing)
    
    last_pitch = cv2.getTrackbarPos("Pitch", "Servo Degree")
    last_yaw = cv2.getTrackbarPos("Yaw", "Servo Degree")

    gimbal_pose_ctrl_pub.publish(gimbal_pose)
    
    while 1:
        # frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

        pitch = cv2.getTrackbarPos("Pitch", "Servo Degree")
        yaw = cv2.getTrackbarPos("Yaw", "Servo Degree")
        rst = cv2.getTrackbarPos("Reset", "Servo Degree")

        cv2.line(frame, (0, img_center[1]), (w, img_center[1]), (0, 255, 0), 2)
        cv2.line(frame, (img_center[0], 0), (img_center[0], h), (0, 255, 0), 2)
        cv2.putText(frame, f'Pitch {(900-pitch)/10}', (10, 30), 3, 1, (0, 0, 255), 2)
        cv2.putText(frame, f'Yaw   {(900-yaw)/10}', (10, 70), 3, 1, (0, 0, 255), 2)
        cv2.imshow('Servo Degree', frame)
        key = cv2.waitKey(1) & 0xff

        if not rst:
            if pitch != last_pitch or yaw != last_yaw:
                gimbal_pose.pitch = (900-pitch)/10
                gimbal_pose.yaw = (900-yaw)/10
                gimbal_pose_ctrl_pub.publish(gimbal_pose)
                last_pitch = pitch
                last_yaw = yaw
        else:
            gimbal_pose.pitch = 0
            gimbal_pose.yaw = 0
            gimbal_pose_ctrl_pub.publish(gimbal_pose)
            cv2.setTrackbarPos("Pitch", "Servo Degree", 900)
            cv2.setTrackbarPos("Yaw", "Servo Degree", 900)
            cv2.setTrackbarPos("Reset", "Servo Degree", 0)

        if key == 27:
            task_pub.publish(Bool(False))
            break
    
if __name__ == '__main__':
    main()
    
