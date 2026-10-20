#!/usr/bin/python3
# -*- coding: utf-8 -*-
import time
import rospy
import numpy as np
from sensor_msgs.msg import JointState


def radians_list(degree):
    """
    将list中的角度全部转换为弧度制
    """
    return [np.radians(deg) for deg in degree]


def arm_wait(arm_dir, claw, arm_servo_degree:JointState, arm_servo_degree_pub:rospy.Publisher):
    arm_servo_degree.position = radians_list([arm_dir, 0, 90, 0, 0, claw])
    arm_servo_degree.header.stamp = rospy.Time.now()
    arm_servo_degree_pub.publish(arm_servo_degree)
    time.sleep(.5)


def grab(grab_degree, loose, tight, lr:bool, arm_servo_degree:JointState, arm_servo_degree_pub:rospy.Publisher):
    lr = lr * 2 - 1

    if lr == -1: # 右臂
        arm_servo_degree.position = radians_list([-48, 54, 49, 45, 130, loose])
        arm_servo_degree.header.stamp = rospy.Time.now()
        arm_servo_degree_pub.publish(arm_servo_degree)
        time.sleep(1)

    # Move to the grab position with loose gripper
    arm_servo_degree.position = radians_list([grab_degree[0], grab_degree[1], grab_degree[2], grab_degree[3], grab_degree[4], loose])
    arm_servo_degree.header.stamp = rospy.Time.now()
    arm_servo_degree_pub.publish(arm_servo_degree)
    time.sleep(1)

    # Close the gripper
    arm_servo_degree.position = radians_list([grab_degree[0], grab_degree[1], grab_degree[2], grab_degree[3], grab_degree[4], tight])
    arm_servo_degree.header.stamp = rospy.Time.now()
    arm_servo_degree_pub.publish(arm_servo_degree)
    time.sleep(1)

    # Move to a safe position
    arm_wait(0, tight, arm_servo_degree, arm_servo_degree_pub)
    time.sleep(.5)


def put(arm_front, loose, tight, lr:bool, arm_servo_degree:JointState, arm_servo_degree_pub:rospy.Publisher):
    lr = lr * 2 - 1

    
    L_put_sequence = [        [44, 64, 77, 20, 30, tight],        [78, 76, 66, 32, 89, tight],        [78, 76, 66, 32, 89, loose]    ]
    R_put_sequence = [        [-44, 64, 77, 20, 30, tight],        [-78, 76, 66, 32, 89, tight],        [-78, 76, 66, 32, 89, loose]    ]

    # 执行放置序列
    if lr == 1: # 左臂
        for pose in L_put_sequence:
            arm_servo_degree.position = radians_list(pose)
            arm_servo_degree.header.stamp = rospy.Time.now()
            arm_servo_degree_pub.publish(arm_servo_degree)
            time.sleep(1)
    else: # 右臂
        for pose in R_put_sequence:
            arm_servo_degree.position = radians_list(pose)
            arm_servo_degree.header.stamp = rospy.Time.now()
            arm_servo_degree_pub.publish(arm_servo_degree)
            time.sleep(1)

    # 放置完毕后，按照相反的顺序移动，夹爪保持松开
    if lr == 1: # 左臂
        for pose in reversed(L_put_sequence):
            return_pose = pose[:-1] + [loose] # 保持前5个舵机角度，夹爪松开
            arm_servo_degree.position = radians_list(return_pose)
            arm_servo_degree.header.stamp = rospy.Time.now()
            arm_servo_degree_pub.publish(arm_servo_degree)
            time.sleep(1)
    else: # 右臂
        for pose in reversed(R_put_sequence):
            return_pose = pose[:-1] + [loose] # 保持前5个舵机角度，夹爪松开
            arm_servo_degree.position = radians_list(return_pose)
            arm_servo_degree.header.stamp = rospy.Time.now()
            arm_servo_degree_pub.publish(arm_servo_degree)
            time.sleep(1)

    arm_wait(0, loose, arm_servo_degree, arm_servo_degree_pub)
    time.sleep(.5)
