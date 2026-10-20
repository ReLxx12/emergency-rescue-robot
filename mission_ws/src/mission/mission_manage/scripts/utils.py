#!/usr/bin/python3
# -*- coding: utf-8 -*-
import cv2
import time
import numpy as np
from collections import Counter
from scipy.spatial.transform import Rotation
from tf.transformations import euler_matrix, quaternion_from_euler, euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose
from arm_motions import grab

import rospy
import tf2_ros
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist, PoseStamped
from user_msgs.msg import CarCmd, ExitCondition


color_info = {
    "red":    ([0, 50, 50], [5, 255, 255], 0),
    "yellow": ([20, 50, 50], [30, 255, 255], 1),
    "blue":   ([90, 50, 50], [120, 255, 255], 2),
    "green":  ([40, 50, 50], [80, 255, 255], 3)
}
red_lower1 = np.array([0, 50, 50])
red_upper1 = np.array([5, 255, 255])
red_lower2 = np.array([175, 50, 50])
red_upper2 = np.array([180, 255, 255])


fruit_dict = {
    "坏柠檬":   0,
    "坏樱桃":   1,
    "生柠檬":   2,
    "生樱桃":   3,
    "熟柠檬":   4,
    "熟樱桃":   5
}
fruit_dict_ = {
    0:          "坏柠檬",
    1:          "坏樱桃",
    2:          "生柠檬",
    3:          "生樱桃",
    4:          "熟柠檬",
    5:          "熟樱桃"
}
color_dict = {
    "红":   0,
    "黄":   1,
    "蓝":   2,
    "绿":   3
}
color_dict_ = {
    0:      "红",
    1:      "黄",
    2:      "蓝",
    3:      "绿"
}

class GrabParam:
    def __init__(self) -> None:
        self.cx = 320
        self.cy = 240
        self.fx = 600
        self.fy = 600
        self.t_cam_2_car = [0, 0, 0]
        self.t_arm_2_car = [0, 0, 0]
        self.rpy_cam_2_car = [0, 0, 0]

        self.l_j5_2_claw = 0.085
        self.theta_j5_2_claw = 0

        self.sight_middle_h = 300
        self.sight_middle_w = [269, 457]
        self.x_obj_to_cam_close = 0.205
        self.x_obj_to_cam_middle = 0.26
        self.x_obj_to_cam_far = 0.34


def chassis_drive_x(dist, car_cmd_pub:rospy.Publisher, cmd_vel_pub:rospy.Publisher):
    """
    底盘行驶

    Args:
        dist:               底盘行驶距离 右手系x
        car_cmd_pub:        底盘任务发布
        cmd_vel_pub:        底盘速度指令发布
    Returns:
        None
    """
    car_cmd = CarCmd()
    cmd_vel = Twist()

    car_cmd.cmd = CarCmd.REMOTE
    for i in range(20):
        car_cmd_pub.publish(car_cmd)
        time.sleep(.01)
    cmd_vel.linear.x = 0.3 * dist / abs(dist)
    cmd_vel_pub.publish(cmd_vel)
    
    time.sleep(abs(dist) / 0.3)

    cmd_vel.linear.x = 0.0
    cmd_vel_pub.publish(cmd_vel)
    car_cmd.cmd = CarCmd.HOLD
    car_cmd_pub.publish(car_cmd)


def chassis_drive_y(dist, car_cmd_pub:rospy.Publisher, cmd_vel_pub:rospy.Publisher):
    """
    底盘行驶

    Args:
        dist:               底盘行驶距离 右手系y
        car_cmd_pub:        底盘任务发布
        cmd_vel_pub:        底盘速度指令发布
    Returns:
        None
    """
    car_cmd = CarCmd()
    cmd_vel = Twist()
    
    car_cmd.cmd = CarCmd.REMOTE
    for i in range(20):
        car_cmd_pub.publish(car_cmd)
        time.sleep(.01)
    cmd_vel.linear.y = 0.3 * dist / abs(dist)
    cmd_vel_pub.publish(cmd_vel)
    
    time.sleep(abs(dist) / 0.3)

    cmd_vel.linear.y = 0.0
    cmd_vel_pub.publish(cmd_vel)
    car_cmd.cmd = CarCmd.HOLD
    car_cmd_pub.publish(car_cmd)


def chassis_turn(ang_deg, car_cmd_pub:rospy.Publisher, cmd_vel_pub:rospy.Publisher):
    """
    底盘转向

    Args:
        ang_deg:            底盘转向角度  z轴右手螺旋定则
        car_cmd_pub:        底盘任务发布
        cmd_vel_pub:        底盘速度指令发布
    Returns:
        None
    """
    car_cmd = CarCmd()
    cmd_vel = Twist()
    print(f"\n开始旋转 {ang_deg}")

    car_cmd.cmd = CarCmd.REMOTE
    for i in range(20):
        car_cmd_pub.publish(car_cmd)
        time.sleep(.01)
    cmd_vel.angular.z = np.radians(60) * ang_deg / abs(ang_deg)
    cmd_vel_pub.publish(cmd_vel)
    
    time.sleep(abs(ang_deg) / 60)

    cmd_vel.angular.z = 0.0
    cmd_vel_pub.publish(cmd_vel)
    car_cmd.cmd = CarCmd.HOLD
    car_cmd_pub.publish(car_cmd)
    print("完成旋转\n")


def sort_basket_color_seq(img, sort_basket_rois):
    img_cpy = img.copy()
    seq_list = []
    for i in range(4):
        x1, y1 = sort_basket_rois[i][0]
        x2, y2 = sort_basket_rois[i][1]
        cv2.rectangle(img_cpy, (x1, y1), (x2, y2), (255, 0, 255), 2)
        cv2.putText(img_cpy, f"{i+1}", (x1, y1), 3, 0.8, (255, 0, 255), 2)
        roi = img[y1:y2, x1:x2]
        if roi.size == 0:
            continue

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
        max_count = 0
        color_name = "unknown"
        color_id_ = -1
        for name, (lower, upper, color_id) in color_info.items():
            if name == "red":
                mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
                mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

            pixel_count = cv2.countNonZero(mask)
            
            if pixel_count > max_count:
                max_count = pixel_count
                color_name = name
                color_id_ = color_id        
        
        seq_list.append(color_id_)
        cv2.putText(img_cpy, f"{color_name}", (x1+15, y1-10), 
                    3, 0.8, (255, 0, 255), 2)
    
    cv2.imshow("basket", img_cpy)
    cv2.waitKey(1)

    return seq_list


def find_most_seq(seq_lists):
    tuple_list = [tuple(sublist) for sublist in seq_lists]
    counter = Counter(tuple_list)
    most_common = counter.most_common(1)[0][0]
    basket_seq = list(most_common)

    return basket_seq


def pub_wp(wp_info,
           car_cmd:CarCmd, at_target:Int16, car_target:PoseStamped, exit_condition:ExitCondition, 
           car_cmd_pub:rospy.Publisher, at_target_pub:rospy.Publisher, exit_condition_pub:rospy.Publisher, car_target_pub:rospy.Publisher):
    
    # wp_info [x, y, yaw, id, exit_l, exit_a, cnt]
    
    car_cmd.cmd = CarCmd.NAVIGATION
    car_cmd_pub.publish(car_cmd)

    at_target.data = wp_info[3]
    at_target_pub.publish(at_target)

    car_target.header.frame_id = "map"
    car_target.pose.position.x = wp_info[0]
    car_target.pose.position.y = wp_info[1]
    q = Rotation.from_euler("xyz", [0, 0, wp_info[2]], degrees=True).as_quat()
    car_target.pose.orientation.x = q[0]
    car_target.pose.orientation.y = q[1]
    car_target.pose.orientation.z = q[2]
    car_target.pose.orientation.w = q[3]

    exit_condition.exit_l = wp_info[4]
    exit_condition.exit_a = wp_info[5]
    exit_condition.arrive_cnt = wp_info[6]

    exit_condition_pub.publish(exit_condition)
    car_target_pub.publish(car_target)


def calc_j5_from_claw(lr:bool, p_pixel:list, err_x:float, g_p:GrabParam):
    """
    运动学逆解服务器不解算执行机构 故需更正机械臂末端位置

    Args:
        lr:                 左/右机械臂  左 True / 右 False
        p_pixel:            像素平面上点的坐标
        err_x:              车体对正底座前后误差
    Returns:
        j5_pose             j5末端在底座坐标系下的位置姿态
    """
    j5_pose = [0, 0, 0, 0, 0, 0]
    x_obj_2_base = 0.26

    # 根据像素位置判断物体距离(因每次停止位置变动很小而可以认为距离为定值)
    if p_pixel[1] <= g_p.sight_middle_h and g_p.sight_middle_w[0] <= p_pixel[0] <= g_p.sight_middle_w[1]:
        x_obj_2_base = g_p.x_obj_to_cam_close + err_x
        # 防止距离过近导致反向抓取
        if x_obj_2_base < 0.15:
            rospy.logwarn(f"计算距离过近({x_obj_2_base:.3f}m), 修正为0.15m以防后抓")
            x_obj_2_base = 0.15
            
        print(f"\n上层中间果到相机CCD距离校正: {g_p.x_obj_to_cam_close} + {err_x} = {x_obj_2_base}")
        if lr:
            j5_pose[4] = -6.1
            j5_pose[5] = -62.6
            print(f"\n使用左臂抓取上层中间果\n")
        else:
            j5_pose[4] = -3.2
            j5_pose[5] = 71
            print(f"\n使用右臂抓取上层中间果\n")
    elif p_pixel[1] <= g_p.sight_middle_h and (p_pixel[0] < g_p.sight_middle_w[0] or p_pixel[0] > g_p.sight_middle_w[1]):
        x_obj_2_base = g_p.x_obj_to_cam_far + err_x
        # 防止距离过近导致反向抓取
        if x_obj_2_base < 0.15:
            rospy.logwarn(f"计算距离过近({x_obj_2_base:.3f}m), 修正为0.15m以防后抓")
            x_obj_2_base = 0.15
            
        print(f"\n上层两侧果到相机CCD距离校正: {g_p.x_obj_to_cam_far} + {err_x} = {x_obj_2_base}")
        if lr:
            j5_pose[4] = -3.5
            j5_pose[5] = -28
            print(f"\n使用左臂抓取上层左侧果\n")
        else:
            j5_pose[4] = -3.5
            j5_pose[5] = 28
            print(f"\n使用右臂抓取上层右侧果\n")
    elif p_pixel[1] > g_p.sight_middle_h and (p_pixel[0] < g_p.sight_middle_w[0] or p_pixel[0] > g_p.sight_middle_w[1]):
        x_obj_2_base = g_p.x_obj_to_cam_middle + err_x
        # 防止距离过近导致反向抓取
        if x_obj_2_base < 0.15:
            rospy.logwarn(f"计算距离过近({x_obj_2_base:.3f}m), 修正为0.15m以防后抓")
            x_obj_2_base = 0.15
            
        print(f"\n下层两侧果到相机CCD距离校正: {g_p.x_obj_to_cam_middle} + {err_x} = {x_obj_2_base}")
        if lr:
            j5_pose[4] = 7.5
            j5_pose[5] = -55
            print(f"\n使用左臂抓取下层左侧果\n")
        else:
            j5_pose[4] = 7.5
            j5_pose[5] = 55
            print(f"\n使用右臂抓取下层右侧果\n")
    else:
        print("\n未定义的水果区间 无法指派距离与抓取姿态\n")
        return None

    # 像素坐标系 -> 相平面坐标系
    x_phase = (p_pixel[0] - g_p.cx) / g_p.fx
    y_phase = (p_pixel[1] - g_p.cy) / g_p.fy
    
    # 相平面坐标系 -> 相机坐标系
    # 车体坐标系中目标的x距离为定值 据此解相机坐标系中的深度
    # 先构建归一化目标点坐标 则有 p_cam = z_cam · p_cam_norm
    p_cam_norm = np.array([x_phase, y_phase, 1]).reshape((3, 1))
    # 计算相机坐标系->车体坐标系的旋转矩阵
    R_cam_2_base = euler_matrix(np.radians(g_p.rpy_cam_2_car[0]), 
                                np.radians(g_p.rpy_cam_2_car[1]), 
                                np.radians(g_p.rpy_cam_2_car[2]), 'sxyz')[:3, :3]
    # p_base = R_cam_2_base · p_cam + t_cam_2_base
    # x_base = r11·z_cam·x_cam_norm + r12·z_cam·y_cam + r13·z_cam + x_cam_2_base
    # z_cam = (x_base - x_cam_2_base) / (r11·x_cam_norm + r12·y_cam_norm + r13)
    denom = R_cam_2_base[0, 0] * p_cam_norm[0] + R_cam_2_base[0, 1] * p_cam_norm[1] + R_cam_2_base[0, 2]
    if abs(denom) < 1e-6:
        print("\n分母接近零 无法求解深度\n")
        return None
    
    z_cam = (x_obj_2_base - g_p.t_cam_2_car[0]) / denom
    p_cam = p_cam_norm * z_cam

    # 相机坐标系 -> 车体坐标系
    p_base = R_cam_2_base.dot(p_cam) + np.array(g_p.t_cam_2_car).reshape((3, 1))
    print("fruit_pos\n", list(p_base))

    # 以上解算到目标在车体坐标系中的位置
    # 考虑到机械臂运动学反解不包含夹爪,只到5号关节末端 以下解算j5末端位姿
    # 夹爪与j5不共轴 存在俯仰夹角
    pitch_claw = j5_pose[4] - g_p.theta_j5_2_claw
    # 依据 夹爪中心-j5相对长度 抓取姿态 更新j5末端位置
    j5_pose[0] = round(float(p_base[0]) - g_p.l_j5_2_claw * np.cos(np.radians(pitch_claw)) * np.cos(np.radians(j5_pose[5])), 3)
    j5_pose[1] = round(float(p_base[1]) - g_p.l_j5_2_claw * np.cos(np.radians(pitch_claw)) * np.sin(np.radians(j5_pose[5])), 3)
    j5_pose[2] = round(float(p_base[2]) + g_p.l_j5_2_claw * np.sin(np.radians(pitch_claw)), 3)

    print("j5_pose\n", j5_pose)

    return j5_pose


def calc_grab(arm_ns, pose, tf_buffer:tf2_ros.Buffer):
    
    # 发布车体坐标系中的j5位姿
    pose_in_car_base = PoseStamped()
    pose_in_car_base.header.frame_id = "car_base_link"
    pose_in_car_base.pose.position.x = pose[0]
    pose_in_car_base.pose.position.y = pose[1]
    pose_in_car_base.pose.position.z = pose[2]
    quaternion = quaternion_from_euler(np.radians(pose[3]), 
                                       np.radians(pose[4]), 
                                       np.radians(pose[5]))
    pose_in_car_base.pose.orientation.x = quaternion[0]
    pose_in_car_base.pose.orientation.y = quaternion[1]
    pose_in_car_base.pose.orientation.z = quaternion[2]
    pose_in_car_base.pose.orientation.w = quaternion[3]

    # 计算j5在机械臂坐标系中的位姿
    transform = tf_buffer.lookup_transform(
        target_frame=f"{arm_ns[1:]}arm_base_link",
        source_frame="car_base_link",
        time=rospy.Time(0),
        timeout=rospy.Duration(1.0)
    )
    pose_in_arm_base = do_transform_pose(pose_in_car_base, transform)

    quat = [pose_in_arm_base.pose.orientation.x, pose_in_arm_base.pose.orientation.y, 
            pose_in_arm_base.pose.orientation.z, pose_in_arm_base.pose.orientation.w]
    euler_angles = euler_from_quaternion(quat)
    print("\nj5末端相对于 arm_base_link 的位姿\n[x, y, z, R, P, Y]  cm °\n%.1f %.1f %.1f %.1f %.1f %.1f" % 
          (pose_in_arm_base.pose.position.x*100, pose_in_arm_base.pose.position.y*100, pose_in_arm_base.pose.position.z*100,
           np.rad2deg(euler_angles[0]), np.rad2deg(euler_angles[1]), np.rad2deg(euler_angles[2])))
    
    return pose_in_arm_base
