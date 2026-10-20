#!/usr/bin/python3
# -*- coding: utf-8 -*-
import cv2
import time
import copy
import numpy as np
import threading
import queue
from utils import *

import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Bool, String
from sensor_msgs.msg import CameraInfo, Image, JointState
from user_msgs.msg import ObjDet, ObjDets
from geometry_msgs.msg import PoseStamped

img_raw = None
cylinder_dets = None   # 圆柱体检测结果
arm_ik_servo_degree = JointState()
get_cmd_ik = False
img_lock = threading.Lock()
det_lock = threading.Lock()

display_queue = queue.Queue(maxsize=1)
click_queue = queue.Queue()
exit_event = threading.Event()


def image_raw_cb(msg: Image):
    global img_raw
    bridge = CvBridge()
    with img_lock:
        img_raw = bridge.imgmsg_to_cv2(msg, 'bgr8')


def cylinder_dets_cb(msg: ObjDets):
    global cylinder_dets
    with det_lock:
        cylinder_dets = msg


def arm_ik_servo_degree_cb(msg:JointState):
    global arm_ik_servo_degree, get_cmd_ik
    arm_ik_servo_degree = msg
    get_cmd_ik = True


def draw_detections(img):
    img_copy = img.copy()
    with det_lock:
        if cylinder_dets is not None:
            for det in cylinder_dets.dets:
                det: ObjDet
                tl = (int(det.bbox.x1), int(det.bbox.y1))
                br = (int(det.bbox.x2), int(det.bbox.y2))
                color = (0, 0, 255) if det.label == "red_cylinder" else (255, 0, 0)
                cv2.rectangle(img_copy, tl, br, color, 2)
                cv2.putText(img_copy, det.label, (tl[0], tl[1]-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return img_copy


def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        click_queue.put((x, y))


def t_display():
    cv2.namedWindow('Camera')
    cv2.setMouseCallback('Camera', mouse_callback)
    
    while not exit_event.is_set():
        try:
            current_img = None
            with img_lock:
                if img_raw is not None:
                    current_img = img_raw.copy()
            
            if current_img is None:
                time.sleep(0.01)
                continue
            
            img_with_dets = draw_detections(current_img)
            
            cv2.imshow('Camera', img_with_dets)
            
            key = cv2.waitKey(1) & 0xff
            if key == 27:
                exit_event.set()
                break
                
        except Exception as e:
            rospy.logerr(f"Display error: {str(e)}")
            exit_event.set()
            break
    
    cv2.destroyAllWindows()


def main():
    global img_raw, cylinder_dets, arm_ik_servo_degree, get_cmd_ik
    
    rospy.init_node('grab_cylinder_node', anonymous=True)

    arm_ns = rospy.get_namespace()
    arm_dir = arm_ns[5:-1]

    if arm_dir in ['l', 'L', 'left', 'Left', 'LEFT']:
        lr = True
    elif arm_dir in ['r', 'R', 'right', 'Right', 'RIGHT']:
        lr = False
    else:
        lr = True
    
    grab_param = GrabParam()
    camera_info_msg: CameraInfo = rospy.wait_for_message('/ascamera_hp60c/rgb0/camera_info', CameraInfo, timeout=None)
    grab_param.cx = camera_info_msg.K[2]
    grab_param.cy = camera_info_msg.K[5]
    grab_param.fx = camera_info_msg.K[0]
    grab_param.fy = camera_info_msg.K[4]
    grab_param.t_arm_2_car = rospy.get_param('translation', [0, 0, 0])
    grab_param.t_cam_2_car = rospy.get_param('/ascamera/translation', [0, 0, 0])
    grab_param.rpy_cam_2_car = rospy.get_param('/ascamera/rotation', [-90, 0, -90])
    grab_param.sight_middle_h = rospy.get_param('/grab_param/sight_middle_h', 300)
    grab_param.sight_middle_w = rospy.get_param('/grab_param/sight_middle_w', [269, 457])
    grab_param.x_obj_to_cam_close = rospy.get_param('/grab_param/x_obj_to_cam_close', 0.205)
    grab_param.x_obj_to_cam_middle = rospy.get_param('/grab_param/x_obj_to_cam_middle', 0.26)
    grab_param.x_obj_to_cam_far = rospy.get_param('/grab_param/x_obj_to_cam_far', 0.34)
    x_j5_2_claw = rospy.get_param('/grab_param/x_j5_2_claw', 0.085)
    z_j5_2_claw = rospy.get_param('/grab_param/z_j5_2_claw', 0.0115)
    grab_param.l_j5_2_claw = np.sqrt(x_j5_2_claw**2 + z_j5_2_claw**2)
    grab_param.theta_j5_2_claw = np.rad2deg(np.arctan2(z_j5_2_claw, x_j5_2_claw))
    tight = rospy.get_param('claw_tight', 0)
    loose = rospy.get_param('claw_loose', 70)

    arm_servo_degree = JointState()
    arm_servo_degree.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    # 订阅图像和圆柱体检测结果
    image_raw_sub = rospy.Subscriber('/ascamera_hp60c/rgb0/image', Image, image_raw_cb)
    cylinder_dets_sub = rospy.Subscriber('/usb_cam/obj_dets_with_pose', ObjDets, cylinder_dets_cb)
    arm_servo_degree_sub = rospy.Subscriber("arm_ik_servo_degree", JointState, arm_ik_servo_degree_cb)

    arm_target_pose_pub = rospy.Publisher('arm_target', PoseStamped, queue_size=3)
    arm_servo_degree_pub = rospy.Publisher('arm_joint_states', JointState, queue_size=3)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    display_thread = threading.Thread(target=t_display, daemon=True)
    display_thread.start()
    
    while not exit_event.is_set() and not rospy.is_shutdown():
        try:
            if not click_queue.empty():
                x, y = click_queue.get(timeout=0.1)
                
                with det_lock:
                    if cylinder_dets is None:
                        continue
                    else:
                        dets_copy: ObjDets = copy.copy(cylinder_dets)

                center_pix = [x, y]
                selected_det = None
                for det in dets_copy.dets:
                    det: ObjDet
                    if (det.bbox.x1 <= x <= det.bbox.x2) and (det.bbox.y1 <= y <= det.bbox.y2):
                        center_pix = [det.bbox.x, det.bbox.y]
                        selected_det = det
                        break
                
                if selected_det is None:
                    print("未点击到任何圆柱体")
                    continue

                print(f"\n处理圆柱体: {selected_det.label} 中心像素 ({center_pix[0]}, {center_pix[1]})\n")
                if lr:  # 左臂
                    if center_pix[0] < grab_param.sight_middle_w[1]:  # 左半边
                        pass
                    else:
                        print("左臂无法抓取右侧圆柱体")
                        continue
                else:  # 右臂
                    if center_pix[0] > grab_param.sight_middle_w[0]:  # 右半边
                        pass
                    else:
                        print("右臂无法抓取左侧圆柱体")
                        continue

                # 计算抓取位姿（使用像素中心 + 假设深度）
                pose = calc_j5_from_claw(lr, center_pix, 0, grab_param)
                pose_in_arm_base = calc_grab(arm_ns, pose, tf_buffer)
                arm_target_pose_pub.publish(pose_in_arm_base)
                
                t0 = time.time()
                while not get_cmd_ik and (time.time() - t0 < 2):
                    pass
                
                if get_cmd_ik:
                    grab_degree = arm_ik_servo_degree.position
                    get_cmd_ik = False
                    print(f"开始抓取 {selected_det.label}\n")
                    grab(grab_degree, loose, tight, lr, arm_servo_degree, arm_servo_degree_pub)
                
            else:
                time.sleep(0.01)
                
        except queue.Empty:
            continue
        except Exception as e:
            rospy.logerr(f"Main thread error: {str(e)}")
            exit_event.set()
    
    exit_event.set()
    display_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
