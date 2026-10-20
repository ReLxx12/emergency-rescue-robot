#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-
import cv2
import numpy as np

import rospy
from std_msgs.msg import Empty, Bool
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from user_msgs.msg import CannonStatus


frame = None
bridge = CvBridge()
bullet_num = 0  # 剩余弹药数量
rdy2fire = True  # 是否待发


def nothing(x):
    pass


def calculate_scope_scale(z_cannon_to_cam, a_cannon_to_cam, camera_matrix, distortion_coefficients, x_cannon_to_cam, img_center, scope_rate):
    scope_scale = [[], []]
    
    last_h = [2*img_center[1], (scope_rate+1)*img_center[1]]
    
    for i in range(10, 100+1, 10):
        h_0 = camera_matrix[1][2] + (z_cannon_to_cam-i*np.tan(a_cannon_to_cam)) * camera_matrix[1][1] / i
        
        undistort_point = np.array([[[img_center[0]+x_cannon_to_cam, h_0]]], dtype=np.float64)
        distorted_points_norm = cv2.undistortPoints(undistort_point, camera_matrix, distortion_coefficients)
        distorted_points_homogeneous = np.concatenate([distorted_points_norm, np.ones((distorted_points_norm.shape[0], 1, 1))], axis=2)
        distorted_points = np.dot(camera_matrix, distorted_points_homogeneous.reshape(3, 1)).T
        distorted_points = distorted_points[:, :2]
        h_0 = int(distorted_points[0, 1])
        
        h_1 = int(img_center[1] + (h_0 - img_center[1]) * scope_rate)

        if last_h[0] - h_0 >= 2:
            scope_scale[0].append([i, h_0])
        last_h[0] = h_0
        # if last_h[1] - h_1 >= 3:
        scope_scale[1].append([i, h_1])
        last_h[1] = h_1
        
    return scope_scale


def grid_scope(img, scope_mask,
               center, img_size,  
               full_screen, use_scope, zoom, scope_radius, scope_rate, scope_scale,
               cross_color, aim_color):
    if use_scope:
        cv2.line(img, (0, center[1]), (img_size[0], center[1]), cross_color, 1)
        cv2.line(img, (center[0], 0), (center[0], img_size[1]), cross_color, 1)
        scope_mask_ = scope_mask.copy()
        if zoom:
            cv2.circle(scope_mask_, (center[0], center[1]), scope_radius*scope_rate, (255, 255, 255), -1)
        else:
            cv2.circle(scope_mask_, (center[0], center[1]), scope_radius, (255, 255, 255), -1)
        
        for num, scale_pair in enumerate(scope_scale[int(zoom)]):
            if num & 1:
                cv2.line(img, (center[0], scale_pair[1]), (center[0]+10+num*13, scale_pair[1]), cross_color, 1)
                cv2.putText(img, str(scale_pair[0]), (center[0]+num*13+10, scale_pair[1]+5), 3, 0.5, cross_color, 1)
            else:
                cv2.line(img, (center[0]-10-num*13, scale_pair[1]), (center[0], scale_pair[1]), cross_color, 1)
                cv2.putText(img, str(scale_pair[0]), (center[0]-num*13-30, scale_pair[1]+5), 3, 0.5, cross_color, 1)
        img = cv2.bitwise_and(img, scope_mask_)
    # 视野中心
    cv2.circle(img, center, 3+(-2*int(full_screen)), aim_color, -1)
    return img 


def image_raw_cb(msg:Image):
    global frame
    frame = bridge.imgmsg_to_cv2(msg, 'bgr8')


def cannon_status_cb(msg:CannonStatus):
    global bullet_num, rdy2fire
    bullet_num = msg.bullet_num
    rdy2fire = msg.rdy2fire


def main():
    try:
        rospy.init_node('cannon_calib_node', anonymous=True)
    except rospy.ROSInterruptException:
        rospy.logerr("激光校准节点初始化失败...")
        exit(0)

    global frame

    serial_port = rospy.get_param('/cannon/serial_port', '/dev/CH340')
    serial_baudrate = rospy.get_param('/cannon/serial_baudrate', 115200)
    irradiation_time = rospy.get_param('/cannon/irradiation_time', 1)
    x_cannon_to_cam = rospy.get_param('cannon/x_cannon_to_cam', 0)  # 激光光点在像素平面上到视野中心的像素距离 pix
    z_cannon_to_cam = rospy.get_param('cannon/z_cannon_to_cam', 3)  # 激光光路到相机光轴的距离 cm
    a_cannon_to_cam = rospy.get_param('cannon/a_cannon_to_cam', 0)  # 激光光路到相机光轴的夹角 °

    image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_raw_cb)
    cannon_statue_sub = rospy.Subscriber('cannon_status', CannonStatus, cannon_status_cb)
    frame = bridge.imgmsg_to_cv2(rospy.wait_for_message('/usb_cam/image_raw', Image, timeout=None), 'bgr8')
    camera_info_msg = rospy.wait_for_message('/usb_cam/camera_info', CameraInfo, timeout=None)
    camera_matrix = np.array([[camera_info_msg.K[0], 0.0, camera_info_msg.K[2]],  # [fx, 0, cx]
                              [0.0, camera_info_msg.K[4], camera_info_msg.K[5]],  # [0, fy, cy]
                              [0.0, 0.0, 1.0]],
                             dtype=np.float64)
    distortion_coefficients = np.array([camera_info_msg.D[0], camera_info_msg.D[1], camera_info_msg.D[2], camera_info_msg.D[3], camera_info_msg.D[4]],
                                       dtype=np.float64)
    (h, w, c) = frame.shape
    img_size_src = (w, h)
    img_center = (w//2, h//2)

    zoom = False
    use_scope = False
    scope_rate = 3
    # 瞄具蒙版
    scope_mask = np.zeros((h, w, c), np.uint8)
    
    task_pub = rospy.Publisher("/task", Bool, queue_size=1) 
    trigger_pub = rospy.Publisher("/trigger", Empty, queue_size=1)

    cv2.imshow('Cannon Calib', frame)

    cv2.createTrackbar("x_cannon_to_cam", "Cannon Calib", int(100+x_cannon_to_cam), 200, nothing)
    cv2.createTrackbar("z_cannon_to_cam", "Cannon Calib", int(100+z_cannon_to_cam*20), 200, nothing)
    cv2.createTrackbar("a_cannon_to_cam", "Cannon Calib", int(100+a_cannon_to_cam*20), 200, nothing)
    
    r = rospy.Rate(30)

    while 1:
        
        # 获取相机画面
        img_frame_src = frame.copy()
        
        x_cannon_to_cam = int(cv2.getTrackbarPos("x_cannon_to_cam", "Cannon Calib") - 100)
        z_cannon_to_cam = (cv2.getTrackbarPos("z_cannon_to_cam", "Cannon Calib") - 100) / 20
        a_cannon_to_cam = (cv2.getTrackbarPos("a_cannon_to_cam", "Cannon Calib") - 100) / 20

        scope_scale = calculate_scope_scale(z_cannon_to_cam, np.radians(a_cannon_to_cam), 
                                            camera_matrix, distortion_coefficients, 
                                            x_cannon_to_cam, img_center, scope_rate)

        # 依据激光光路水平偏移量调整画面水平中位
        if x_cannon_to_cam > 0:
            img = np.zeros_like(img_frame_src, np.uint8)
            img[:, 0:img_size_src[0]-x_cannon_to_cam] = img_frame_src[:, x_cannon_to_cam:img_size_src[0]]
        elif x_cannon_to_cam < 0:
            img = np.zeros_like(img_frame_src, np.uint8)
            img[:, -x_cannon_to_cam:img_size_src[0]] = img_frame_src[:, 0:img_size_src[0]+x_cannon_to_cam]
        else:
            img = img_frame_src.copy()
            
        # 先图像放大 减小全屏放大的运算量
        if zoom:
            # 根据放大比例 先扣出区域 再放大 减小全图放大的运算量
            img_zoom = img[int(img_center[1]-img_size_src[1]/scope_rate/2):int(img_center[1]+img_size_src[1]/scope_rate/2),
                           int(img_center[0]-img_size_src[0]/scope_rate/2):int(img_center[0]+img_size_src[0]/scope_rate/2)]
            img = cv2.resize(img_zoom, img_size_src)
        
        img = grid_scope(img, scope_mask,
                         img_center, img_size_src,  
                         False, use_scope, zoom, 200, scope_rate, scope_scale,
                         [0, 0, 255], [255, 150, 0])
        
        cv2.putText(img, f"Ammo:{bullet_num}", (20, 30), 3, 0.8, (255, 0, 255), 2)
        cv2.putText(img, f"Rdy", (150, 30), 3, 0.8, (0, 255, 0) if rdy2fire else (0, 0, 255), 2)
        cv2.putText(img, f"x:{x_cannon_to_cam}", (20, 60), 3, 0.8, (255, 0, 255), 2)
        cv2.putText(img, f"z:{z_cannon_to_cam}", (20, 90), 3, 0.8, (255, 0, 255), 2)
        cv2.putText(img, f"a:{a_cannon_to_cam}", (20, 120), 3, 0.8, (255, 0, 255), 2)
        cv2.imshow('Cannon Calib', img)
        key = cv2.waitKey(1) & 0xff

        if key == 27:
            task_pub.publish(Bool(False))
            print("\n\n\n\n\n将下列参数复制至 cannon_config.yaml \n\ncannon:")
            print("  serial_port:         ", serial_port)
            print("  serial_baudrate:     ", serial_baudrate)
            print("  irradiation_time:    ", irradiation_time)
            print("  # 以下参数 须手动调节相机球方向 确保激光光路与相机光轴平行", )
            print("  x_cannon_to_cam:     ", x_cannon_to_cam)
            print("  z_cannon_to_cam:     ", z_cannon_to_cam)
            print("  a_cannon_to_cam:     ", a_cannon_to_cam, "\n\n\n\n\n")
            break
        elif key == ord(' '):
            trigger_pub.publish(Empty())
        elif key == 225:
            use_scope = not use_scope
        elif key in [ord('z'), ord('Z')]:
            zoom = not zoom

        r.sleep()   
    
if __name__ == '__main__':
    main()
    
