#!/usr/bin/python3
# -*- coding: utf-8 -*-
import re
import ast
import cv2
import time
import copy
import numpy as np
from scipy.spatial.transform import Rotation
from utils import *
from arm_motions import *

import rospy
from std_msgs.msg import Bool, String, Int16, Float32
from sensor_msgs.msg import CameraInfo, Image, JointState
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
from user_msgs.msg import CarCmd, ExitCondition, ObjDet, ObjDets

# 定义映射字典
hostage_dict = {
    "红":   0,
    "蓝":   1,
    "红色": 0,
    "蓝色": 1,
}
hostage_dict_ = {
    0:          "红",
    1:          "蓝",
}
place_dict = {
    "一号":   0,
    "二号":   1,
    "1号":    0,
    "2号":    1,
}
place_dict_ = {
    0:      "一号",
    1:      "二号"
}

img_raw = None
car_cmd = CarCmd()
arrived = False
cylinder_dets = ObjDets()
left_arm_ik_servo_degree = JointState()
left_arm_get_cmd_ik = False
right_arm_ik_servo_degree = JointState()
right_arm_get_cmd_ik = False

success_rescure_count = 0
current_err_x = 0.0

def image_raw_cb(msg:Image):
    global img_raw
    bridge = CvBridge()
    img_raw = bridge.imgmsg_to_cv2(msg, 'bgr8')

def car_cmd_cb(msg:CarCmd):
    global car_cmd, arrived
    if not arrived and (car_cmd.cmd == CarCmd.NAVIGATION or car_cmd.cmd == CarCmd.HOLD) and msg.cmd == CarCmd.ARRIVE:
        arrived = True
    elif arrived and msg.cmd == CarCmd.NAVIGATION:
        arrived = False    
    car_cmd = msg

def cylinder_dets_cb(msg:ObjDets):
    global cylinder_dets
    cylinder_dets = msg

def left_arm_ik_servo_degree_cb(msg:JointState):
    global left_arm_ik_servo_degree, left_arm_get_cmd_ik
    left_arm_ik_servo_degree = msg
    left_arm_get_cmd_ik = True

def right_arm_ik_servo_degree_cb(msg:JointState):
    global right_arm_ik_servo_degree, right_arm_get_cmd_ik
    right_arm_ik_servo_degree = msg
    right_arm_get_cmd_ik = True

def err_x_cb(msg:Float32):
    global current_err_x
    current_err_x = msg.data

# --------------------------------------------------------------------------------
# 卡死检测与脱困函数
# --------------------------------------------------------------------------------
def wait_and_check_stuck(tf_buffer, cmd_vel_pub, car_cmd_pub, timeout=5.0):
    """
    监测卡死并执行“暂停-后退-恢复”逻辑
    """
    global arrived
    last_pos = None
    last_move_time = time.time()
    stuck_threshold = 0.05 # 5cm 判定阈值
    rate = rospy.Rate(10) 
    
    while not rospy.is_shutdown():
        if arrived:
            return
            
        try:
            trans = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
            current_pos = np.array([trans.transform.translation.x, trans.transform.translation.y])
            
            if last_pos is None:
                last_pos = current_pos
                last_move_time = time.time()
            else:
                dist = np.linalg.norm(current_pos - last_pos)
                
                # 如果移动超过阈值，重置计时器
                if dist > stuck_threshold:
                    last_pos = current_pos
                    last_move_time = time.time()
                else:
                    # 超时未移动，判定为卡死
                    if time.time() - last_move_time > timeout:
                        rospy.logwarn(">>> 检测到卡死！开始执行脱困程序 <<<")
                        
                        # 1. 暂停导航 
                        stop_cmd = CarCmd()
                        stop_cmd.cmd = CarCmd.HOLD 
                        car_cmd_pub.publish(stop_cmd)
                        rospy.sleep(0.2) 
                        
                        # 2. 强制后退 2秒
                        twist = Twist()
                        twist.linear.x = -0.25 
                        for _ in range(20): 
                            cmd_vel_pub.publish(twist)
                            rospy.sleep(0.1)
                            
                        # 3. 停车缓冲
                        twist.linear.x = 0
                        cmd_vel_pub.publish(twist)
                        rospy.sleep(0.5)
                        
                        # 4. 重置状态
                        try:
                            trans_now = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))
                            last_pos = np.array([trans_now.transform.translation.x, trans_now.transform.translation.y])
                        except:
                            pass
                        last_move_time = time.time()
                        
                        # 5. 恢复导航
                        rospy.loginfo(">>> 脱困完成，恢复导航 <<<")
                        resume_cmd = CarCmd()
                        resume_cmd.cmd = CarCmd.NAVIGATION
                        car_cmd_pub.publish(resume_cmd)
                        
        except Exception as e:
            pass
            
        rate.sleep()
# --------------------------------------------------------------------------------


def main():
    global arrived, left_arm_get_cmd_ik, right_arm_get_cmd_ik
    global success_rescure_count
    rospy.init_node('mission_manage_node', anonymous=True)
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    # 获取参数
    go_to_grab_num = rospy.get_param('/mission_magage/go_to_grab/wp_num', 0)
    go_to_grab_wp = [rospy.get_param(f'/mission_magage/go_to_grab/wp_{i+1}') for i in range(go_to_grab_num)]
    transit_wp = rospy.get_param('/mission_magage/transit_pt/wp_1')
    grab_num = rospy.get_param('/mission_magage/grab/wp_num', 0)
    grab_wp = [rospy.get_param(f'/mission_magage/grab/wp_{i+1}') for i in range(grab_num)]
    put_num = rospy.get_param('/mission_magage/put/wp_num', 0)
    put_wp = [rospy.get_param(f'/mission_magage/put/wp_{i+1}') for i in range(put_num)]
    go_back_wp = rospy.get_param('/mission_magage/go_back/wp_1')
    park_num = rospy.get_param('/mission_magage/park/wp_num', 0)
    park_wp = [rospy.get_param(f'/mission_magage/park/wp_{i+1}') for i in range(park_num)]
    
    grab_param_l = GrabParam()
    grab_param_r = GrabParam()

    # 参数加载 (保持原样)
    t_larm_2_car = rospy.get_param('/arm_left/translation', [0, 0, 0])
    t_rarm_2_car = rospy.get_param('/arm_right/translation', [0, 0, 0])
    t_cam_2_car = rospy.get_param('/ascamera/translation', [0, 0, 0])
    rpy_cam_2_car = rospy.get_param('/ascamera/rotation', [-90, 0, -90])
    sight_middle_h = rospy.get_param('/grab_param/sight_middle_h', 300)
    sight_middle_w = rospy.get_param('/grab_param/sight_middle_w', [269, 457])
    x_obj_to_cam_close = rospy.get_param('/grab_param/x_obj_to_cam_close', 0.205)
    x_obj_to_cam_middle = rospy.get_param('/grab_param/x_obj_to_cam_middle', 0.26)
    x_obj_to_cam_far = rospy.get_param('/grab_param/x_obj_to_cam_far', 0.34)
    x_j5_2_claw = rospy.get_param('/grab_param/x_j5_2_claw', 0.085)
    z_j5_2_claw = rospy.get_param('/grab_param/z_j5_2_claw', 0.0115)
    l_j5_2_claw = np.sqrt(x_j5_2_claw**2 + z_j5_2_claw**2)
    theta_j5_2_claw = np.rad2deg(np.arctan2(z_j5_2_claw, x_j5_2_claw))
    left_arm_front = rospy.get_param('turn_front', 90)
    left_arm_middle = rospy.get_param('turn_middle', 0)
    left_arm_back = rospy.get_param('turn_back', -90)
    left_arm_tight = rospy.get_param('/arm_left/claw_tight', 30)
    left_arm_loose = rospy.get_param('/arm_left/claw_loose', 60)
    right_arm_front = rospy.get_param('/arm_right/turn_front', 90)
    right_arm_middle = rospy.get_param('/arm_right/turn_middle', 0)
    right_arm_back = rospy.get_param('/arm_right/turn_back', -90)
    right_arm_tight = rospy.get_param('/arm_right/claw_tight', 30)
    right_arm_loose = rospy.get_param('/arm_right/claw_loose', 60)
    mission_time = rospy.get_param('/mission/mission_time', 300) - 25

    # 订阅与发布
    image_raw_sub = rospy.Subscriber('/ascamera_hp60c/rgb0/image', Image, image_raw_cb)
    car_cmd_sub = rospy.Subscriber('/car_cmd', CarCmd, car_cmd_cb)
    yolov5_dets_sub = rospy.Subscriber('/usb_cam/obj_dets_with_pose', ObjDets, cylinder_dets_cb)
    left_arm_servo_degree_sub = rospy.Subscriber("/arm_left/arm_ik_servo_degree", JointState, left_arm_ik_servo_degree_cb)
    right_arm_servo_degree_sub = rospy.Subscriber("/arm_right/arm_ik_servo_degree", JointState, right_arm_ik_servo_degree_cb)
    err_x_sub = rospy.Subscriber('/wp_nav_pid/err_x', Float32, err_x_cb)

    ds_task_pub = rospy.Publisher('/deepseek/task', Bool, queue_size=1)
    ds_input_pub = rospy.Publisher('/deepseek/input_text', String, queue_size=1)
    at_target_pub = rospy.Publisher('/at_target_id', Int16, queue_size=1)
    car_target_pub = rospy.Publisher('/car_target', PoseStamped, queue_size=1)
    exit_condition_pub = rospy.Publisher('exit_condition', ExitCondition, queue_size=1)
    car_cmd_pub = rospy.Publisher('/car_cmd', CarCmd, queue_size=3)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
    left_arm_target_pose_pub = rospy.Publisher('/arm_left/arm_target', PoseStamped, queue_size=3)
    left_arm_servo_degree_pub = rospy.Publisher('/arm_left/arm_joint_states', JointState, queue_size=3)
    right_arm_target_pose_pub = rospy.Publisher('/arm_right/arm_target', PoseStamped, queue_size=3)
    right_arm_servo_degree_pub = rospy.Publisher('/arm_right/arm_joint_states', JointState, queue_size=3)

    camera_info_msg: CameraInfo = rospy.wait_for_message('/ascamera_hp60c/rgb0/camera_info', CameraInfo, timeout=None)
    for param in [grab_param_l, grab_param_r]:
        param.cx = camera_info_msg.K[2]
        param.cy = camera_info_msg.K[5]
        param.fx = camera_info_msg.K[0]
        param.fy = camera_info_msg.K[4]
        param.t_cam_2_car = t_cam_2_car
        param.rpy_cam_2_car = rpy_cam_2_car
        param.sight_middle_h = sight_middle_h
        param.sight_middle_w = sight_middle_w
        param.x_obj_to_cam_close = x_obj_to_cam_close
        param.x_obj_to_cam_middle = x_obj_to_cam_middle
        param.x_obj_to_cam_far = x_obj_to_cam_far
        param.l_j5_2_claw = l_j5_2_claw
        param.theta_j5_2_claw = theta_j5_2_claw
    grab_param_l.t_arm_2_car = t_larm_2_car
    grab_param_r.t_arm_2_car = t_rarm_2_car
    
    arm_servo_degree = JointState()
    arm_servo_degree.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    # ################################################################################
    # # DeepSeek 任务解析逻辑
    # # ##############################
    ds_input = String()
    input_temp = input("输入任务指令 (例如: 把红色放一号，蓝色放二号)\n>>>")
    
    # 提示词：Few-Shot (少样本) 模式
    prompt = "Input: 把红色的放到一号，蓝色的放到二号。\n" \
             "Output: [['红色', '一号'], ['蓝色', '二号']]\n\n" \
             "Input: 红色去二号，蓝色去一号。\n" \
             "Output: [['红色', '二号'], ['蓝色', '一号']]\n\n" \
             "Input: " + input_temp + "。\n" \
             "Output: " 
             
    ds_input.data = prompt
    ds_input_pub.publish(ds_input)

    t0 = time.time()
    rospy.loginfo("任务开始，等待 DeepSeek 响应...\n")

    ds_msg = rospy.wait_for_message('/deepseek/output_text', String, timeout=None)
    ds_output_raw = ds_msg.data
    rospy.loginfo(f"DeepSeek 原始返回: {ds_output_raw}")

    # 使用正则查找列表格式 (findall 取最后一个)
    matches = re.findall(r'\[\[.*?\]\]', ds_output_raw)
    
    mission = []
    if matches:
        orig_str = matches[-1]
        rospy.loginfo(f"提取到的关键数据: {orig_str}")
        
        # 给中文加上引号
        processed = re.sub(r"(?<=[\s\[,])([^\d\s\[\],']{1,5})(?=[\s\],])", r"'\1'", orig_str)
        
        try:
            temp_mission = ast.literal_eval(processed)
            # 防复读检查
            if len(temp_mission) > 0 and len(temp_mission[0]) > 0:
                s_check = str(temp_mission[0])
                if "颜色" in s_check or "地点" in s_check:
                    raise ValueError("Model repeated template text")
            
            mission = temp_mission
            print(f"解析成功: {mission}")
            
        except Exception as e:
            rospy.logerr(f"解析失败: {e}，转为暴力匹配")
            mission = [] 
    else:
        rospy.logerr("未找到列表数据，转为暴力匹配")

    # --- 保底逻辑 ---
    if not mission:
        rospy.logwarn(f"启用关键词暴力匹配模式 (基于输入: {input_temp})")
        if "红" in input_temp and ("一" in input_temp or "1" in input_temp): mission.append([0, 0])
        if "红" in input_temp and ("二" in input_temp or "2" in input_temp): mission.append([0, 1])
        if "蓝" in input_temp and ("一" in input_temp or "1" in input_temp): mission.append([1, 0])
        if "蓝" in input_temp and ("二" in input_temp or "2" in input_temp): mission.append([1, 1])
        print(f"保底解析结果: {mission}")

    # 任务列表最终整理
    grab_mission = mission
    mission_tasks = []
    for item in grab_mission:
        if isinstance(item, list) and len(item) == 2:
            color_code = item[0]
            place_code = item[1]
            
            if isinstance(color_code, int): pass
            elif isinstance(color_code, str):
                if "红" in color_code: color_code = 0
                elif "蓝" in color_code: color_code = 1
                else: continue
            else: continue

            if isinstance(place_code, int): pass
            elif isinstance(place_code, str):
                if "一" in place_code or "1" in place_code: place_code = 0
                elif "二" in place_code or "2" in place_code: place_code = 1
                else: continue
            else: continue
            
            mission_tasks.append({
                'color': color_code,    # 0:红, 1:蓝
                'place': place_code,    # 0:一号, 1:二号
                'status': 'pending',    # pending, captured, completed
                'arm': None             # 0:right, 1:left
            })

    print("救援放置任务列表：", mission_tasks)
    ds_task_pub.publish(Bool(False))

    # --------------------------------------------------------------------------------
    # 任务执行逻辑
    # --------------------------------------------------------------------------------

    # 【新增】追踪每个救援点位的尝试次数 (最多尝试3次)
    MAX_ATTEMPTS = 3
    # grab_wp_attempts = {0: 0, 1: 0, ...}
    grab_wp_attempts = {i: 0 for i in range(grab_num)} 
    
    chassis_turn(180, car_cmd_pub, cmd_vel_pub)
    at_target = Int16()
    car_target = PoseStamped()
    exit_condition = ExitCondition()

    print("\n离开起始区...\n")
    for i in range(go_to_grab_num):
        arrived = False
        pub_wp(go_to_grab_wp[i], car_cmd, at_target, car_target, exit_condition, car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
        wait_and_check_stuck(tf_buffer, cmd_vel_pub, car_cmd_pub)

    chassis_turn(180, car_cmd_pub, cmd_vel_pub)
    print("\n前往中间点...\n")
    arrived = False
    pub_wp(transit_wp, car_cmd, at_target, car_target, exit_condition, car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
    wait_and_check_stuck(tf_buffer, cmd_vel_pub, car_cmd_pub)
    
    # 救援循环现在需要找到下一个待尝试的救援点
    current_grab_wp_index = 0
    
    while not rospy.is_shutdown() and (time.time() - t0 < mission_time):
        
        # 1. 检查是否所有任务完成
        if all(t['status'] == 'completed' for t in mission_tasks):
            print("所有任务已完成")
            break

        # 2. 检查手臂是否已满，如果满了则先去放置
        captured_tasks = [t for t in mission_tasks if t['status'] == 'captured']
        if len(captured_tasks) >= 2: 
            print("双手已满，前往放置")
            pass # 跳到放置阶段

        else: # 手臂未满，继续寻找待抓取点

            # --- 抓取阶段：寻找下一个救援点 ---
            
            # 找到下一个可以尝试的救援点位索引 (循环遍历)
            next_grab_wp_index = -1
            start_index = current_grab_wp_index
            
            # 从当前点开始，循环遍历所有点，直到找到一个：
            # 1. 尝试次数 < MAX_ATTEMPTS
            # 2. 且该点位上至少有一个 pending 任务（虽然现在无法判断，但先去看看）
            # 简化逻辑：只要尝试次数没用完，就去。如果去到发现没任务了，下次直接跳过。
            
            for i in range(grab_num):
                test_index = (start_index + i) % grab_num
                if grab_wp_attempts[test_index] < MAX_ATTEMPTS:
                    next_grab_wp_index = test_index
                    break
            
            if next_grab_wp_index == -1:
                # 遍历完所有点，要么都抓完了，要么都失败了3次
                print("所有救援点都已尝试或达到最大次数。")
                
                # 如果仍有 pending 任务，但手臂已满，跳到放置
                if any(t['status'] == 'pending' for t in mission_tasks) and len(captured_tasks) >= 1:
                     print("有未完成任务，但手臂有货，先去放置。")
                else:
                    # 彻底没救了，退出大循环
                    break
            
            else: # 找到下一个救援点 next_grab_wp_index
                
                grab_wp_index = next_grab_wp_index
                
                # 记录抓取前手臂上任务的数量
                captured_before = len(captured_tasks)

                # --- 前往救援位置 ---
                print(f"\n前往{grab_wp_index+1}号救援位置 (尝试次数: {grab_wp_attempts[grab_wp_index] + 1})...\n")
                arrived = False
                pub_wp(grab_wp[grab_wp_index], car_cmd, at_target, car_target, exit_condition, car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
                wait_and_check_stuck(tf_buffer, cmd_vel_pub, car_cmd_pub)

                print(f"\n到达 {grab_wp_index+1}号 救援位置 开始检测\n")
                time.sleep(1)
                
                # --- 检测与抓取逻辑 ---
                t1 = time.time()
                cylinder_dets_ = None
                while not rospy.is_shutdown() and cylinder_dets_ is None:
                    if time.time() - t1 > 2:
                        rospy.logwarn("超时")
                        break
                    if not cylinder_dets.num:
                        rospy.Rate(1).sleep()
                        continue
                    cylinder_dets_ = copy.copy(cylinder_dets)

                if cylinder_dets_ is None:
                    print(f"\n{grab_wp_index+1}号 救援位置未检测到物体\n")
                
                # 实际执行抓取
                else:
                    print("\n检测到物体 准备匹配任务\n")
                    right_arm_busy = any(t['status'] == 'captured' and t['arm'] == 0 for t in mission_tasks)
                    left_arm_busy = any(t['status'] == 'captured' and t['arm'] == 1 for t in mission_tasks)

                    for det in cylinder_dets_.dets:
                        det: ObjDet
                        matched_task = next((t for t in mission_tasks if t['status'] == 'pending' and t['color'] == det.class_id), None)
                        if matched_task is None: continue
                        
                        target_center_pix = [det.bbox.x, det.bbox.y]
                        
                        # 尝试左臂
                        if target_center_pix[0] < sight_middle_w[1] and not left_arm_busy:
                            print(f"尝试左臂抓取 {hostage_dict_.get(matched_task['color'])}")
                            pose = calc_j5_from_claw(True, target_center_pix, current_err_x, grab_param_l)
                            if pose is not None:
                                pose_in_left_arm_base = calc_grab("/arm_left/", pose, tf_buffer)
                                left_arm_target_pose_pub.publish(pose_in_left_arm_base)
                                t2 = time.time()
                                while not left_arm_get_cmd_ik and (time.time() - t2 < 2): pass
                                if left_arm_get_cmd_ik:
                                    grab_degree = left_arm_ik_servo_degree.position
                                    left_arm_get_cmd_ik = False
                                    grab(grab_degree, left_arm_loose, left_arm_tight, True, arm_servo_degree, left_arm_servo_degree_pub)
                                    matched_task['status'] = 'captured'
                                    matched_task['arm'] = 1 
                                    left_arm_busy = True
                                    print(f"左臂抓取成功")
                                    continue 
                        
                        # 尝试右臂
                        if target_center_pix[0] >= sight_middle_w[0] and not right_arm_busy:
                            print(f"尝试右臂抓取 {hostage_dict_.get(matched_task['color'])}")
                            pose = calc_j5_from_claw(False, target_center_pix, current_err_x, grab_param_r)
                            if pose is not None:
                                pose_in_right_arm_base = calc_grab("/arm_right/", pose, tf_buffer)
                                right_arm_target_pose_pub.publish(pose_in_right_arm_base)
                                t2 = time.time()
                                while not right_arm_get_cmd_ik and (time.time() - t2 < 2): pass
                                if right_arm_get_cmd_ik:
                                    grab_degree = right_arm_ik_servo_degree.position
                                    right_arm_get_cmd_ik = False
                                    grab(grab_degree, right_arm_loose, right_arm_tight, False, arm_servo_degree, right_arm_servo_degree_pub)
                                    matched_task['status'] = 'captured'
                                    matched_task['arm'] = 0 
                                    right_arm_busy = True
                                    print(f"右臂抓取成功")
                                    continue

                # --- 尝试计数更新与下一个点位设置 ---
                captured_after = len([t for t in mission_tasks if t['status'] == 'captured'])

                if captured_after > captured_before:
                    # 成功抓取，重置该点位计数器，并转到下一个点位
                    grab_wp_attempts[grab_wp_index] = 0
                    current_grab_wp_index = (grab_wp_index + 1) % grab_num
                else:
                    # 抓取失败 (可能是没检测到，或 IK/抓取失败)
                    grab_wp_attempts[grab_wp_index] += 1
                    print(f"点位 {grab_wp_index+1} 抓取失败，尝试次数: {grab_wp_attempts[grab_wp_index]}")
                    
                    if grab_wp_attempts[grab_wp_index] >= MAX_ATTEMPTS:
                        print(f"点位 {grab_wp_index+1} 达到最大尝试次数，永久跳过此点位。")
                    
                    # 转到下一个点位进行尝试
                    current_grab_wp_index = (grab_wp_index + 1) % grab_num


        # 3. --- 放置阶段 ---
        
        # 重新检查是否有货物需要放置
        captured_tasks = [t for t in mission_tasks if t['status'] == 'captured']
        if not captured_tasks:
            continue # 没有抓到任何东西，继续抓取循环

        print("\n准备前往放置，先去中间点...\n")
        arrived = False
        pub_wp(transit_wp, car_cmd, at_target, car_target, exit_condition, car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
        wait_and_check_stuck(tf_buffer, cmd_vel_pub, car_cmd_pub)

        captured_tasks.sort(key=lambda x: x['place'])
        for task in captured_tasks:
            place = task['place']
            arm = task['arm']
            print(f"\n前往 {place_dict_.get(place)} 放置位...\n")
            arrived = False
            pub_wp(put_wp[place], car_cmd, at_target, car_target, exit_condition, car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
            wait_and_check_stuck(tf_buffer, cmd_vel_pub, car_cmd_pub)
            time.sleep(1)
            print(f"{'左' if arm==1 else '右'}臂 将 {hostage_dict_.get(task['color'])} 放置到 {place_dict_.get(place)}")
            if arm == 1: 
                put(left_arm_front, left_arm_loose, left_arm_tight, True, arm_servo_degree, left_arm_servo_degree_pub)
            else: 
                put(right_arm_front, right_arm_loose, right_arm_tight, False, arm_servo_degree, right_arm_servo_degree_pub)
            task['status'] = 'completed'
            success_rescure_count += 1
            print("\n前往中间点...\n")
            arrived = False
            pub_wp(transit_wp, car_cmd, at_target, car_target, exit_condition, car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
            wait_and_check_stuck(tf_buffer, cmd_vel_pub, car_cmd_pub)

        # 放置完成，回到下一个救援点索引（如果已经遍历完了，下次循环会重新开始找）
        # 索引 current_grab_wp_index 在抓取失败时已经被更新为下一个点位
        

    # 4. --- 任务结束，返回 ---
    print("\n前往中间点...\n")
    arrived = False
    pub_wp(transit_wp, car_cmd, at_target, car_target, exit_condition, car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
    wait_and_check_stuck(tf_buffer, cmd_vel_pub, car_cmd_pub)

    print("\n返回\n")
    arrived = False
    pub_wp(go_back_wp, car_cmd, at_target, car_target, exit_condition, car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
    wait_and_check_stuck(tf_buffer, cmd_vel_pub, car_cmd_pub)

    chassis_turn(180, car_cmd_pub, cmd_vel_pub)
    print(f"\n停入车位\n")
    arrived = False
    pub_wp(park_wp[0], car_cmd, at_target, car_target, exit_condition, car_cmd_pub, at_target_pub, exit_condition_pub, car_target_pub)
    wait_and_check_stuck(tf_buffer, cmd_vel_pub, car_cmd_pub)

if __name__ == "__main__":
    main()