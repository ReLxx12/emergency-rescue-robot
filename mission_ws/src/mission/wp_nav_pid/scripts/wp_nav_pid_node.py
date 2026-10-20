#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-
import time
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from pid import PID_incremental

import rospy
from std_msgs.msg import Bool
from user_msgs.msg import CarCmd, PID, ExitCondition
from geometry_msgs.msg import Twist, PoseStamped


class WaypointNavigationPIDNode(object):
    def __init__(self) -> None:
        
        rospy.loginfo(f"航点导航初始化中...\n")

        self.kp_x = rospy.get_param('/wp_nav/pid_x/kp', 0)
        self.ki_x = rospy.get_param('/wp_nav/pid_x/ki', 0)
        self.kd_x = rospy.get_param('/wp_nav/pid_x/kd', 0)
        i_a_x = rospy.get_param('/wp_nav/pid_x/i_a', 0)
        o_t_x = rospy.get_param('/wp_navpid_x//o_t', 0.2)
        self.kp_y = rospy.get_param('/wp_nav/pid_y/kp', 0)
        self.ki_y = rospy.get_param('/wp_nav/pid_y/ki', 0)
        self.kd_y = rospy.get_param('/wp_nav/pid_y/kd', 0)
        i_a_y = rospy.get_param('/wp_nav/pid_y/i_a', 0)
        o_t_y = rospy.get_param('/wp_nav/pid_y/o_t', 0.2)
        self.kp_yaw = rospy.get_param('/wp_nav/pid_yaw/kp', 0)
        self.ki_yaw = rospy.get_param('/wp_nav/pid_yaw/ki', 0)
        self.kd_yaw = rospy.get_param('/wp_nav/pid_yaw/kd', 0)
        i_a_yaw = rospy.get_param('/wp_nav/pid_yaw/i_a', 0)
        o_t_yaw = rospy.get_param('/wp_nav/pid_yaw/o_t', 0.2)

        self.exit_l = rospy.get_param('/wp_nav/exit_l', 0.01)
        self.exit_a = rospy.get_param('/wp_nav/exit_a', 1)
        self.arrive_cnt_num = rospy.get_param('/wp_nav/exit_cnt', 5)

        self.tune = rospy.get_param('/wp_nav/tune', False)
        self.tune_x = rospy.get_param('/wp_nav/tune_x', False)
        self.tune_y = rospy.get_param('/wp_nav/tune_y', False)
        self.tune_yaw = rospy.get_param('/wp_nav/tune_yaw', False)
        if self.tune and not self.tune_x and not self.tune_y and not self.tune_yaw:
            rospy.logerr("调试时虚至少指定一轴进行调试\n\n\n")
            return

        self.DE2RA = np.pi / 180

        self.task = True
        self.get_car_pose = False
        self.car_pose = PoseStamped()
        self.car_pose.pose.orientation.w = 1
        self.get_target = False
        self.now_target_id = 1
        self.last_target_id = -1
        self.car_target = PoseStamped()
        self.car_target.pose.orientation.w = 1
        self.car_target_xyz = [0, 0, 0]
        self.car_target_rpy = [0, 0, 0]
        self.last_yaw = 0
        self.last_spin_dir = 1

        self.wait_for_acquire = False
        self.loc_lost_time = time.time()
        self.suspend = False
        self.arrive_cnt = 0
        
        self.car_cmd_s = CarCmd()
        self.car_cmd_s.cmd = CarCmd.REMOTE
        self.car_cmd_p = CarCmd()
        self.car_cmd_p.cmd = CarCmd.HOLD
        self.cmd_vel = Twist()
        self.zero_vel = Twist()

        self.car_pid_x = PID_incremental(p=self.kp_x, i=self.ki_x, d=self.kd_x, i_a=i_a_x, o_t=o_t_x)
        self.car_pid_y = PID_incremental(p=self.kp_y, i=self.ki_y, d=self.kd_y, i_a=i_a_y, o_t=o_t_y)
        self.car_pid_yaw = PID_incremental(p=self.kp_yaw, i=self.ki_yaw, d=self.kd_yaw, i_a=i_a_yaw, o_t=o_t_yaw)

        car_pid_x_sub = rospy.Subscriber('wp_nav_pid/pid_x', PID, self.car_pid_x_cb)
        car_pid_y_sub = rospy.Subscriber('wp_nav_pid/pid_y', PID, self.car_pid_y_cb)
        car_pid_yaw_sub = rospy.Subscriber('wp_nav_pid/pid_yaw', PID, self.car_pid_yaw_cb)
        
        car_cmd_sub = rospy.Subscriber('car_cmd', CarCmd, self.car_cmd_cb)
        tag_det_sub = rospy.Subscriber('apriltag_detected', Bool, self.tag_det_cb)
        car_pose_sub = rospy.Subscriber('car_pose_filt_map', PoseStamped, self.car_pose_cb)
        car_target_sub = rospy.Subscriber('car_target', PoseStamped, self.car_target_cb)
        exit_condition_sub = rospy.Subscriber('exit_condition', ExitCondition, self.exit_condition_cb)
        
        self.car_cmd_pub = rospy.Publisher('car_cmd', CarCmd, queue_size=3)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=3)

        time.sleep(12)
        self.car_nav_timer = rospy.Timer(rospy.Duration(0.05), self.car_navigator)

        rospy.loginfo(f"航点导航成功启动!\n")

    def __del__(self):
        print('\n航点导航终止\n')

    def tag_det_cb(self, msg:Bool):
        self.get_car_pose = msg.data

    def car_pose_cb(self, msg:PoseStamped):
        self.car_pose = msg

    def car_pid_x_cb(self, msg:PID):
        self.kp_x = msg.p
        self.ki_x = msg.i
        self.kd_x = msg.d
        self.car_pid_x.update_param(p=msg.p, i=msg.i, d=msg.d, i_a=msg.i_activate, o_t=msg.out_thresh)
    
    def car_pid_y_cb(self, msg:PID):
        self.kp_y = msg.p
        self.ki_y = msg.i
        self.kd_y = msg.d
        self.car_pid_y.update_param(p=msg.p, i=msg.i, d=msg.d, i_a=msg.i_activate, o_t=msg.out_thresh)
        
    def car_pid_yaw_cb(self, msg:PID):
        self.kp_yaw = msg.p
        self.ki_yaw = msg.i
        self.kd_yaw = msg.d
        self.car_pid_yaw.update_param(p=msg.p, i=msg.i, d=msg.d, i_a=msg.i_activate, o_t=msg.out_thresh)        

    def car_cmd_cb(self, msg:CarCmd):
        self.car_cmd_s = msg

    def car_target_cb(self, msg:PoseStamped):
        self.car_cmd_s.cmd = CarCmd.NAVIGATION
        # if msg.header.seq > self.last_target_id:
        self.now_target_id = msg.header.seq
        self.car_pid_x.reset()
        self.car_pid_y.reset()
        self.car_pid_yaw.reset()
        self.car_target_xyz = [msg.pose.position.x, 
                               msg.pose.position.y, 
                               msg.pose.position.z]
        rpy =  Rotation.from_quat([msg.pose.orientation.x, 
                                   msg.pose.orientation.y, 
                                   msg.pose.orientation.z, 
                                   msg.pose.orientation.w]).as_euler('xyz', degrees=True)
        self.car_target_rpy = [rpy[0], 
                               rpy[1], 
                               rpy[2]]
        self.get_target = True
        rospy.logwarn("航点导航获得第%d个航点 @ x:%.2f  y:%.2f  yaw:%.2f\n\n", self.now_target_id, self.car_target_xyz[0]*100, self.car_target_xyz[1]*100, self.car_target_rpy[2])
        # else:
        #     self.get_target = False

    def exit_condition_cb(self, msg:ExitCondition):
        self.exit_l = msg.exit_l
        self.exit_a = msg.exit_a
        self.arrive_cnt_num = msg.arrive_cnt

    def car_navigator(self, *args):
        if not self.task:
            return
        
        canvas = np.zeros((400, 480, 3), dtype=np.uint8)

        if self.car_cmd_s.cmd == CarCmd.REMOTE:
            cv2.putText(canvas, "Remote Control",
                        (10, 30), 3, 0.8, (0, 0, 255), 2)
            
            if self.get_car_pose:
                quat = self.car_pose.pose.orientation
                rpy = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz', degrees=True)
                cur = [self.car_pose.pose.position.x,
                    self.car_pose.pose.position.y,
                    rpy[2]]
                cv2.putText(canvas, "Cur x:%.1f y:%.1f a:%.1f" 
                            % (cur[0] * 100, cur[1] * 100, cur[2]),
                            (10, 170), 3, 0.8, (255, 120, 0), 2)
            cv2.imshow('nav_info', canvas)
            key = cv2.waitKey(1) % 0xff
            if key == 27:
                cv2.destroyAllWindows()
                self.car_cmd_p.cmd = self.car_cmd_p.HOLD
                self.car_cmd_pub.publish(self.car_cmd_p)
                self.task = False
            return

        if not self.get_car_pose:
            if not self.wait_for_acquire:
                self.loc_lost_time = time.time()
                self.wait_for_acquire = True
            if time.time() - self.loc_lost_time > 1 and self.car_cmd_s.cmd == CarCmd.NAVIGATION:
                self.cmd_vel.linear.x = -0.05
                self.cmd_vel.linear.y = 0
                self.cmd_vel.angular.z = self.last_spin_dir * 5 * self.DE2RA
                self.car_cmd_p.cmd = self.car_cmd_p.NAVIGATION
                self.car_cmd_pub.publish(self.car_cmd_p)
                self.cmd_vel_pub.publish(self.cmd_vel)

            cv2.putText(canvas, "Wait for self Location",
                        (10, 30), 3, 0.8, (0, 0, 255), 2)
            
            cv2.imshow('nav_info', canvas)
            key = cv2.waitKey(1) % 0xff
            if key == 27:
                cv2.destroyAllWindows()
                self.car_cmd_p.cmd = self.car_cmd_p.HOLD
                self.car_cmd_pub.publish(self.car_cmd_p)
                self.task = False
            elif key == 32:
                self.car_cmd_p.cmd = self.car_cmd_p.HOLD
                self.car_cmd_pub.publish(self.car_cmd_p)
                self.suspend = True
            return
        
        self.wait_for_acquire = False
        quat = self.car_pose.pose.orientation
        rpy = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz', degrees=True)
        
        cur = [self.car_pose.pose.position.x,
               self.car_pose.pose.position.y,
               rpy[2]]
        cur_yaw_RA = rpy[2] * self.DE2RA

        cv2.putText(canvas, "X p:%.2f i:%.2f d:%.2f" 
                    % (self.kp_x, self.ki_x, self.kd_x),
                    (10, 60), 3, 0.8, (255, 120, 0), 2)
        cv2.putText(canvas, "Y p:%.2f i:%.2f d:%.2f" 
                    % (self.kp_y, self.ki_y, self.kd_y),
                    (10, 85), 3, 0.8, (255, 120, 0), 2)
        cv2.putText(canvas, "W p:%.2f i:%.2f d:%.2f" 
                    % (self.kp_yaw, self.ki_yaw, self.kd_yaw),
                    (10, 110), 3, 0.8, (255, 120, 0), 2)
        
        cv2.putText(canvas, "Cur x:%.1f y:%.1f a:%.1f" 
                    % (cur[0] * 100, cur[1] * 100, cur[2]),
                    (10, 170), 3, 0.8, (255, 120, 0), 2)
        
        if not self.get_target:
            if self.car_cmd_s.cmd == CarCmd.NAVIGATION:
                self.car_cmd_p.cmd = self.car_cmd_p.HOLD
                self.car_cmd_pub.publish(self.car_cmd_p)
            cv2.putText(canvas, "Wait for Target",
                        (10, 30), 3, 0.8, (0, 0, 255), 2)
            cv2.imshow('nav_info', canvas)
            key = cv2.waitKey(1) % 0xff
            if key == 27:
                cv2.destroyAllWindows()
                self.car_cmd_p.cmd = self.car_cmd_p.HOLD
                self.car_cmd_pub.publish(self.car_cmd_p)
                self.task = False
            elif key == 32:
                self.car_cmd_p.cmd = self.car_cmd_p.HOLD
                self.car_cmd_pub.publish(self.car_cmd_p)
                self.suspend = True
            return
        
        if self.suspend:
            self.car_cmd_p.cmd = self.car_cmd_p.HOLD
            self.car_cmd_pub.publish(self.car_cmd_p)

            cv2.putText(canvas, "Nav Suspend",
                        (10, 30), 3, 0.8, (0, 0, 255), 2)
            
            cv2.imshow('nav_info', canvas)
            key = cv2.waitKey(1) % 0xff
            if key == 27:
                cv2.destroyAllWindows()
                self.car_cmd_p.cmd = self.car_cmd_p.HOLD
                self.car_cmd_pub.publish(self.car_cmd_p)
                self.task = False
            elif key == 32:
                self.car_cmd_p.cmd = self.car_cmd_p.HOLD
                self.car_cmd_pub.publish(self.car_cmd_p)
                self.suspend = False
            return
        
        cv2.putText(canvas, "Nav On",
                    (10, 30), 3, 0.8, (0, 255, 0), 2)    
        cv2.putText(canvas, "Tar-%d x:%.1f y:%.1f a:%.1f" 
                    % (self.now_target_id,
                       self.car_target_xyz[0] * 100, 
                       self.car_target_xyz[1] * 100, 
                       self.car_target_rpy[2]),
                    (10, 140), 3, 0.8, (255, 120, 0), 2)
        
        err_yaw = self.car_target_rpy[2] - cur[2]
        err_yaw = 180 - (err_yaw + 180) % 360

        vel_x_world = self.car_pid_x.update(target=self.car_target_xyz[0], now=cur[0]) if not self.tune or (self.tune and self.tune_x) else 0
        vel_y_world = self.car_pid_y.update(target=self.car_target_xyz[1], now=cur[1]) if not self.tune or (self.tune and self.tune_y) else 0
        vel_z_world = self.car_pid_yaw.update(target=0, now=err_yaw) if not self.tune or (self.tune and self.tune_yaw) else 0

        cv2.putText(canvas, "err_x:%.1f err_y:%.1f err_a:%.1f" % (self.car_pid_x.err*100, self.car_pid_y.err*100, self.car_pid_yaw.err),
                    (10, 260), 3, 0.8, (255, 120, 0), 2)
        
        cond_x = (abs(self.car_pid_x.err) < self.exit_l) if not self.tune or (self.tune and self.tune_x) else True
        cond_y = (abs(self.car_pid_y.err) < self.exit_l) if not self.tune or (self.tune and self.tune_y) else True
        cond_yaw = (abs(self.car_pid_yaw.err) < self.exit_a) if not self.tune or (self.tune and self.tune_yaw) else True
        cond = cond_x and cond_y and cond_yaw
        if cond:
            self.arrive_cnt += 1
            rospy.logwarn(f"航点 {self.now_target_id} 到位 {self.arrive_cnt}\n")
            if self.arrive_cnt >= self.arrive_cnt_num:
                self.car_pid_x.reset()
                self.car_pid_y.reset()
                self.car_pid_yaw.reset()
                self.cmd_vel_pub.publish(self.zero_vel)
                time.sleep(.1)
                self.get_target = False
                self.last_target_id = self.now_target_id
                self.car_cmd_p.cmd = self.car_cmd_p.ARRIVE
                self.car_cmd_pub.publish(self.car_cmd_p)
                time.sleep(.1)
                self.arrive_cnt = 0
            cv2.imshow('nav_info', canvas)
            key = cv2.waitKey(1) % 0xff
            if key == 27:
                cv2.destroyAllWindows()
                self.car_cmd_p.cmd = self.car_cmd_p.HOLD
                self.car_cmd_pub.publish(self.car_cmd_p)
                self.task = False
            elif key == 32:
                self.car_cmd_p.cmd = self.car_cmd_p.HOLD
                self.car_cmd_pub.publish(self.car_cmd_p)
                self.suspend = True
            return
        else:
            self.arrive_cnt = 0
        
        vel_body_x = (vel_x_world * np.cos(cur_yaw_RA) + vel_y_world * np.sin(cur_yaw_RA))  # x fb
        vel_body_y = (-vel_x_world * np.sin(cur_yaw_RA) + vel_y_world * np.cos(cur_yaw_RA)) # y lr
        vel_body_z = vel_z_world                                                            # w rl
        
        self.cmd_vel.linear.x = vel_body_x
        self.cmd_vel.linear.y = vel_body_y
        self.cmd_vel.angular.z = vel_body_z * self.DE2RA

        if vel_body_z > 0:
            self.last_spin_dir = 1 
        elif vel_body_z < 0:
            self.last_spin_dir = -1
        else:
             self.last_spin_dir = 0 
        self.car_cmd_p.cmd = self.car_cmd_p.NAVIGATION
        self.car_cmd_pub.publish(self.car_cmd_p)
        self.cmd_vel_pub.publish(self.cmd_vel)
        
        cv2.putText(canvas, 'w  x:%.1f y:%.1f z:%.1f' % (vel_x_world*100, vel_y_world*100, vel_z_world),
                    (10, 305), 3, 0.6, (255, 120, 0), 2)
        cv2.putText(canvas, 'b  x:%.1f y:%.1f z:%.1f' % (vel_body_x*100, vel_body_y*100, vel_body_z),
                    (10, 330), 3, 0.6, (255, 120, 0), 2)
        
        cv2.imshow('nav_info', canvas)

        key = cv2.waitKey(1) % 0xff
        
        if key == 27:
            cv2.destroyAllWindows()
            self.car_cmd_p.cmd = self.car_cmd_p.HOLD
            self.car_cmd_pub.publish(self.car_cmd_p)
            self.task = False
        elif key == 32:
            self.car_cmd_p.cmd = self.car_cmd_p.HOLD
            self.car_cmd_pub.publish(self.car_cmd_p)
            self.suspend = True



if __name__ == "__main__":
    rospy.init_node('wp_nav_pid_node')

    wp_nav_pid_node = WaypointNavigationPIDNode()
    rospy.spin()
    
