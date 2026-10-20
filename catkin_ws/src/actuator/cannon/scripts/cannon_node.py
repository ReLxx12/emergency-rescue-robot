#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-
import time
import serial

import rospy
from std_msgs.msg import Empty, Bool
from user_msgs.msg import CannonStatus


class CannonNode:
    def __init__(self) -> None:
        rospy.loginfo(f"激光节点初始化中...\n")

        serial_port = rospy.get_param('/cannon/serial_port', '/dev/CH340')
        serial_baudrate = rospy.get_param('/cannon/serial_baudrate', 115200)
        irradiation_time = rospy.get_param('/cannon/irradiation_time', 1)

        self.head = 0xa5
        self.tail = 0x5a

        self.t_irradiation = int(irradiation_time*10)  # *10s

        self.task = True
        try:
            self.ser = serial.Serial(port=serial_port, 
                                     baudrate=serial_baudrate, 
                                     # bytesize=8, parity='N', stopbits=1, 
                                     timeout=0.1)  # 打开串口
        except Exception as e:
            print(e)
            exit(f"tty串口 {serial_port} @ 波特率-{serial_baudrate} 打开失败")

        if self.ser.is_open:
            pass
        else:
            exit(f"tty串口 {serial_port} @ 波特率-{serial_baudrate} 打开失败")
            
        # self.cannon_cmd_data = [self.head] + [0x01, self.t_irradiation, 0x00, 0x00, 0x00, 0x00] + [self.tail]
        # print('send', self.cannon_cmd_data)
        # self.ser.write(self.cannon_cmd_data)

        rospy.loginfo(f"激光 \33[32m 解锁\33[0m\n")

        self.cannon_status = CannonStatus()
        self.recv_buf = []

        # 创建一个Subscriber，订阅名为cmd_vel的topic，注册回调函数cmd_vel_cb
        self.task_sub = rospy.Subscriber('/task', Bool, self.car_task_cb)
        self.trigger_sub = rospy.Subscriber('/trigger', Empty, self.trigger_cb)
        
        self.cannon_status_pub = rospy.Publisher('/cannon_status', CannonStatus, queue_size=5)

        self.controller_timer = rospy.Timer(rospy.Duration(0.02), self.controller)

        rospy.loginfo(f"激光节点初始化成功\n")
    
    def __del__(self):
        rospy.loginfo(f"激光 \33[31m锁定\33[0m\n")
        self.ser.close()
        rospy.logwarn('激光节点结束\n')
    
    def car_task_cb(self, msg:Bool):
        time.sleep(1)
        self.task = msg.data
        if not self.task:
            rospy.logwarn('激光节点开始关闭\n')
            self.__del__()
        
    def trigger_cb(self, msg:Empty):
        self.cannon_cmd_data = [self.head] + [0x01, self.t_irradiation, 0x00, 0x00, 0x00, 0x00] + [self.tail]
        while not self.cannon_status.rdy2fire:
            pass
        # rospy.loginfo(f'\33[31m射击 \33[35m{self.cannon_cmd_data}\33[0m\n')
        self.ser.write(self.cannon_cmd_data)

    def controller(self, *args):
        if not self.task:
            return
        
        try:
            recv_data = self.ser.readline()
        except TypeError:
            return
            
        if recv_data == b'':
            return
        
        recv_hex_ = []
        for byte in recv_data:
            recv_hex_.append(hex(byte)[2:].zfill(2))
        self.recv_buf += recv_hex_
        # rospy.loginfo(f"\033[32mGet msg\033[0m"
        #               f"\033[34m{recv_hex_}\033[0m\n")
        
        if 'a5' in self.recv_buf and '5a' in self.recv_buf:
            index_head = self.recv_buf.index('a5')
            index_tail = self.recv_buf.index('5a')
            buf = self.recv_buf[index_head:index_tail+1]
            self.recv_buf = self.recv_buf[index_tail+1:]
            
            self.cannon_status.bullet_num = int(buf[3], 16)
            self.cannon_status.capacity = int(buf[4], 16)
            self.cannon_status.reload_time = int(buf[6], 16)/10
            self.cannon_status.rdy2fire = bool(int(buf[5], 16))
            self.cannon_status_pub.publish(self.cannon_status)
        
    
def main():
    try:
        rospy.init_node('cannon_node', anonymous=True)
    except rospy.ROSInterruptException:
        rospy.logwarn("激光节点初始化失败...")
    else:
        chassis = CannonNode()
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            try:
                chassis.__del__()
            except serial.serialutil.PortNotOpenError:
                pass
    
    
if __name__ == '__main__':
    main()
    
