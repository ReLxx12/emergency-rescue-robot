#!/home/orangepi/miniconda3/envs/rknn/bin/python
# -*- coding: utf-8 -*-
import time
import math
import smbus
import numpy as np

import rospy
from std_msgs.msg import Bool
from user_msgs.msg import GimbalPoseDeg


class PCA9685ControlNode:
    # Registers/etc.
    __SUBADR1 = 0x02
    __SUBADR2 = 0x03
    __SUBADR3 = 0x04
    __MODE1 = 0x00
    __PRESCALE = 0xFE
    __LED0_ON_L = 0x06
    __LED0_ON_H = 0x07
    __LED0_OFF_L = 0x08
    __LED0_OFF_H = 0x09
    __ALLLED_ON_L = 0xFA
    __ALLLED_ON_H = 0xFB
    __ALLLED_OFF_L = 0xFC
    __ALLLED_OFF_H = 0xFD

    def __init__(self) -> None:
        rospy.loginfo(f"云台节点初始化中...\n")
        # self.debug = False
        # if self.debug:
        #     print("Reseting PCA9685")
        i2c_bus = rospy.get_param('gimbal/i2c_bus', 1)  # jetson port
        i2c_address = rospy.get_param('gimbal/i2c_address', 0x40)  # servo port
        
        self.id_pitch = rospy.get_param('gimbal/id_pitch', 1)
        self.id_yaw = rospy.get_param('gimbal/id_yaw', 0)
        
        self.err_pitch = -rospy.get_param('gimbal/differ_pitch', 0)
        self.err_yaw = rospy.get_param('gimbal/differ_yaw', 0)
        
        self.range_pitch = rospy.get_param('gimbal/range_pitch', [-10, 10])
        self.range_yaw = rospy.get_param('gimbal/range_yaw', [-90, 90])

        self.bus = smbus.SMBus(i2c_bus)
        # if self.debug:
        #     print(f"Linked to i2c_{i2c_bus}")
        self.address = i2c_address
        # if self.debug:
        #     print(f"Linked to i2c device address {i2c_address}")
        self.write(self.__MODE1, 0x00)

        self.setPWMFreq(50)
        self.setServoDegree(self.id_pitch, 90+self.err_pitch)
        self.setServoDegree(self.id_yaw, 90+self.err_yaw)

        self.task = True
        self.get_cmd = False

        self.task_sub = rospy.Subscriber('/task', Bool, self.task_cb)
        self.gimbal_pose_ctrl_sub = rospy.Subscriber("/gimbal_pose_ctrl", GimbalPoseDeg, self.gimbal_pose_cb)
        
        self.gimbal_pose_now_pub = rospy.Publisher("/gimbal_pose_now", GimbalPoseDeg, queue_size=1) 

        rospy.loginfo(f"云台节点初始化成功\n")
      
    def write(self, reg, value):
        """Writes an 8-bit value to the specified register/address"""
        self.bus.write_byte_data(self.address, reg, value)
        # if self.debug:
        #     print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))

    def read(self, reg):
        """Read an unsigned byte from the I2C device"""
        result = self.bus.read_byte_data(self.address, reg)
        # if self.debug:
        #     print("I2C: Device 0x%02X returned 0x%02X from reg 0x%02X" % (self.address, result & 0xFF, reg))
        return result

    def setPWMFreq(self, freq):
        """Sets the PWM frequency"""
        # if self.debug:
        #     print("Setting PWM frequency to %d Hz" % freq)
        prescaleval = 25000000.0  # 25MHz
        prescaleval /= 4096.0  # 12-bit
        prescaleval /= float(freq)
        prescaleval -= 1.0
        # if self.debug:
        #     print("Estimated pre-scale: %d" % prescaleval)
        prescale = math.floor(prescaleval + 0.5)
        # if self.debug:
        #     print("Final pre-scale: %d" % prescale)

        oldmode = self.read(self.__MODE1)
        newmode = (oldmode & 0x7F) | 0x10  # sleep
        self.write(self.__MODE1, newmode)  # go to sleep
        self.write(self.__PRESCALE, int(math.floor(prescale)))
        self.write(self.__MODE1, oldmode)
        time.sleep(0.005)
        self.write(self.__MODE1, oldmode | 0x80)

    def setPWM(self, channel, on, off):
        """Sets a single PWM channel"""
        self.write(self.__LED0_ON_L + 4 * channel, on & 0xFF)
        self.write(self.__LED0_ON_H + 4 * channel, on >> 8)
        self.write(self.__LED0_OFF_L + 4 * channel, off & 0xFF)
        self.write(self.__LED0_OFF_H + 4 * channel, off >> 8)
        # if self.debug:
        #     print("channel: %d  LED_ON: %d LED_OFF: %d" % (channel, on, off))

    # region
    # def setServoPulse(self, channel, pulse):
    #     """Sets the Servo Pulse,The PWM frequency must be 50HZ"""
    #     pulse = int(pulse * 4096 / 20000)  # PWM frequency is 50HZ,the period is 20000us
    #     self.setPWM(channel, 0, pulse)

    # def setMotoPluse(self, channel, pulse):
    #     if pulse > 3000:
    #         self.setPWM(channel, 0, 3000)
    #     else:
    #         self.setPWM(channel, 0, pulse)
    # endregion

    def setServoDegree(self, channel, degree):
        """Sets the Servo Degree,The PWM frequency must be 50HZ"""
        pulse = int((500 + degree * 2000 / 180) * 4096 / 20000)  # PWM frequency is 50HZ,the period is 20000us
        self.setPWM(channel, 0, pulse)

    def task_cb(self, msg:Bool):
        time.sleep(.5)
        self.task = msg.data
        if not self.task:
            print('\n云台节点结束\n')

    def gimbal_pose_cb(self, msg:GimbalPoseDeg):
        if not self.task:
            return
        pitch = np.clip(-msg.pitch, self.range_pitch[0], self.range_pitch[1])
        yaw = np.clip(msg.yaw, self.range_yaw[0], self.range_yaw[1])
        gimbal_pose_now = GimbalPoseDeg()
        gimbal_pose_now.pitch = -pitch
        gimbal_pose_now.yaw = yaw
        target_pitch = 90+pitch+self.err_pitch  # msg.pitch [-90, 90]  degree [0, 180]
        target_yaw = 90+yaw+self.err_yaw  # msg.yaw [-90, 90]  degree [0, 180]
        self.setServoDegree(self.id_pitch, degree=np.clip(target_pitch, 0, 180))
        self.setServoDegree(self.id_yaw, degree=np.clip(target_yaw, 0, 180))
        # time.sleep(.2)
        self.gimbal_pose_now_pub.publish(gimbal_pose_now)
        
    
def main():
    try:
        rospy.init_node('gimbal_node', anonymous=True)
    except rospy.ROSInterruptException:
        rospy.logwarn("云台节点初始化失败...")
    else:
        gimbal = PCA9685ControlNode()
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
    
    
if __name__ == '__main__':
    main()
    
