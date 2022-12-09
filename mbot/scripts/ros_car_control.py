#!/usr/bin/env python3
# -*- coding:utf8 -*-
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32MultiArray,Int16MultiArray,Int32

import os, sys, select, termios, tty,time, signal
import logging
import subprocess

logger = logging.getLogger('mylogger')
logger.setLevel(logging.DEBUG)

class RosCarControl(object):
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_arm = rospy.Publisher('xcar/arm', Int16MultiArray, queue_size=1)
        self.pub_gripper = rospy.Publisher('xcar/gripper', Int32, queue_size=1)
        self.pub_camera = rospy.Publisher('xcar/camptz', Int32, queue_size=1)
        
        rospy.init_node('car_move',anonymous=True)
        # self.speed = 1
        # self.angular = 0.5
        # self.sensor_type_list = []
        # self.data_value_list = []

    def send_message(self, control_speed, control_turn):
        twist = Twist()
        twist.linear.x = control_speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = control_turn
        self.pub.publish(twist)

    def speed_turn_info(self, speed, turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)
    
    def control_func(self, default_speed, default_angular, control_speed, control_turn):
        if (control_speed is not None and control_turn is None):
            speed = (float(control_speed) * default_speed)
            self.send_message(speed, 0)
        elif (control_turn is not None and control_speed is None):
            turn = (float(control_turn) * default_angular)
            self.send_message(1, turn)
        elif (control_speed is not None and control_turn is not None):
            speed = (float(control_speed) * default_speed)
            turn = (float(control_turn) * default_angular)
            self.send_message(speed, turn)
        elif (control_speed is None and control_turn is None):
            self.send_message(default_speed, default_angular)
        return "success"

    def car_control(self, type, cmd, control_speed, control_turn):
        result = ""
        if type == '0':
            if cmd == "forward":
                print(cmd)
                result = self.control_func(1, 0, control_speed, control_turn)
            elif cmd == "backward":
                print(cmd)
                result = self.control_func(-1, 0, control_speed, control_turn)
            elif cmd == "left":
                print(cmd)
                result = self.control_func(1, 1, control_speed, control_turn)

            elif cmd == "right":
                print(cmd)
                result = self.control_func(1, -1, control_speed, control_turn)

        elif type == '1':
            ####初始化机械臂 ####
            arm_arr = [0, -90, -90, -45, 0, 2000]
            arm_pickup = Int16MultiArray(data=arm_arr)
            rospy.loginfo(arm_pickup)
            self.pub_arm.publish(arm_pickup)
            rospy.sleep(3)
            if cmd == "throw_rubbish":
                print(cmd)
                #### 机械臂下降 ####
                arm_arr1 = [0, -90, 20, 100, 0, 2000]
                arm_pickup1 = Int16MultiArray(data=arm_arr1)
                rospy.loginfo(arm_pickup1)
                self.pub_arm.publish(arm_pickup1)
                rospy.sleep(1)

                #### 打开爪子 ####
                gripper_open = Int32(data=30)
                rospy.loginfo(gripper_open)
                self.pub_gripper.publish(gripper_open)
                rospy.sleep(1)

                #### 关闭爪子 ####
                gripper_close = Int32(data=-30)
                rospy.loginfo(gripper_close)
                self.pub_gripper.publish(gripper_close)
                rospy.sleep(1)

                #### 机械臂左移上升 ####
                arm_arr2 = [0, -90, 50, 50, 50, 2000]
                arm_pickup2 = Int16MultiArray(data=arm_arr2)
                rospy.loginfo(arm_pickup2)
                self.pub_arm.publish(arm_pickup2)
                rospy.sleep(2)

                #### 机械臂下降 ####
                arm_arr3 = [0, -90, 20, 100, 50, 2000]
                arm_pickup3 = Int16MultiArray(data=arm_arr3)
                rospy.loginfo(arm_pickup3)
                self.pub_arm.publish(arm_pickup3)
                rospy.sleep(2)

                #### 打开爪子 ####
                gripper_open = Int32(data=30)
                rospy.loginfo(gripper_open)
                self.pub_gripper.publish(gripper_open)
                rospy.sleep(1)

                ####机械臂复位####
                arm_arr = [0, -90, -90, -45, 0, 2000]
                arm_pickup = Int16MultiArray(data=arm_arr)
                rospy.loginfo(arm_pickup)
                self.pub_arm.publish(arm_pickup)
                rospy.sleep(3)
                result = "丢垃圾"
            elif cmd == "pour_water":
                print(cmd)
                #### 机械臂下降 ####
                arm_arr1 = [0, -90, 20, 100, 0, 2000]
                arm_pickup1 = Int16MultiArray(data=arm_arr1)
                rospy.loginfo(arm_pickup1)
                self.pub_arm.publish(arm_pickup1)
                rospy.sleep(1)

                #### 打开爪子 ####
                gripper_open = Int32(data=30)
                rospy.loginfo(gripper_open)
                self.pub_gripper.publish(gripper_open)
                rospy.sleep(1)

                #### 关闭爪子 ####
                gripper_close = Int32(data=-30)
                rospy.loginfo(gripper_close)
                self.pub_gripper.publish(gripper_close)
                rospy.sleep(1)

                #### 机械臂左移上升 ####
                arm_arr2 = [0, -90, 50, 50, 50, 2000]
                arm_pickup2 = Int16MultiArray(data=arm_arr2)
                rospy.loginfo(arm_pickup2)
                self.pub_arm.publish(arm_pickup2)
                rospy.sleep(2)

                #### 机械臂倒水 ####
                arm_arr3 = [50, -90, 50, 50, 50, 2000]
                arm_pickup3 = Int16MultiArray(data=arm_arr3)
                rospy.loginfo(arm_pickup3)
                self.pub_arm.publish(arm_pickup3)
                rospy.sleep(3)

                #### 机械臂下降 ####
                arm_arr3 = [0, -90, 20, 100, 50, 2000]
                arm_pickup3 = Int16MultiArray(data=arm_arr3)
                rospy.loginfo(arm_pickup3)
                self.pub_arm.publish(arm_pickup3)
                rospy.sleep(2)

                ####机械臂复位####
                arm_arr = [0, -90, -90, -45, 0, 2000]
                arm_pickup = Int16MultiArray(data=arm_arr)
                rospy.loginfo(arm_pickup)
                self.pub_arm.publish(arm_pickup)
                rospy.sleep(3)
                result = "倒水"
        elif type == '2':
            print("car avoid obstacles")
            global p
            if cmd == "open":
                print("open car avoid obstacles")
                p = subprocess.Popen(args=['/home/zonesion/catkin_ws/src/mbot/bin/start_avoid_obstacles.sh'], shell=True, close_fds=True, preexec_fn = os.setsid)
                result = "打开自动避障"
            elif cmd == "close":
                print("close car avoid obstacles")
                print(p.pid)
                os.killpg( p.pid,signal.SIGUSR1)
                self.control_func(0, 0, 0, 0)
                time.sleep(3)
                result = "关闭自动避障"
        elif type == '3':
            angle = int(cmd)
            self.pub_camera.publish(angle)
            result = "success"
        return result
