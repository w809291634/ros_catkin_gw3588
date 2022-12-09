#!/usr/bin/env python3
# -*- coding:utf8 -*-
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32MultiArray,Int16MultiArray,Int32

import logging

logger = logging.getLogger('mylogger')
logger.setLevel(logging.DEBUG)

class RosCarAvoidObstacles(object):
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/xcar/sensors", Int32MultiArray, self.callback)
        rate = rospy.Rate(1)
        self.left_sonar = 0
        self.right_sonar = 0
        self.rear_sonar = 0
        self.warn_value = 60
        
        self.default_speed = 0.2
        self.default_angular = 0.5

        self.left_counter = 0
        self.right_counter = 0
        self.backward_counter = 0

        self.left_min = 50   # maximum 720
        self.left_max = 700   # maximum 720
        self.right_min = 100  # maximum 260
        self.right_max = 250  # maximum 260

        self.hit_left_counter = 0
        self.hit_right_counter = 0

    def send_message(self, control_speed, control_turn):
        twist = Twist()
        twist.linear.x = control_speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = control_turn
        self.pub.publish(twist)

    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + " data_info:%s", data.data)
        data_str = (str(data.data)[1:-1])
        data_arr = data_str.split(", ")
        self.left_sonar = int(data_arr[8])
        self.right_sonar = int(data_arr[10])
        self.rear_sonar = int(data_arr[9])
        print("left_sonar: "+str(self.left_sonar))
        print("right_sonar: "+str(self.right_sonar))
        print("rear_sonar: "+str(self.rear_sonar))
        self.avoid_obstacles()

    def avoid_obstacles(self):
        result = ""
       
        if ((self.left_sonar > self.warn_value) and (self.right_sonar > self.warn_value)):
            self.send_message(self.default_speed, 0)
            result = "forward"
        if ((self.warn_value < self.left_sonar < self.left_max) and (self.right_sonar < self.warn_value)):
            result = "left"
            if (self.left_counter >= 3):
                self.send_message(self.default_speed, 0.5)
            else:
                self.send_message(self.default_speed, self.default_angular)
            
        elif ((0 < self.left_sonar < self.warn_value) and (self.right_sonar > self.right_sonar > self.warn_value)):
            result = "right"
            if (self.right_counter >= 3):
                self.send_message(self.default_speed, -0.5)
            else:
                self.send_message(self.default_speed, -self.default_angular)
            
        elif ((0 < self.left_sonar < self.warn_value) and (0 < self.right_sonar < self.warn_value)):
            # self.send_message(-self.default_speed, 0)
            result = "backward"
            self.backward_counter += 1
            if (self.backward_counter >= 3):
                self.send_message(-0.3, 0)
            else:
                self.send_message(-self.default_speed, 0)
        elif (self.left_sonar > self.left_max):
            print("hit left.......")
            result = "backward"
            self.hit_left_counter += 1
            self.send_message(-0.2, -1)
            
        elif (self.right_sonar > self.right_max):
            print("hit right.......")
            result = "backward"
            self.hit_right_counter += 1
            self.send_message(-0.2, 1)
        elif (0 < self.rear_sonar < self.warn_value):
            self.send_message(0, 0)
            result = "stop"
        elif (self.hit_left_counter >=3):
            print("hit left counter.......")
            self.send_message(0.1, -1)
        elif (self.hit_right_counter >=3):
            print("hit right counter.......")
            self.send_message(0.1, 1)

        # else:
        #     self.send_message(self.default_speed, 0)
        #     result = "forward"
        print(result)
        # return result

if __name__ == '__main__':
    rospy.init_node('car_move',anonymous=True)
    p = RosCarAvoidObstacles()
    rospy.spin()