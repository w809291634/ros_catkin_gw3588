#!/usr/bin/env python3
# -*- coding:utf8 -*-
import rospy
from std_msgs.msg import String, Int32MultiArray
import logging
import threading 
import sys
import json

logger = logging.getLogger('mylogger')
logger.setLevel(logging.DEBUG)


class RosCarSensors(object):
    def __init__(self):
        
        # rospy.init_node('car_sensors', anonymous=True)
        rospy.Subscriber("/xcar/sensors", Int32MultiArray, self.callback)

        self.msg_count = 0
        self.speed = 0

    def spin_thread(self):
        rospy.spin()

    def get_sensor_info(self,car_control_speed):
        print("speed")
        self.speed = car_control_speed

        ros_thread = threading.Thread(target=self.spin_thread)
        ros_thread.start()
        return self.sensor_data_list

    def process_data(self,data_list1):
        # print(data_list1)
        data_list = []
        data_value = {}
        data_value = {"type":"battery_voltage","value":data_list1[0],"unit":"V"}
        data_list.append(data_value)

        data_value = {"type":"temp","value":int(data_list1[1])/10,"unit":"C"}
        data_list.append(data_value)

        data_value = {"type":"hum","value":data_list1[2],"unit":"%"}
        data_list.append(data_value)

        data_value = {"type":"pressure_temp","value":data_list1[3],"unit":"C"}
        data_list.append(data_value)

        data_value = {"type":"pressure","value":data_list1[4],"unit":""}
        data_list.append(data_value)

        data_value = {"type":"illumination","value":data_list1[5],"unit":"Lux"}
        data_list.append(data_value)

        data_value = {"type":"air_quality","value":data_list1[6],"unit":"kPa"}
        data_list.append(data_value)

        data_value = {"type":"smoke","value":data_list1[7],"unit":""}
        data_list.append(data_value)

        data_value = {"type":"left_forward","value":data_list1[8],"unit":"cm"}
        data_list.append(data_value)

        data_value = {"type":"backward","value":data_list1[9],"unit":"cm"}
        data_list.append(data_value)

        data_value = {"type":"right_forward","value":data_list1[10],"unit":"cm"}
        data_list.append(data_value)

        forward_value = (int(data_list1[8]) + int(data_list1[10]))/2
        data_value = {"type":"forward","value":forward_value,"unit":"cm"}
        data_list.append(data_value)

        data_value = {"type":"speed","value":self.speed,"unit":"m/s"}
        data_list.append(data_value)

        self.sensor_data_list = data_list

    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + " data_info:%s", data.data)
        self.msg_count+=1
        data_str = (str(data.data)[1:-1])
        data_arr = data_str.split(", ")
        self.process_data(data_arr)

# if __name__ == '__main__':
#     r = RosCarSensors()
#     sys.stdout.write('input:\n')
#     # sensor_list = []
#     while True:
#         line = sys.stdin.readline()
#         sensor_list = line.replace("\n","").split(",")
#         print(sensor_list)
#         r.get_sensor_info(sensor_list)
#         # r.get_sensor_info(["temp","hum","voltage"])