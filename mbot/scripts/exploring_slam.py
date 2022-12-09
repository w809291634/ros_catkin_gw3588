#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import roslib;
import rospy  
import actionlib  
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
from random import sample  
from math import pow, sqrt  
import sys, select, termios, tty

msg = """
Control mbot!
---------------------------
选择移动到坐标点：
1: 到坐标点1号位置
2: 到坐标点2号位置
3: 到坐标点3号位置
4: 到坐标点4号位置

CTRL-C to quit
"""
class ExploringSlam():
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        rospy.init_node('exploring_slam', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  

        # 在每个目标位置暂停的时间 (单位：s)
        self.rest_time = rospy.get_param("~rest_time", 2)  

        # 是否仿真？  
        self.fake_test = rospy.get_param("~fake_test", True)  

        # 到达目标的状态  
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',   
                       'SUCCEEDED', 'ABORTED', 'REJECTED',  
                       'PREEMPTING', 'RECALLING', 'RECALLED',  
                       'LOST']  


        # 发布控制机器人的消息  
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)  

        # 订阅move_base服务器的消息  
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)  

        rospy.loginfo("Waiting for move_base action server...")  

        # 60s等待时间限制  
        self.move_base.wait_for_server(rospy.Duration(60))  
        rospy.loginfo("Connected to move base server")  
      

        # 保存机器人的在rviz中的初始位置  
        initial_pose = PoseWithCovarianceStamped()  

        # 保存成功率、运行时间、和距离的变量  
        n_goals = 0  
        n_successes = 0  
        distance_traveled = 0  
        start_time = rospy.Time.now()  
        running_time = 0  
        location = ""  
        last_location = ""    

        # 确保有初始位置  
        while initial_pose.header.stamp == "":  
            rospy.sleep(1) 
        
        rospy.loginfo("Starting navigation test")  

        print(msg)
        # 开始主循环，定点导航 
        while not rospy.is_shutdown():  

            # 设定目标点  
            self.goal = MoveBaseGoal()  
            
            self.goal.target_pose.header.frame_id = 'map'  
            self.goal.target_pose.header.stamp = rospy.Time.now()  

            key = self.getKey()
            if key == '1':
                # 设置1号位置坐标 
                location = Pose(Point(4.589, -0.376, 0.000),  Quaternion(0.0, 0.0, 0.0, 1.0))  
                self.goal.target_pose.pose = location
                self.move_base.send_goal(self.goal)  
                rospy.loginfo("正在行驶去坐标点1号位置")  
                # 五分钟时间限制  
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))   
                # 查看是否成功到达  
                if not finished_within_time:  
                    self.move_base.cancel_goal()  
                    rospy.loginfo("到达目的地超时")  
                else:  
                    state = self.move_base.get_state()  
                    if state == GoalStatus.SUCCEEDED:  
                        rospy.loginfo("成功到达目的地!") 
                        n_successes += 1  
                    else:  
                        rospy.loginfo("错误: " + str(goal_states[state]))  


            elif key == '2':
                # 设置2号位置坐标
                location = Pose(Point(4.231, -6.050, 0.000),  Quaternion(0.000, 0.000, -0.847, 0.532))  
                self.goal.target_pose.pose = location
                self.move_base.send_goal(self.goal)  
                rospy.loginfo("正在行驶去坐标点2号位置")
                # 五分钟时间限制  
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))   
                # 查看是否成功到达  
                if not finished_within_time:  
                    self.move_base.cancel_goal()  
                    rospy.loginfo("到达目的地超时")  
                else:  
                    state = self.move_base.get_state()  
                    if state == GoalStatus.SUCCEEDED:  
                        rospy.loginfo("成功到达目的地!")  
                        n_successes += 1  
                    else:  
                        rospy.loginfo("错误: " + str(goal_states[state]))  
            elif key == '3':
                # 设置3号位置坐标
                location = Pose(Point(-0.674, -5.244, 0.000), Quaternion(0.000, 0.000, 0.000, 1.000))  
                self.goal.target_pose.pose = location
                self.move_base.send_goal(self.goal)  
                rospy.loginfo("正在行驶去坐标点3号位置") 
                # 五分钟时间限制  
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))   
                # 查看是否成功到达  
                if not finished_within_time:  
                    self.move_base.cancel_goal()  
                    rospy.loginfo("到达目的地超时")  
                else:  
                    state = self.move_base.get_state()  
                    if state == GoalStatus.SUCCEEDED:  
                        rospy.loginfo("成功到达目的地!")   
                        n_successes += 1  
                    else:  
                        rospy.loginfo("错误: " + str(goal_states[state]))  
            elif key == '4':
                # 设置4号位置坐标
                location = Pose(Point(-5.543, -4.779, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764))  
                self.goal.target_pose.pose = location
                self.move_base.send_goal(self.goal)  
                rospy.loginfo("正在行驶去坐标点4号位置")
                # 五分钟时间限制  
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))   
                # 查看是否成功到达  
                if not finished_within_time:  
                    self.move_base.cancel_goal()  
                    rospy.loginfo("到达目的地超时")   
                else:  
                    state = self.move_base.get_state()  
                    if state == GoalStatus.SUCCEEDED:  
                        rospy.loginfo("成功到达目的地!")  
                        n_successes += 1  

                    else:  
                        rospy.loginfo("错误: " + str(goal_states[state]))  

            else:
                if (key == '\x03'):
                    break
            

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    def shutdown(self):
        rospy.loginfo("Stopping the robot...")  
        self.move_base.cancel_goal()  
        rospy.sleep(2)  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)  

if __name__ == '__main__':  
    
    try:  
        ExploringSlam()  
        rospy.spin()  

    except rospy.ROSInterruptException:  
        rospy.loginfo("Exploring SLAM finished.")

    