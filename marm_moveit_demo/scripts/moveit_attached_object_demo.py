#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import rospy, sys
import _thread, copy
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface,Constraints
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene ,JointConstraint
from math import radians
from copy import deepcopy

class MoveAttachedObjectDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_attached_object_demo')
        
        # 初始化场景对象
        scene = PlanningSceneInterface()
        rospy.sleep(1)
                                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('manipulator')

        # 设定路径约束,规定arm_joint_4和arm_joint_3的运动范围
        Constraint=Constraints()

        arm_joint_4= JointConstraint()
        arm_joint_4.joint_name='arm_joint_4'
        arm_joint_4.position=1.36
        arm_joint_4.tolerance_above=1
        arm_joint_4.tolerance_below=1.5
        Constraint.joint_constraints.append(arm_joint_4)

        arm_joint_3= JointConstraint()
        arm_joint_3.joint_name='arm_joint_3'
        arm_joint_3.position=1.36
        arm_joint_3.tolerance_above=1
        arm_joint_3.tolerance_below=1.5
        Constraint.joint_constraints.append(arm_joint_3)

        arm.set_path_constraints(Constraint)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
       
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)

        # 设置每次运动规划的时间限制：10s
        arm.set_planning_time(10)

        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()

        # 移除场景中之前运行残留的物体
        scene.remove_attached_object(end_effector_link, 'tool')
        scene.remove_world_object('table') 
        scene.remove_world_object('tool')

        # 设置桌面的高度
        table_ground = 0.4
        
        # 设置table的三维尺寸
        table_size = [0.1, 0.3, 0.02]
        tool_size = [0.02, 0.02, 0.05]

        # 设置tool的位姿
        p = PoseStamped()
        p.header.frame_id = end_effector_link

        p.pose.position.x = 0.0
        p.pose.position.y = 0.0
        p.pose.position.z = 0.0+0.02
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 0
        
        #将tool附着到机器人的终端
        scene.attach_box(end_effector_link, 'tool', p, tool_size)

        # 将table加入场景当中
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'base_link'
        table_pose.pose.position.x = 0.20
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box('table', table_pose, table_size)
        
        rospy.sleep(2)       
        # 控制机械臂回到start位置
        arm.set_named_target('start')
        arm.go()

        rospy.sleep(2)   

        # 控制机械臂回到home位置
        arm.set_named_target('home')
        arm.go()

        # 移除所有场景便于下次实验
        scene.remove_attached_object(end_effector_link, 'tool')
        rospy.sleep(1) 
        scene.remove_world_object('table') 
        scene.remove_world_object('tool')
        rospy.sleep(2)  

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveAttachedObjectDemo()

    
