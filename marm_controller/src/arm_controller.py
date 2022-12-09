#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from actionlib import SimpleActionServer
from control_msgs.msg import FollowJointTrajectoryAction  
from control_msgs.msg import FollowJointTrajectoryResult
from std_msgs.msg import Int16MultiArray,Int32,Int32MultiArray
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import  GoalID
import time
import yaml
import sys
this = sys.modules[__name__]

this.config_path="/home/zonesion/catkin_ws/src/marm_controller/config/config.yaml"
with open(config_path, "r") as f:
    config = yaml.load(f.read(), Loader=yaml.FullLoader)

virtual_arm_joint_names=['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
virtual_arm_home_point=(0, -0.7850, 1.57, 1.57, 0, 0)                           
actually_joint_home_point=Int16MultiArray(data=[ 0, -89, -89, -44, 0, 2000])                         #home点和速度参数
g_open=config["g_open"]
arm_query_time=config["arm_query_time"]     #机械臂查询关节位置周期
arm_wait_para=config["arm_wait_para"]       #机械臂运动超时时间系数
arm_arrive_err=config["arm_arrive_err"]
gripper_arrive_err=config["gripper_arrive_err"]

this.arm_joint=[]
this.arm_res=0
this.cmd_cancel=0
    
class JointTrajectoryActionServer(object):
    # create messages that are used to publish feedback/result
    _result = FollowJointTrajectoryResult()

    def __init__(self, name):
        self._action_name = name
        self.ctlPub = rospy.Publisher('/xcar/arm', Int16MultiArray, queue_size=0, latch=True)
        self.gripperPub = rospy.Publisher('/xcar/gripper', Int32, queue_size=0, latch=True)
        self.jstPub = rospy.Publisher('/xcar/arm_joint_states', JointState, queue_size=0, latch=True)
        self._as = SimpleActionServer(self._action_name, FollowJointTrajectoryAction,  
                                                execute_cb=self.execute_cb_ex, auto_start=False)  
        self.arm_status_pub=rospy.Publisher('/xcar/arm_status_req', Int32, queue_size=0, latch=True)
        self._as.start()                                                        
        self.return_home()                                                      

    def arm_JointState_update(self,name,point):     
        jst = JointState()                          
        jst.name = name                             
        jst.position = point                        
        self.jstPub.publish(jst)                     

    def return_home(self):                                          
        self.ctlPub.publish(actually_joint_home_point)   
        time.sleep(0.5) 
        self.ctlPub.publish(actually_joint_home_point)   
        time.sleep(0.5) 
        self.ctlPub.publish(actually_joint_home_point)          #防止丢回home消息
        time.sleep(0.5)  
        self.gripperPub.publish(g_open)        
        time.sleep(1)                                                           
        self.arm_JointState_update(virtual_arm_joint_names,virtual_arm_home_point)      
        # tar_joint=actually_joint_home_point.data[:-1]
        # self.arm_run(actually_joint_home_point,tar_joint,6)
        # self.arm_JointState_update(virtual_arm_joint_names,virtual_arm_home_point)
        # self.gripperPub.publish(g_open)                                                

    def deocdePos(self, name, msg):
        #这里的/xcar/arm消息为6轴,其中位于列表第一个参数可以为0进行填充,后面5个列表参数如下
        jn = ["arm_joint_5", "arm_joint_4", "arm_joint_3", "arm_joint_2", "arm_joint_1"]
        direction = [1, -1, -1, 1, 1]
        offset = [ 0, 0, 0, 0, 0]
        sv = []
        i = 0
        for i in range(len(jn)):
            try:
                pos = msg.positions[name.index(jn[i])]                          
                pos = int(pos * (180 / 3.1415926)) * direction[i] + offset[i]   
            except:
                pos = 0
            sv.append(pos)                                                     
        return sv                                   
    
    def execute_cb(self, arg):                      #旧版本控制方式留存接口
        trajectory = arg.trajectory           
        this.cmd_cancel=0                           #默认为0    
        st=0                                        #轨迹点的起始运动时间 
        ns = [trajectory.points[0]]
        i = 0
        for point in trajectory.points[1:-1]:
            i += 1
            if i % 5 == 0:
                ns.append(point)
        ns.append(trajectory.points[-1])
        trajectory.points = ns
        rospy.logwarn("arm points len %d"%len(ns))
        for point in trajectory.points[1:]:     
            if this.cmd_cancel ==1:
                rospy.logwarn("The robots are stopping")
                return   
            at = point.time_from_start.secs + point.time_from_start.nsecs / 1000000000.0
            delay = at - st                 #计算此次点的路径时间
            st = at                         #保存此次时间点
            pos = self.deocdePos(trajectory.joint_names, point) 
            pos.append(int(delay*1000))     #机械臂最后一个参数可以控制运动速度
            pos = Int16MultiArray(data=pos) #转换消息格式
            self.ctlPub.publish(pos)                
            delay = delay - 0.02
            if delay > 0:
                time.sleep(delay)           #根据轨迹点设置动态延时
            self.arm_JointState_update(trajectory.joint_names,point.positions)
        self._as.set_succeeded(self._result)        

    def arm_arrive(self,joint1,joint2,err=10):
        if len(joint1)>0 and len(joint2)>0:
            if abs(joint1[0]-joint2[0])<err and abs(joint1[1]-joint2[1])<err and abs(joint1[2]-joint2[2])<err and \
            abs(joint1[3]-joint2[3])<err and abs(joint1[4]-joint2[4])<err:
                return True
            else:
                return False
        else:
            return False    

    def gripper_arrive(self,data1,data2,err=10):
        if abs(data1-data2)<err:
            return True
        else:
            return False  

    def arm_run(self,pos,tar_joint,wt):
        # pos: 发布给机械臂运动的消息，5个关节和1个时间参数
        # tar_joint：关节目标值
        # wt：超时等待时间
        st1=time.time()
        while True:
            self.ctlPub.publish(pos)  
            this.arm_res=0
            self.arm_status_pub.publish(Int32(1))           # 请求查询机械臂的关节位置
            time.sleep(arm_query_time)
            st=time.time()
            while not this.arm_res:
                time.sleep(0.02)
                if time.time()-st>arm_query_time:           # 等待
                    print("no arm_res")
                    break 
            if this.arm_res==1:
                # print("1",tar_joint)
                # print("2",this.arm_joint[1:])
                # print("//////////")
                if self.arm_arrive(tar_joint,this.arm_joint[1:],arm_arrive_err)==True:
                    rospy.loginfo("arm run success")
                    break
            if time.time()-st1>wt:                          # 机械臂的运动超时时间
                rospy.logerr("Manipulator movement timeout")
                break   

    def execute_cb_ex(self, arg):                    
        trajectory = arg.trajectory   
        run_time=len(trajectory.points)
        point=trajectory.points[-1]
        pos = self.deocdePos(trajectory.joint_names, point) 
        tar_joint=tuple(pos)   
        pos.append(int(run_time*40))                        #机械臂最后一个参数可以控制运动速度
        wt=float((abs(pos[-1]/1000)+1)*arm_wait_para)   
        rospy.loginfo("arm wait time %fs"%wt)
        pos = Int16MultiArray(data=pos)                     #转换消息格式
        self.arm_run(pos,tar_joint,wt)
        self.arm_JointState_update(trajectory.joint_names,point.positions)  #更新rviz虚拟机械臂位置
        self._as.set_succeeded(self._result)                #机械臂运动完成标志位

if __name__ == '__main__':
    rospy.init_node('arm-controller')               
    server = JointTrajectoryActionServer("arm_controller/follow_joint_trajectory")   #自定义类,启动回调函数
    def cmdfu(arg):                                         #机械臂取消动作回调函数
        rospy.logwarn("Console cancel command")
        this.cmd_cancel=1
    rospy.Subscriber('/arm_controller/follow_joint_trajectory/cancel',GoalID,cmdfu)    #订阅控制板命令,实现机械臂紧急停止
    
    def __arm_joint(msg):                                   #机械臂关节查询回调函数
        this.arm_joint=msg.data
        this.arm_res=1
    rospy.Subscriber('/xcar/arm_status', Int32MultiArray, __arm_joint)
    rospy.spin()

