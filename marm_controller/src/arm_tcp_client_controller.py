#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from actionlib import SimpleActionServer
from control_msgs.msg import FollowJointTrajectoryAction   
from control_msgs.msg import FollowJointTrajectoryResult
from std_msgs.msg import Int16MultiArray,Int16
from sensor_msgs.msg import JointState
from actionlib_msgs.msg import  GoalID
import socket  
import time
import yaml
import sys,signal,os
from marm_controller.srv import *
this = sys.modules[__name__]

this.config_path="/home/zonesion/catkin_ws/src/marm_controller/config/config.yaml"
with open(config_path, "r") as f:
    config = yaml.load(f.read(), Loader=yaml.FullLoader)

virtual_arm_joint_names=['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']
virtual_arm_home_point=(0, -0.7850, 1.57, 1.57, 0, 0)                               #home点
actually_joint_home_point=[ 0, -89, -89, -44, 0, 2000]                              #home点和速度参数
#小车服务器的IP地址
server_ipaddress='192.168.100.144'
server_port=9090
cmd_cancel=0
g_open=config["g_open"]

import threading
mutex = threading.Lock()

def quit(signum, frame):
    print('EXIT APP') 
    sys.exit()
    # rospy.signal_shutdown("arm is stopping")                  #发出机械臂停止运动信号

def cmdfu(arg):                         #机械臂取消动作回调函数
    rospy.logwarn("Console cancel command")
    global cmd_cancel
    cmd_cancel=1

def client_senddata(ty,data):        #客户端发送数据函数
    if ty==0:
        point_str =[str(i) for i in data] 
        point_str.insert(0,'arm')
        data=','.join(point_str)  
        client.send(data.encode())  
    if ty==1:
        data_list=[]
        data_list.append(data)
        data_str =[str(i) for i in data_list] 
        data_str.insert(0,'gripper')
        data=','.join(data_str) 
        client.send(data.encode())
        pass

def response():
    try:
        data=client.recv(100)           #从服务器接受响应帧.阻塞式等待数据
        data = data.decode()
        if data=="arm_response" :       #如果数据正确
            return 1
        elif data=="gripper_response" :
            return 2   
        elif data==None:                #服务器不存在，没有数据
            return -1
        else:
            return -1
    except (socket.timeout, socket.error, Exception) as e:  #数据超时,数据错误
        rospy.logerr(str(e))            #打印具体信息
        client.close()                  #关闭客户端
        rospy.logerr('client close Done') 
        os._exit(0)  

def gripper_control(data):                  #夹具控制
    mutex.acquire()
    client_senddata(1,data.data)                            # 发送控制数据
    code=response()                                         # 获取请求结果
    if code==2:
        rospy.logwarn("gripper go success!")
        mutex.release()
        return gripperResponse(gripperResponse.SUCCESS)     # 返回夹具控制成功
    else:
        rospy.logerr("gripper go fail!")
        mutex.release()
        return gripperResponse(gripperResponse.ERROR)

class JointTrajectoryActionServer(object):
    # create messages that are used to publish feedback/result
    _result = FollowJointTrajectoryResult()

    def __init__(self, name):
        self._action_name = name
        # self.ctlPub = rospy.Publisher('/xcar/arm', Int16MultiArray, queue_size=0, latch=True)    
        self.jstPub = rospy.Publisher('/xcar/arm_joint_states', JointState, queue_size=0, latch=True)
        self._as = SimpleActionServer(self._action_name, FollowJointTrajectoryAction,  
                                                execute_cb=self.execute_cb_ex, auto_start=False)  
        self._as.start()                            
        self.return_home() 
                              
    def arm_JointState_update(self,name,point):     
        jst = JointState()                          
        jst.name = name                             
        jst.position = point                        
        self.jstPub.publish(jst)        #发布理论机械臂的位置

    def return_home(self):                                          
        client_senddata(0,actually_joint_home_point)                              
        if(response()==1):              #服务器是否发送响应帧
            self.arm_JointState_update(virtual_arm_joint_names,virtual_arm_home_point)
        else:
            return
        client_senddata(1,g_open)  
        response()
            
    def deocdePos(self, name, msg):     #分析轨迹点,转换到真实机械臂动作
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
    
    def execute_cb(self, arg):                    
        trajectory = arg.trajectory   
        global cmd_cancel                   #取消动作命令
        cmd_cancel=0                        #默认为0
        st=0                                #轨迹点的起始运动时间
        for point in trajectory.points[1:]:    
            if cmd_cancel ==1:
                rospy.logwarn("The robots are stopping")
                return
            at = point.time_from_start.secs + point.time_from_start.nsecs / 1000000000.0
            delay = at - st                 #计算此次点的路径时间
            st = at                         #保存此次时间点
            pos = self.deocdePos(trajectory.joint_names, point) 
            pos.append(int(delay*1000))     #机械臂最后一个参数可以控制运动速度
            client_senddata(0,pos)                    
            delay = delay - 0.02
            if delay > 0:
                time.sleep(delay)
            if(response()==1):              #服务器响应
                self.arm_JointState_update(trajectory.joint_names,point.positions)
            else:
                return
        self._as.set_succeeded(self._result)        

    def execute_cb_ex(self, arg):                       #机械臂控制
        mutex.acquire()             
        trajectory = arg.trajectory   
        time=len(trajectory.points)
        point=trajectory.points[-1]
        pos = self.deocdePos(trajectory.joint_names, point) 
        pos.append(int(time*40))                        #机械臂最后一个参数可以控制运动速度
        client_senddata(0,pos) 
        if(response()==1):                              #服务器响应
            self.arm_JointState_update(trajectory.joint_names,point.positions)
        else:
            rospy.logerr("server connect error!")       #打印具体信息
            client.close()                              #关闭客户端
            rospy.logerr('client close Done') 
            os._exit(0)
            return
        rospy.logwarn("arm go success!")
        self._as.set_succeeded(self._result)  
        mutex.release()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, quit)                          
    signal.signal(signal.SIGTERM, quit)

    rospy.init_node('arm-controller')     
    # 相关的socket客户端配置
    client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    client.settimeout(15)                               #收发数据超时时间等等
    res=client.connect_ex((server_ipaddress,server_port))  
    if res==0:
        rospy.loginfo("xcar server connect ok")
    else:
        rospy.logerr("xcar server connect fail")
        sys.exit()
    # 相关服务
    server = JointTrajectoryActionServer("arm_controller/follow_joint_trajectory")   
    rospy.Subscriber('/arm_controller/follow_joint_trajectory/cancel',GoalID,cmdfu)         # 订阅控制板命令,实现机械臂紧急停止
    service_gripper = rospy.Service('/arm_controller/gripper', gripper, gripper_control)    # 建立服务 等待客户端进行连接
    rospy.spin()
