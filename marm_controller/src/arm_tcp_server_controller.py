#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import Int16MultiArray,Int32,Int32MultiArray
import sys
import socket
import time
import yaml

this = sys.modules[__name__]
#小车服务器的IP地址
server_ipaddress='192.168.100.144'                      #IP地址根据实际情况设定
server_port=9090
this.config_path="/home/zonesion/catkin_ws/src/marm_controller/config/config.yaml"
with open(config_path, "r") as f:
    config = yaml.load(f.read(), Loader=yaml.FullLoader)

g_open=config["g_open"]
arm_query_time=config["arm_query_time"]     #机械臂查询关节位置周期
arm_wait_para=config["arm_wait_para"]      #机械臂运动超时时间系数
arm_arrive_err=config["arm_arrive_err"]

grip_query_time=config["grip_query_time"]       #机械臂夹具查询关节位置周期
grip_wait_time=config["grip_wait_time"]         #机械臂夹具运动超时时间系数
gripper_arrive_err=config["gripper_arrive_err"] 

this.arm_joint=[]
this.arm_res=0

def arm_arrive(joint1,joint2,err=10):
    if len(joint1)>0 and len(joint2)>0:
        if abs(joint1[0]-joint2[0])<err and abs(joint1[1]-joint2[1])<err and abs(joint1[2]-joint2[2])<err and \
        abs(joint1[3]-joint2[3])<err and abs(joint1[4]-joint2[4])<err:
            return True
        else:
            return False
    else:
        return False    

def gripper_arrive(data1,data2,err=10):
    if abs(data1-data2)<err:
        return True
    else:
        return False     
        
if __name__=='__main__':
    rospy.init_node('arm_tcp_controller')               
    arm_ctlPub=rospy.Publisher('/xcar/arm', Int16MultiArray, queue_size=0, latch=True)
    gripper_ctlPub=rospy.Publisher('/xcar/gripper', Int32, queue_size=0, latch=True)
    arm_status_pub=rospy.Publisher('/xcar/arm_status_req', Int32, queue_size=0, latch=True)
    def __arm_joint(msg):
        this.arm_joint=msg.data
        this.arm_res=1
    rospy.Subscriber('/xcar/arm_status', Int32MultiArray, __arm_joint)
    
    server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1) 
    server.bind((server_ipaddress,server_port))        
    server.listen(1)                                    
    while True:                                         
        rospy.loginfo("The server is ready and waiting for connection")
        try:
            conn,addr = server.accept()                     
            print(conn,addr)
        except socket.timeout as e:
            rospy.logerr(e)  
            continue
        while True:
                try:
                    data = conn.recv(100)                   #接收从客户端来的数据
                    data = data.decode()
                except socket.timeout as e:
                    rospy.logwarn(str(e)+',retry')  
                    continue
                if data:
                    if data.find('arm')!=-1 and data.find('arm')==0:    # 动作机械臂的消息
                        symbol=data.find('arm')  
                        data=data[symbol+4:]
                        print('data',data)
                        point=[]                            
                        while data.find(',')>0:                     # 对接收的数据进行转换处理
                            symbol=data.find(',')  
                            coord=int(data[0:symbol])               # 
                            point.append(coord)             
                            data=data[symbol+1:]                               
                        point.append(int(data))  
                        
                        wt=float((abs(point[-1]/1000)+1)*arm_wait_para)   
                        rospy.loginfo("arm wait time %fs"%wt)
                        point_tuple=tuple(point[0:-1])   
                        point=Int16MultiArray(data=point)           #转换消息格式
                        st1=time.time()
                        while True:
                            arm_ctlPub.publish(point)               #发布数据控制真实机械臂    
                            arm_res=0
                            arm_status_pub.publish(Int32(1))
                            time.sleep(arm_query_time)
                            st=time.time()
                            while not this.arm_res:
                                time.sleep(0.02)
                                if time.time()-st>arm_query_time:          # 等待
                                    print("no arm_res")
                                    break 
                            if this.arm_res==1:
                                # print("arm_1",point_tuple)
                                # print("arm_2",arm_joint[1:])
                                if arm_arrive(point_tuple,arm_joint[1:],arm_arrive_err)==True:
                                    rospy.loginfo("arm run success")
                                    break
                            if time.time()-st1>wt:               # 等待
                                # print("arm_1",point_tuple)
                                # print("arm_2",arm_joint[1:])
                                rospy.logerr("arm run error!")
                                break   
                        conn.send("arm_response".encode())

                    elif data.find('gripper')!=-1 and data.find('gripper')==0:  # 动作夹具的消息
                        symbol=data.find('gripper')  
                        data=int(data[symbol+8:])               #去掉数据帧头
                        st1=time.time()
                        while True:
                            gripper_ctlPub.publish(Int32(data))              
                            this.arm_res=0
                            arm_status_pub.publish(Int32(1))
                            time.sleep(grip_query_time)
                            st=time.time()
                            while not this.arm_res:
                                time.sleep(0.02)
                                if time.time()-st>grip_query_time:          # 等待
                                    print("no arm_res")
                                    break 
                            if this.arm_res==1:
                                # print("g_1",data)
                                # print("g_2",arm_joint[0])
                                if gripper_arrive(data,arm_joint[0],gripper_arrive_err)==True:
                                    rospy.loginfo("gripper run success")
                                    break
                            if time.time()-st1>grip_wait_time:               # 等待
                                rospy.logerr("gripper run error!")
                                print("g_1",data)
                                print("g_2",arm_joint[0])
                                break 
                        conn.send("gripper_response".encode())
                else :
                    print("data error or client exit!")
                    conn.close()                        
                    break                               

