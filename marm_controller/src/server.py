#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import sys
# reload(sys)
# sys.setdefaultencoding('utf8')
from std_msgs.msg import Int16MultiArray,Int32
import socket
import time
# 建立一个服务端

data="-90,-10.23,-89.9,-89.9,-44,-44.56"

point=[]
while data.find(',')>0:
    symbol=data.find(',')
    coord=float(data[0:symbol])    
    point.append(coord)
    data=data[symbol+1:]
point.append(float(data))
print(point)

server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1) 
server.bind(('192.168.100.175',9090)) #绑定要监听的端口
server.listen(1) #开始监听 表示可以使用五个链接排队\
while True:# conn就是客户端链接过来而在服务端为期生成的一个链接实例
    conn,addr = server.accept() #等待链接,多个链接的时候就会出现问题,其实返回了两个值
    print(conn,addr)
    while True:
            data = conn.recv(100)                   #接收从客户端来的数据
            if data:
                if data.find('arm')!=-1 and data.find('arm')==0:
                    print "arm"
                    symbol=data.find('arm')  
                    data=data[symbol+4:]
                    print data
                    point=[]                            
                    while data.find(',')>0:             #对接收的数据进行转换处理
                        symbol=data.find(',')           
                        coord=float(data[0:symbol])     
                        point.append(coord)             
                        data=data[symbol+1:]            
                    point.append(float(data))           
                    point=Int16MultiArray(data=point)   #转换消息格式
                    # ctlPub.publish(point)               #发布数据控制真实机械臂
                    print point
                    conn.send("arm_response")
                if  data.find('gripper')!=-1 and data.find('gripper')==0:
                    symbol=data.find('gripper')  
                    data=data[symbol+8:]                #去掉数据帧头
                    data=Int32(data)
                    # gripper_ctlPub.publish(Int32(data))
                    print data
                    conn.send("gripper_response")
            else :
                print("data error or client exit!")
                conn.close()                        
                break 