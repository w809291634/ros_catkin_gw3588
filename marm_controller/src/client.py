#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import sys
# reload(sys)
# sys.setdefaultencoding('utf8')/
import socket# 客户端 发送一个数据，再接收一个数据
import time
client = socket.socket(socket.AF_INET,socket.SOCK_STREAM) #声明socket类型，同时生成链接对象
client.connect(('192.168.100.175',9090)) #建立一个链接，连接到本地的6969端口
print("xcar server connect ok")
while True:
    # addr = client.accept()
    # print '连接地址：', addr
    msg = 'wanghao,wanghao,wanghao'  #strip默认取出字符串的头尾空格
    client.send(msg)  #发送一条信息 python3 只接收btye流
    time.sleep(1)

client.close() #关闭这个链接