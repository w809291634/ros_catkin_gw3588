#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import cv2
import numpy as np
import os
from copy import deepcopy
import time

c_dir = os.path.split(os.path.realpath(__file__))[0]
pill_net = cv2.dnn.readNetFromTensorflow(c_dir+'/pill_detection_20200717.pb', c_dir+'/pill_detection_20200717.pbtxt')  #设置药盒模板
pill_class_names = ['box1', 'box2', 'box3', 'box4']                                                                     #药盒名称

traffic_net = cv2.dnn.readNetFromTensorflow(c_dir+'/traffic_20200706.pb', c_dir+'/traffic_20200706.pbtxt')
traffic_class_names = ['red', 'green', 'right', 'stop', 'left', 'straight', 'pedestrain', 'nolight']

def __check(img, net, class_name):
    #img = deepcopy(frame)
    im_width, im_height, img_channel = img.shape            #从cap.read()返回的image的数据中获取
    net.setInput(cv2.dnn.blobFromImage(img, size=(300, 300), swapRB=True, crop=False))      #设置神经网络图像输入
    cvOut = net.forward()                                   #向前运作
    
    rets = []                                               #初始化三个列表
    types = []
    pp = []
    
    for detection in cvOut[0, 0, :, :]:                     #进行目标检测, #取四维数组中的后面两维
        """
        detection输出结果为7维数据表示[0, 对应lable id, 置信度, 最左, 最上, 最右, 最下]
        最左与最上组成左上坐标(x1,y1)  最右与最下组成右下角坐标[x2,y2]
        """
        score = float(detection[2])
        if score > 0.5:
            label = int(detection[1])                   #取出检测物体的lable号
   
            left = int(detection[3] * im_height)        #检测的物体方位,(left,top)为左上角坐标
            top = int(detection[4] * im_width)
            right = int(detection[5] * im_height)       #检测的物体方位,(right,bottom)为右下角坐标
            bottom = int(detection[6] * im_width)

            cv2.rectangle(img, (int(left), int(top)), (int(right), int(bottom)), (0, 0, 255), thickness=2)  #处理图像,并画矩形
            cv2.putText(img, class_name[label - 1] + ': {:.2f}'.format(score), (int(left), int(top)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 0, 255), thickness=2)       #在图像中加入文本,并显示分值

            rets.append((left,top,right-left,bottom-top)) #返回左上角的坐标,和物体的宽度\高度
            types.append(class_name[label - 1])         #检测物体的标签名称
            pp.append(score)                            #物体的分数
  
    return img, rets, types, pp #img:   处理后的图像
                                #rets:  返回左上角的坐标,和物体的宽度\高度
                                #types: 检测物体的标签名称
                                #pp:    物体的分数

def pilldetect(frame):          #药盒检测，frame为从摄像头读取,img就是每一帧的图像，是个三维矩阵
    img, rect, types, pp = __check(frame, pill_net, pill_class_names)   #进入检查药盒的函数,参数为cap.read()返回的image,其他为模板和名称
    #img:   处理后的图像
    #rect:  返回左上角的坐标,和物体的宽度\高度
    #types: 检测物体的标签名称
    #pp:    物体的分数                                                             #
    maxidx = 0
    maxval = 0
    ret = []
    _type = ""
    for i in range(len(pp)):
        if pp[i] > maxval:
            maxidx = i
            maxval = pp[i]      #取最大的分数
            ret = rect[i]       #ret,返回左上角的坐标,和物体的宽度\高度
            _type = types[i]
    return img, ret, _type

def trafficdetect(fram):                #交通标志检测
    img, rect, types, pp = __check(frame, traffic_net, traffic_class_names)
    return img, rect, types, pp
    
    
if __name__ == '__main__':
    cap = cv2.VideoCapture(2)
    _, first_img = cap.read()
    rows, cols, _ = first_img.shape
    print(rows, cols, _ )
    while True:
        ret, img = cap.read()       #从摄像头读取,img就是每一帧的图像，是个三维矩阵
        if ret is None:
            break
        img, rets, _type = pilldetect(img)   #从视频中检测药盒,img作为pilldetect函数的frame参数
        #img:   处理后的图像
        #rets:  返回左上角的坐标,和物体的宽度\高度
        #_type: 检测物体的标签名称,分数最大的一个
        cv2.imshow('img', img)              #显示处理的图像
        key = cv2.waitKey(1) & 0xFF         #退出等等

        if key == ord('q'):
            break
