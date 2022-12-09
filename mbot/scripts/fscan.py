#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
import copy
from threading import Event, Lock

 
laserData = []
evtScan = Event()

def laserCb(msg):
    global laserData
    global evtScan
    global lock
    m = copy.deepcopy(msg)
    if len(laserData)<1:
        laserData.append(m)
        evtScan.set()
    #franges = list(msg.ranges)
    #franges = []
    #ranges = msg.ranges
    #for i in range(len(ranges)-1) :
    #    franges.append(ranges[i])

def proc():
    global laserData
    global evtScan
    pub  = rospy.Publisher('/scan_f', LaserScan, queue_size=0,latch=False)
    while True:
        evtScan.wait()
        if len(laserData)>0:
            msg = laserData[0]
            del laserData[0]
         
            dr = []
            try:
                franges = list(msg.ranges)
                ranges = msg.ranges
                for i in range(len(ranges)-1) :
                    dr.append(abs(ranges[i+1] - ranges[i]))
                dr.append(abs(ranges[-1] - ranges[0]))
                num = 0
                start_i = 0
                dx = franges[0]
                if dx > 1:
                    dx = 1 
                if dx < 0.2:
                    dx = 0.2
                for i in range(len(dr)):
                    if dr[i] < 0.05:
                        num += 1
                    else:
                        if num < (2.5-dx)*5: #小于5个点清除
                            for j in range(start_i, i):
                                franges[j] = 5  #'inf')
                            franges[start_i] = 5 #float('inf')
                            franges[i] = 5 #float('inf')
                        num = 0
                        start_i = i
                        dx = franges[i]
                        if dx > 1: 
                            dx = 1 
                        if dx < 0.2:
                            dx = 0.2
                msg.ranges = franges
                #msg.header.stamp = rospy.Time()
                pub.publish(msg)
            except:
                pass
        

def run():
    print('register scan') 
    rospy.Subscriber('/scan', LaserScan,  laserCb)
    proc()
    
