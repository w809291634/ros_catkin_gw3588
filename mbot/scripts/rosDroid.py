#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8MultiArray
from nav_msgs.msg import OccupancyGrid
from tf.msg import tfMessage
from nav_msgs.msg import Path
import tf
import sys
import threading
this = sys.modules[__name__]

tf_listener = None
def talker():
    global tf_listener
    pub = rospy.Publisher('/xcar/pos', String, queue_size=0)
    rate = rospy.Rate(3) # 10hz

    while not rospy.is_shutdown():
        try:
            t = rospy.Time()
            #tf_listener.clear()
            tf_listener.waitForTransform('/map', '/base_link', t, rospy.Duration(1))
            (trans, rot) = tf_listener.lookupTransform('/map', '/base_link', t)
            euler = tf.transformations.euler_from_quaternion(rot)
            #print trans, euler
            msg = "%f,%f,%f"%(trans[0],trans[1],euler[2])
            #rospy.loginfo("%s : %s"%(rospy.get_time(),msg))
            pub.publish(msg)
        except Exception as e:
            print(e) 
            rospy.loginfo("tf error map->base_link")
        rate.sleep()
        
      
        
        


from PIL import Image
import pngquant
import os
import struct
dirx=os.path.dirname(os.path.abspath(__file__))
os.system("chmod +x "+dirx+"/pngquant/pngquant-arm64")
quant = pngquant.PngQuant()
quant.config(dirx+'/pngquant/pngquant-arm64')

pubpng = rospy.Publisher('/png/map', UInt8MultiArray, queue_size=0,latch=True)

def mapCallback(msg):
    rospy.loginfo(rospy.get_caller_id() + 'I heard map')
    # create bmp
    img = Image.new("RGB", (msg.info.width,msg.info.height))
    dat = []
    for b in msg.data:
        if b == -1:
            b = 0x59<<16 | 0x59<<8 | 0x59
        else:
            b = (127-b)//127 * 200 
            b = b<<16 | b<<8 | b
        dat.append(b)
    img.putdata(dat)
    img.save("/tmp/.map.png","PNG")
    #img.show()
    c,dat = quant.quant_image("/tmp/.map.png", "/tmp/.c_map.png")
 
    x = UInt8MultiArray(data=dat) 
    print('pub png') 
    pubpng.publish(x)
    os.remove("/tmp/.map.png")   
    os.remove("/tmp/.c_map.png")

cmddir = dirx+"/../bin"
def userCmdCallback(msg):
    print('user cmd', msg) 
    cmd = cmddir + "/"+ msg.data +" &"
    
    os.system(cmd)

def path2mpath(p):
    
    global tf_listener
     
    #(trans, rot) = tf_listener.lookupTransform('/map', '/odom', rospy.Time(0))
    tf_listener.waitForTransform('/map', '/odom', rospy.Time(), rospy.Duration(1))
    p.header.frame_id = "map"
    poses = []
    for pos in p.poses:
        pos.header.stamp = rospy.Time()
        pos = tf_listener.transformPose("map", pos)
        poses.append(pos)
    p.poses = poses
    return p
import time
pubgpath = rospy.Publisher('/droid/gpath', Path, queue_size=0,latch=False)
gtime = time.time()
def gpath2mpath(p):
    global gtime
    if time.time() - gtime > 1:
        mp = path2mpath(p)
        pubgpath.publish(mp)
        gtime = time.time()
publpath = rospy.Publisher('/droid/lpath', Path, queue_size=0,latch=False)
ltime = time.time()
def lpath2mpath(p):
    global ltime
    if time.time() - ltime > 1:
        mp = path2mpath(p)
        publpath.publish(mp)
        ltime = time.time()

mMapMsg = None
mgPath = None
mlpath = None
evtPub = threading.Event()
def _mapCallback(msg):
    global mMapMsg
    global evtPub
   
    mMapMsg = msg
    evtPub.set()
    
def _gpath2mpathCallback(msg):
    global mgPath
    global evtPub
    
    mgPath = msg
    evtPub.set()
def _lpath2mpathCallback(msg):
    global mlpath
    global evtPub
    
    mlpath = msg
    evtPub.set()
    
def pubThread():
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        global mMapMsg
        global mgPath
        global mlpath
        global evtPub
        evtPub.clear()
        evtPub.wait()
        #print 'wake up'
        if mMapMsg != None:
            #print 'pub map'
            mapCallback(mMapMsg)
            mMapMsg = None
        if mgPath != None:
            #print 'pub global path'
            gpath2mpath(mgPath)
            mgPath = None
            
        if mlpath != None:
            #print 'pub local path'
            lpath2mpath(mlpath)
            mlpath = None
        #rate.sleep()
        
def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    rospy.Subscriber('/map', OccupancyGrid, _mapCallback)

    rospy.Subscriber('/user/cmd', String, userCmdCallback)
    
    rospy.Subscriber('/move_base/DWAPlannerROS/global_plan', Path, _gpath2mpathCallback, queue_size=1)
    rospy.Subscriber('/move_base/DWAPlannerROS/local_plan', Path, _lpath2mpathCallback, queue_size=1)
    rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', Path, _lpath2mpathCallback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
def udpListener():
    import socket
    udpsk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udpsk.bind(('0.0.0.0', 9998))
    print('udp start') 
    name = rospy.get_param("~name", "XCarRos")
    while True:
        data,addr = udpsk.recvfrom(64)
        udpsk.sendto(name, addr)
        


if __name__ == '__main__':

    rospy.init_node('Droid_Server', anonymous=True)

    thread = threading.Thread(target = udpListener)
    thread.setDaemon(True)
    thread.start()
    import fscan
    
    
    
    thread = threading.Thread(target = fscan.run)
    thread.setDaemon(True)
    thread.start()
    
    tf_listener = tf.TransformListener()
    
    thread = threading.Thread(target = pubThread)
    thread.setDaemon(True)
    thread.start()
    
    listener()
    talker()
    rospy.spin()
