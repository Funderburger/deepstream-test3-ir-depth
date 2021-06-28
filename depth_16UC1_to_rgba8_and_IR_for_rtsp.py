#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge 
import cv2

import numpy as np
import time
import sys
import message_filters
import std_msgs.msg

ir_pub = None
depth_pub = None

bridge = CvBridge()

def callback():
    print("ceva")

def callback_sync(ir_msg, depth_msg):#, info_depth_msg):
    global ir_pub
    global depth_pub

    frame = bridge.imgmsg_to_cv2(depth_msg,desired_encoding="passthrough")

    height = frame.shape[0]
    width = frame.shape[1]
    rgba_img = np.zeros((height,width,4), np.uint8)
    rgba_img[:,:,:] = [0,0,0,255]
    
    valid = frame != 0                                                                                                                  
    thousands_digits = frame/1000
    thousands = thousands_digits * 1000
    tens_digits = (frame - thousands) / 10
    units_digits = frame - thousands - tens_digits*10 

    # rgba_img[:,:,0] = np.where(valid,thousands_digits,0)
    # rgba_img[:,:,1] = np.where(valid,tens_digits,0)
    # rgba_img[:,:,2] = np.where(valid,units_digits,0)

    rgba_img[:,:,0] = 7#np.where(valid,7,0)
    rgba_img[:,:,1] = 91#np.where(valid,91,0)
    rgba_img[:,:,2] = 9#np.where(valid,9,0)

    rgba_msg = bridge.cv2_to_imgmsg(rgba_img,"rgba8")
    rgba_msg.header = ir_msg.header

    depth_pub.publish(rgba_msg)
    ir_pub.publish(ir_msg)


def start_node():
    global depth_pub
    global ir_pub

    rospy.init_node('convert_16UC1_to_RGBA8')
    rospy.loginfo('Depth conversion and sync with IR started')

    ir_sub = message_filters.Subscriber("/pico_img_rect_v2_ok", Image)
    depth_sub = message_filters.Subscriber("/pico_zense/depth/image_raw", Image)
    info_depth_sub = message_filters.Subscriber("/pico_zense/depth/camera_info", CameraInfo)
    
    ir_pub = rospy.Publisher('/pico_IR_sync', Image, queue_size=10)
    depth_pub = rospy.Publisher('/pico_rgba8_depth_sync', Image, queue_size=10)
    
    ts = message_filters.ApproximateTimeSynchronizer([ir_sub,depth_sub], 10, 60, allow_headerless=False)
    ts.registerCallback(callback_sync)
    

if __name__ == '__main__':
    try:
        start_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass