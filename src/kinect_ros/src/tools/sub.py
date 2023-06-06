import os
import os.path as osp
import sys

cur_d = osp.dirname(__file__)
sys.path.append(osp.join(cur_d,'..'))

import json
import rospy
import cv2
import numpy as np
import open3d as o3d

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from cv_bridge import CvBridge
import message_filters
import time

from datetime import datetime
from PIL import Image as im

cv_bridge = CvBridge()

os.makedirs(osp.join(cur_d,"../debug"),exist_ok=True)
data_path = osp.join(cur_d,"../debug")

def now():
    return datetime.now().strftime("%y%m%dT%H%M%SS%f")[:-3]

time_stamp = now()

def write_infomation(kinect_image):
    
    os.makedirs(osp.join(data_path,time_stamp),exist_ok=True)
    
    cv2.imwrite(f"{osp.join(data_path,time_stamp)}/{now()}_.jpg",kinect_image)
    cv2.imwrite(f"{osp.join(data_path,time_stamp)}/{now()}_.bmp",hik_image)
    
#    time.sleep(0.5)

def write_json(info,info_name):
    
    with open(osp.join(data_path,info_name),'w') as f:
    	json.dump(info,f,indent=1)
 

def image_process(kin_image_msg,hik_image_msg):
    kinect_image = cv_bridge.imgmsg_to_cv2(kin_image_msg,desired_encoding="bgr8")
    hik_image = cv_bridge.imgmsg_to_cv2(hik_image_msg,desired_encoding="bgr8")

    write_infomation(kinect_image,hik_image)
    print("success")
    

def sub():
    rospy.init_node("sub",anonymous=True)
    
    hik_image_sub = message_filters.Subscriber("/image",Image)
    kinect_image_sub = message_filters.Subscriber("/rgb/image_raw",Image)
    ts = message_filters.ApproximateTimeSynchronizer([kinect_image_sub,hik_image_sub],10,0.1,allow_headerless=True)
    ts.registerCallback(image_process)
        
    rospy.spin()



if __name__ == '__main__':
    sub()
