#!/usr/bin/env python2

import rospy as rp
import cv2
import numpy as np

from ratslammodule3.MAP_RatSlamModule import ratslam3
from ratslammodule3.MAP_RatSlamModule import _globals3 as gb
from ratslammodule3.main3 import plotResult

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ratslammodule3 import main3

rp.init_node('ratslam')

img = Image()
bridge = CvBridge()
frame = None

# def timerCallBack(event):
#     frame = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
#     print("ok")
#     main3.main3(frame)

def img_callback(msg):
    global img
    global frame
    img = msg
    frame = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
    print("ok")


sub = rp.Subscriber('/camera/rgb/image_raw', Image, img_callback)

# timer = rp.Timer(rp.Duration(0.05), timerCallBack)



cv2.waitKey(1000)
slam = ratslam3.Ratslam()

loop = 0

while True:
    
    loop += 1

    if frame is not None:
        width  = frame.shape[1]
        height = frame.shape[0]

        gb.IMAGE_WIDTH = width
        gb.IMAGE_HEIGHT = height

        img = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )
        img = np.array( img )

        if loop%7==0:
            
            # The True setting is refers to pass a GRAY image. 
            slam(img, True)

            plotResult(frame, slam)

rp.spin()
