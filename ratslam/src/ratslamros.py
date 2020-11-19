#!/usr/bin/env python2

import rospy as rp
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ratslammodule3.main3 import plotResult
from ratslammodule3.MAP_RatSlamModule import ratslam3
from ratslammodule3.MAP_RatSlamModule import _globals3 as gb

rp.init_node('ratslam')

img = Image()
bridge = CvBridge()

# def timerCallBack(event):
#     cv_image = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
#     print("ok")
#     main3.main3(cv_image)

def imgCallBack(msg):
    global img
    img = msg
    frame = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough') 
    print("ok")
    # main3.main3(cv_image)

    slam = ratslam3.Ratslam()
    
    loop = 0
    # _, frame = video.read()

    while True:
        
        loop += 1

        # flag,frame = video.read()
        if frame is None:
            break

        #width  = video.get( cv2.cv.CV_CAP_PROP_FRAME_WIDTH  ) # Change Paulo
        # width  = video.get( cv2.CAP_PROP_FRAME_WIDTH ) 
        # height = video.get( cv2.CAP_PROP_FRAME_HEIGHT )

        width  = frame.shape[1]
        height = frame.shape[0]


        # Changing values in _globals variables as example. The globals file
        # set variables from pose cells, local view cells, visual odometry...  
        gb.IMAGE_WIDTH = width
        gb.IMAGE_HEIGHT = height

        img = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )
        img = np.array( img )

        if loop%7==0:
            
            # The True setting is refers to pass a GRAY image. 
            slam(img, True)

            plotResult(frame, slam)
            break


sub = rp.Subscriber('/camera/rgb/image_raw', Image, imgCallBack)

# timer = rp.Timer(rp.Duration(0.05), timerCallBack)

rp.spin()
