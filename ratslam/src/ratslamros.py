#!/usr/bin/env python2

import rospy as rp

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ratslammodule3 import main3

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
    cv_image = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough') 
    print("ok")
    main3.main3(cv_image)


sub = rp.Subscriber('/camera/rgb/image_raw', Image, imgCallBack)

# timer = rp.Timer(rp.Duration(0.05), timerCallBack)

rp.spin()
