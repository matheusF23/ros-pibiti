#!/usr/bin/env python2

import rospy as rp
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

rp.init_node('wall_follower')

vel = Twist()
scan = LaserScan()
pub = None
sub = None
timer = None
center = 0
left = 0
right = 0

def timerCallBack(event):
    if center > 1.0:
        vel.linear.x = 0.5
        if right > 0.5:
            vel.angular.z = -0.2
        else:
            vel.angular.z = 0.2
    else:
        vel.linear.x = 0
        if left > right:
           vel.angular.z = 0.2
        else:
            vel.angular.z = -0.2

    pub.publish(vel)

def scanCallBack(msg):
    global center, left, right
    center = min(msg.ranges[333:393])
    left = min(msg.ranges[500:560])
    right = min(msg.ranges[170:230])
    print(center, right, left)

pub = rp.Publisher('/p3dx/cmd_vel', Twist, queue_size=1)
sub = rp.Subscriber('/p3dx/laser/scan', LaserScan, scanCallBack)

# timer com 0.1s de periodo (10hz)
timer = rp.Timer(rp.Duration(0.05), timerCallBack)

rp.spin()
