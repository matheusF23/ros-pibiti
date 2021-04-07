#!/usr/bin/env python2

import rospy as rp
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry
import tf
import math

rp.init_node('atrator_repulsor')

vel = Twist()
scan = LaserScan()
quat = [0,0,0,0]
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
    print(len(msg.ranges))
    # print(center, right, left)

def QuatCallBack(msg):
    global quat
    quaternion = msg.pose.pose.orientation
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    # rp.loginfo(180*euler[2]/math.pi) # angulo em graus

pub = rp.Publisher('/p3dx/cmd_vel', Twist, queue_size=1)
sub = rp.Subscriber('/p3dx/laser/scan', LaserScan, scanCallBack)
sub = rp.Subscriber('/p3dx/odom', Odometry, QuatCallBack)

# timer com 0.1s de periodo (10hz)
timer = rp.Timer(rp.Duration(0.05), timerCallBack)

rp.spin()
