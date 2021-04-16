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
angulo_robo = 0
sensores = []
x_alvo = 3
y_alvo = 8
x_robo = 0
y_robo = 0
 

def timerCallBack(event):
    velocidade_angular = desvia_obstaculo() + alcanca_alvo()

    if abs(x_robo - x_alvo) <= 1 and abs(y_robo-y_alvo) <= 1:
        vel.linear.x = 0.0
        vel.angular.z = 0.0
    else:
        vel.linear.x = 0.5
        vel.angular.z = velocidade_angular
    pub.publish(vel)

def alcanca_alvo():
    """Retorna a contribuicao do atrator."""
    global angulo_robo
    angulo_alvo = math.atan((y_alvo-y_robo)/(x_alvo - x_robo))
    magnitude_forca_atracao = 10
    contribuicao_alvo = -magnitude_forca_atracao*math.sin(angulo_robo - angulo_alvo)
    return contribuicao_alvo

def desvia_obstaculo():
    """Retorna a contribuicao do repulsor."""
    global angulo_robo, sensores
    beta1 = 0.05
    beta2 = 20
    angulo_entre_sensores = 0.00576969701797
    contribuicao_obstaculo = 0
    for sensor, distancia in enumerate(sensores):
        angulo_sensor = (sensor - 363)*angulo_entre_sensores
        angulo_obstaculo = angulo_robo - angulo_sensor
        contribuicao_obstaculo += beta1*math.exp(-distancia/beta2)*(angulo_robo-angulo_obstaculo)*math.exp((-(angulo_robo-angulo_obstaculo)**2)/2)
    return contribuicao_obstaculo

def scanCallBack(msg):
    global sensores
    sensores = msg.ranges

def angulo_roboCallBack(msg):
    global quat, x_robo, y_robo, angulo_robo
    x_robo = msg.pose.pose.position.x
    y_robo = msg.pose.pose.position.y
    quaternion = msg.pose.pose.orientation
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    euler = tf.transformations.euler_from_quaternion(quat)
    angulo_robo = euler[2] # angulo do robo em radianos

pub = rp.Publisher('/p3dx/cmd_vel', Twist, queue_size=1)
sub = rp.Subscriber('/p3dx/laser/scan', LaserScan, scanCallBack)
sub = rp.Subscriber('/p3dx/odom', Odometry, angulo_roboCallBack)

# timer com 0.1s de periodo (10hz)
timer = rp.Timer(rp.Duration(0.05), timerCallBack)

rp.spin()
