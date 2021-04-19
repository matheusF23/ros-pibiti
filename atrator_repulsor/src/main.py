#!/usr/bin/env python2

import math
from random import random

import rospy as rp
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf

rp.init_node('atrator_repulsor')

vel = Twist()
scan = LaserScan()
quat = [0,0,0,0]
pub = None
sub = None
timer = None
angulo_robo = 0
sensores = []
angulos_sensores = [-100, -60, -30, 0, 30, 60, 100]
raio_robo = 0.25 # metro
x_alvo = 9
y_alvo = 1
x_robo = 0
y_robo = 0

print("Iniciando movimentacao")

def timerCallBack(event):
    kstoc = 0.1
    forca_repulsiva = desvia_obstaculo()
    forca_atrativa = alcanca_alvo()
    forca_estocastica = kstoc*(2*(random()-0.5))
    velocidade_angular = forca_repulsiva + forca_atrativa + forca_estocastica

    if abs(x_robo - x_alvo) <= 0.25 and abs(y_robo-y_alvo) <= 0.25:
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        print("Chguei ao destino")
    else:
        vel.linear.x = 0.2
        vel.angular.z = velocidade_angular

    pub.publish(vel)

def alcanca_alvo():
    """Retorna a contribuicao do atrator."""
    global angulo_robo
    angulo_alvo = math.atan((y_alvo-y_robo)/(x_alvo - x_robo))
    magnitude_forca_atracao = 1
    contribuicao_alvo = -magnitude_forca_atracao*math.sin(angulo_robo - angulo_alvo)
    return contribuicao_alvo

def desvia_obstaculo():
    """Retorna a contribuicao do repulsor."""
    global angulo_robo, sensores, raio_robo, angulos_sensores
    beta1 = 8
    beta2 = 20
    angulo_entre_sensores = 30*math.pi/180
    contribuicao_obstaculo = 0
    for sensor, distancia in enumerate(sensores):
        angulo_sensor = angulos_sensores[sensor]*math.pi/180
        angulo_obstaculo = angulo_robo + angulo_sensor
        faixa_angular_respulsiva = math.atan(math.tan(angulo_entre_sensores/2) + raio_robo/(raio_robo + distancia))
        contribuicao_obstaculo += beta1*math.exp(-distancia/beta2)*(angulo_robo-angulo_obstaculo)*math.exp((-(angulo_robo-angulo_obstaculo)**2)/(2*faixa_angular_respulsiva**2))
    return contribuicao_obstaculo

def scanCallBack(msg):
    global sensores
    sensores = []
    sensores.append(min(msg.ranges[0:100]))
    sensores.append(min(msg.ranges[100:200]))
    sensores.append(min(msg.ranges[200:300]))
    sensores.append(min(msg.ranges[300:400]))
    sensores.append(min(msg.ranges[400:500]))
    sensores.append(min(msg.ranges[500:600]))
    sensores.append(min(msg.ranges[600:]))

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
