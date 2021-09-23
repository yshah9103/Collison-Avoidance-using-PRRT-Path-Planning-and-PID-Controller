#!/usr/bin/env python
import rospy
from numpy import random
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist, Quaternion
from math import atan2
import os
import time
import math 
from tf.transformations import quaternion_from_euler
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
import numpy as np
import subprocess
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from func import getnewnode, dis, angle, potential, dynamic_potential, getnewpoint, gains, RRT, respawn_tb, forplot, plotpot


#Initializing the Turtlebot3 position and spawning them. Establishing publishing and subscribing ROS nodes. 
respawn_tb()
# waypoints=[(2,0),(5,0.4),(8,0.3)]

kp_distance,ki_distance,kd_distance,kp_angle,ki_angle,kd_angle,x,y,theta,r1,q1,theta1 = gains()

def newOdom(msg):
    global x
    global y
    global theta
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def newOdom1(msg1):
    global p1
    global q1
    global theta1
    p1 = msg1.pose.pose.position.x
    q1 = msg1.pose.pose.position.y
    rot_q1 = msg1.pose.pose.orientation
    (roll1, pitch1, theta1) = euler_from_quaternion([rot_q1.x, rot_q1.y, rot_q1.z, rot_q1.w])


rospy.init_node("speed_controller")

sub = rospy.Subscriber("tb3_0/odom", Odometry, newOdom)
sub1 = rospy.Subscriber("tb3_1/odom", Odometry, newOdom1)
pub = rospy.Publisher("tb3_0/cmd_vel", Twist, queue_size = 1)

move_cmd = Twist()
r = rospy.Rate(20)

last_theta = 0
linear_speed = 1    #kp_distance
angular_speed = 1  #kp_angular

waypoints = [(4,0.75)]
condition = True

#Path Planning and Control Logic
while len(waypoints)!=0:
    (goal_x, goal_y) = (waypoints[0][0],waypoints[0][1])
    print("newgoal")
    print(goal_x,goal_y)
    print("current")
    print(x,y)
    goal_distance = sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2))
    distance = goal_distance
    previous_distance = 0
    total_distance = 0

    previous_angle = 0
    total_angle = 0

    newx,newy=forplot()

    potential1 = np.loadtxt("/home/yash/catkin_ws/src/control/src/test.txt").reshape(1001, 101)
    potential1 = potential1.astype(int)

    while distance > 0.075:
        # print("insidewhile")
        global potential2
        potentialinside = potential1
        # print("start")
        potential2=dynamic_potential(potentialinside,p1,q1)
        # print("end")
        distancebet=dis((x,y),(p1,q1))
        # print(distancebet)
        if distancebet<1.2 and p1>x-0.1 and condition==True:
            print("RRT TRIGGERED")
            # print(potential2)
            waypoints=RRT(potential2,x,y)
            (goal_x, goal_y) = (waypoints[0][0],waypoints[0][1])
            # print("waypoints from func",waypoints)
            condition=False
            aa=True
            break  

        # waypoints=RRT(potential2,x,y)
        potential2=np.concatenate(potential2)
        plotpot(newx, newy, potential2)

        x_start = x
        y_start = y
        path_angle = atan2(goal_y - y_start, goal_x- x_start)
        
        if path_angle < -pi/2 or path_angle > pi/2:
            # print("triggered")
            if y_start < goal_y:
                # print("a")
                path_angle = -2*pi + path_angle
            elif y_start > goal_y:
                # print("b")
                path_angle = 2*pi + path_angle
        if last_theta > pi-0.1 and theta <= 0:
            theta = 2*pi + theta
            # print("ddd")
        elif last_theta < -pi+0.1 and theta > 0:
            # print("nnn")
            theta = -2*pi + theta

        #Distance PID
        distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
        if distance>0.075:
            aa=True
        else:
            aa=False 
        diff_distance = distance - previous_distance
        control_signal_distance = kp_distance*distance + ki_distance*total_distance + kd_distance*diff_distance

        #Angle PID
        diff_angle = path_angle - previous_angle
        # print("diff_angle",diff_angle)
        control_signal_angle = kp_angle*path_angle + kd_angle*diff_angle
        move_cmd.angular.z = (control_signal_angle) - theta
        if goal_y < y_start and goal_x < x_start:
            move_cmd.angular.z = -(control_signal_angle) - theta
        move_cmd.linear.x = min(control_signal_distance, 0.075)
        if move_cmd.angular.z > 0:
            move_cmd.angular.z = min(move_cmd.angular.z, 0.75)
        else:
            move_cmd.angular.z = max(move_cmd.angular.z, -0.75)

        last_theta = theta
        pub.publish(move_cmd)
        r.sleep()
        previous_distance = distance
        total_angle = total_angle + path_angle
        previous_angle = path_angle
        total_distance = total_distance + distance
        # print("Current positin and theta are: ", ((x,y), theta))
        # print("theta",theta)
    if condition==False and aa==True:
        continue
    else: 
        print("reached :)")
        waypoints.pop(0)
        condition=True
        if len(waypoints)==0:
            pub.publish(Twist())

