#!/usr/bin/env python
import rospy
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


#Initializing the Turtlebot3 position and spawning them. Establishing publishing and subscribing ROS nodes. 
initial_position = (4,0.25,3.142)

print('Initial pose is:-')
print('(X, Y, Theta):', initial_position[0], initial_position[1], initial_position[2])
# pub.publish(Twist())
q = quaternion_from_euler(0, 0, initial_position[2])
# state_msg is an object
state_msg = ModelState()
state_msg.model_name = 'tb3_1'
state_msg.pose.position.x = initial_position[0]
state_msg.pose.position.y = initial_position[1]
state_msg.pose.position.z = 0


state_msg.pose.orientation.x = q[0]
state_msg.pose.orientation.y = q[1]
state_msg.pose.orientation.z = q[2]
state_msg.pose.orientation.w = q[3]
rospy.wait_for_service('/gazebo/set_model_state')
set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
resp = set_state(state_msg)
print(resp)

time.sleep(1)
# waypoints=[(3,0.29),(2.5,0.4),(2,0.75)]
waypoints = [(1,0.75)]


kp_distance = 1
ki_distance = 0.01
kd_distance = 0.5

kp_angle = 1
ki_angle = 0.03
kd_angle = 0.05

x = 0.0
y = 0.0 
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("speed_controller1")

sub = rospy.Subscriber("tb3_1/odom", Odometry, newOdom)
pub = rospy.Publisher("tb3_1/cmd_vel", Twist, queue_size = 1)

move_cmd = Twist()
r = rospy.Rate(20)

last_theta = 0
linear_speed = 1    #kp_distance
angular_speed = 1  #kp_angular

#Path Planning and Control Logic
while len(waypoints)!=0:
    (goal_x, goal_y) = (waypoints[0][0],waypoints[0][1])

    goal_distance = sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2))
    print(goal_distance)
    #distance is the error for length, x,y
    distance = goal_distance
    previous_distance = 0
    total_distance = 0

    previous_angle = 0
    total_angle = 0


    while distance > 0.2:
        x_start = x
        y_start = y
        path_angle = atan2(goal_y - y_start, goal_x- x_start)
        
        if path_angle < -pi/4 or path_angle > pi/4:
            if y_start < goal_y:
                path_angle = -2*pi + path_angle
            elif y_start > goal_y:
                path_angle = 2*pi + path_angle
        if last_theta > pi-0.1 and theta <= 0:
            theta = 2*pi + theta
        elif last_theta < -pi+0.1 and theta > 0:
            theta = -2*pi + theta


        #Distance PID
        distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
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
    waypoints.pop(0)
    print("reached :)")
    if len(waypoints)==0:
        pub.publish(Twist())






