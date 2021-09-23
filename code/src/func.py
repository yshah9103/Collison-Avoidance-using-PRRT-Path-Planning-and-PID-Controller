from numpy import random
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
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

#function for RRT path planning
def getnewpoint(a,b):
    while True:
        p1rows=random.uniform(a+0.14,a+0.15)
        p1cols=random.uniform(b-0.06,b+0.06)
        p1rows=round(p1rows,2)
        p1cols=round(p1cols,2)
        p=(p1rows,p1cols)
    # print("Newpoint:",p)
        if p[0] > 10 or p[0] <0 or p[1] > 1 or p[1] < 0:
            continue
        else:
            return p

#function for RRT path planning
def getnewnode(p,vertices):
    dist_new_node=[]
    minindex=0
    for i in range(len(vertices)):
        distance=dis(vertices[i],p)           # calculate distance of new point with respect to all nodes and append in dist_new_node
        dist_new_node.append(distance)
    
    minindex=dist_new_node.index(min(dist_new_node))        # find index value of minimum distance to node and return node 
    return vertices[minindex]    

def dis(p1,p2):
    distance=math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)             # Calculating euclidean distance
    return distance


#function for RRT path planning
def angle(p1,p2):                          # using angle components to find new node in direction of point from nearest node
    step=0.2
    distance_1=dis(p1,p2)
    if distance_1 > step:
        distance_1= step
    
    theta= math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    ii=p1[0]+distance_1*math.cos(theta)
    yy=p1[1]+distance_1*math.sin(theta)
    ii=round(ii,2)
    yy=round(yy,2)
    new_node1=(ii,yy)
    return new_node1

#Initializing potential for static objects in a scene (lane, road barrier)
def potential():
    x1=np.linspace(0,10,1000)
    x1=np.round(x1,2)
    y1=np.size(x1)*[0]
    lane1=list(zip(x1,y1))

    x2=np.linspace(0,10,1000)
    x2=np.round(x2,2)
    y2=np.size(x2)*[0.5]
    mlane=list(zip(x2,y2))

    x3=np.linspace(0,10,1000)
    x3=np.round(x3,2)
    y3=np.size(x3)*[1]
    lane2=list(zip(x3,y3))

    lane1.extend(mlane)
    lane1.extend(lane2)
    obstacles = lane1
    potential=[[0 for i in range (101)] for j in range (1001)]

    # print(potential)
    for i in range(1001):
        # print(i)
        for j in range(101):
            for k in range(len(obstacles)):    
                distance=math.sqrt((((i/100))-obstacles[k][0])**2+(((j/100))-obstacles[k][1])**2)
                if distance<0.05:
                    # print(i,j)
                    potential[i][j]=9
                elif distance>=0.05 and distance<=0.1:
                    if potential[i][j]==9:
                        continue
                    else:
                        potential[i][j]=5


    for i in range(1001):
        potential[i][4]=1
        potential[i][5]=2
        potential[i][6]=1
    return potential

# Defining dynamic potential of moving objects in a scene
def dynamic_potential(potential,p1,q1):
    
    p1 = np.round(p1,2)
    q1 = np.round(q1,2)
    p1 = p1*100
    q1 = q1*100
    p1 = int(p1)
    q1 = int(q1)
    #potential field just around the turtlebot3
    xr=list(range(-40,41))
    yr=list(range(-30,31))

    
    for i in range(len(xr)):
        for j in range(len(yr)):
            if p1+xr[i]>1000 or p1+xr[i]<0 or q1+yr[j]>100 or q1+yr[j]<0:
                continue
            potential[p1+xr[i]][q1+yr[j]]=15
    
    #potential field in front of the turtlebot3

    xr = [-5,-6,-7,-8,-9,-10,-11]
    xr = list(range(-130,-40))
    xr = xr+list(range(41,100))
    for i in range(len(xr)):
        for j in range(len(yr)):
            if p1+xr[i]>1000 or p1+xr[i]<0 or q1+yr[j]>100 or q1+yr[j]<0:
                continue
            c=max(abs(xr[i]),abs(yr[j]))
            potential[p1+xr[i]][q1+yr[j]]=19-c/10 #starting from 19 because lowest value of c is going to be 5
    
    return potential


#PID controller gains
def gains():
    kp_distance = 1
    ki_distance = 0.01
    kd_distance = 0.5

    kp_angle = 1
    ki_angle = 0.03
    kd_angle = 0.05

    x = 0.0
    y = 0.0 
    theta = 0.0

    p1 = 0.0
    q1= 0.0 
    theta1 = 0.0
    return kp_distance,ki_distance,kd_distance,kp_angle,ki_angle,kd_angle,x,y,theta,p1,q1,theta1   

# Path planning for the turtlebot3
def RRT(potential2,x,y):
    a=[]
    cost=[]
    minindex=0
    while len(a)!=15:
        vertices=[(x,y)]
        verticesfornewnode=[(x,y)]
        value=0
        while len(vertices)!=6:
            p=getnewpoint(verticesfornewnode[0][0], verticesfornewnode[0][1])
            verticesfornewnode.pop(0)
            verticesfornewnode.append(p)
            # value=value+potential2[int(newnode[0]*10)][int(newnode[1]*10)]
            value=value+potential2[int(p[0]*100)][int(p[1]*100)]
            vertices.append(p)
        a.append(vertices)
        cost.append(value)
    minindex=cost.index(min(cost))
    # print("minindex",minindex)
    b=a[minindex]

    waypoints=[b[2],(4,0.75)]
    return waypoints

# def RRT(potential2,x,y):

def respawn_tb():
    initial_position = (1,0.75,0)
    q = quaternion_from_euler(0, 0, initial_position[2])
    # state_msg is an object
    state_msg = ModelState()
    state_msg.model_name = 'tb3_0'
    state_msg.pose.position.x = initial_position[0]
    state_msg.pose.position.y = initial_position[1]
    state_msg.pose.position.z = 0

    state_msg.pose.orientation.x = q[0]
    state_msg.pose.orientation.y = q[1]
    state_msg.pose.orientation.z = q[2]
    state_msg.pose.orientation.w = q[3]

    # service = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    # response = service
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    resp = set_state(state_msg)
    print(resp)
    time.sleep(1)


def forplot():
    newy=[]
    for i in range(1001):
        xforplot=np.linspace(0,1,101)
        xforplot=np.round(xforplot,2)
        newy.append(xforplot)

    newy=np.concatenate(newy)
    # print(len(newy))

    newx=[]
    for i in range(1001):
        if i ==0:
            yforplot=[0]*101
            newx.append(yforplot)
        else:
            factor = 0 + i/100
            yforplot=[factor]*101
            newx.append(yforplot)

    newx=np.concatenate(newx)
    return newx,newy

def plotpot(newx,newy,potential2):
    ax = plt.axes(projection='3d')
    # ax.plot3D(newx, newy, potential2, 'gray')
    ax.scatter3D(newx, newy, potential2, c=potential2, cmap='Greens')
    # plt.draw()
    # plt.pause(0.001)
    plt.show()