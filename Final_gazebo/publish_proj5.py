#! /usr/bin/env python
from re import X
import rospy
import math
from geometry_msgs.msg import Twist
import time
import numpy as np
#opening the saved nodes
nodes1=[]
nodes2 =[]
f1 = open('/home/sharmitha/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/path1.txt', 'r')

lines1=f1.readline()
while lines1:
    nodes1.append(int(lines1))
    lines1=f1.readline()
f1.close()
f2 = open('/home/sharmitha/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/path2.txt', 'r')

lines2=f2.readline()
while lines2:
    nodes2.append(int(lines2))
    lines2=f2.readline()
f2.close()

def Eucledian(a, b):
    
    x1 = a[0]
    x2 = b[0]
    y1 = a[1]
    y2 = b[1]
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
   
    dist = round(dist)
    return dist

def test(nodes1,nodes2):
    msg1=Twist()
    msg2 = Twist()
    rospy.init_node('robot_talker',anonymous=True)
    
    pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    
    theta_curr = 0
   
    for i in range(len(nodes1)-1):
        print(nodes1[i],nodes2[i])	
        dist = Eucledian((nodes1[i],nodes2[i]),(nodes1[i+1],nodes2[i+1]))
        t = dist/10
        print('t',t)
        slope = (nodes2[i+1] - nodes2[i])/(nodes1[i+1] - nodes1[i])
        theta= math.atan(slope)
        if nodes2[i+1]>nodes2[i] and np.degrees(theta)<0 :
            
            theta = 180 + np.degrees(theta)
            
            theta = np.radians(theta)
        if nodes2[i+1]<nodes2[i] and np.degrees(theta)>0 :
            
            theta = np.degrees(theta) -180
            
            theta = np.radians(theta)
        ang_t = abs(theta - theta_curr)/2
        print('ang_t',ang_t)
        if theta - theta_curr>0:

            msg1.angular.z=-2 #taking 2rad/s
        else:
            msg1.angular.z=2 #
        msg1.linear.x =0
        msg2.linear.x=0.09 #taking as 0.1m/s
        msg2.linear.y=0.09
        msg2.angular.z = 0

        start = rospy.Time.now()
        while rospy.Time.now() - start <= rospy.Duration(ang_t):
            pub.publish(msg1)
        #time.sleep(0.1)
        msg1.angular.z=0
        pub.publish(msg1)
        start1 = rospy.Time.now()
        while rospy.Time.now() - start1 <= rospy.Duration(t):
            pub.publish(msg2)
        #time.sleep(0.1)   
        msg2.linear.x=0 
        msg2.linear.y=0 
        pub.publish(msg2)
        theta_curr = theta
        

if __name__=='__main__':
	test(nodes1,nodes2)
