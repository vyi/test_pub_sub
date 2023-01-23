#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from math import cos, sin, pi

# Global var
flag_reached= False
delta = 0.2
goal_x = 0
goal_y = 0

def callback(msg):
    global goal_x, goal_y, flag_reached 
    
    x,y = msg.data.strip().split(",")

    if ((float(x)-goal_x)**2 + (float(y)-goal_y)**2)**(0.5) <= 0.3:
        flag_reached = True
        print("Current goal reached! Issuing new goal.")


def talker():
    global flag_reached, delta, goal_x, goal_y
    pub = rospy.Publisher('/current_goal', Point, queue_size=1)
    sub = rospy.Subscriber('/goal_reached', String, callback)
    rospy.init_node('waypoint_generator')
    rate = rospy.Rate(10) # 10hz
    
    initial_theta = 0 #change this param to move the waypoint ahead 
    ###
    # start with first waypoint
    # 
    theta = initial_theta
    a = 1
    b = 2
    A = 5
    B = 3
    x = A*cos(a*theta)
    y = B*sin(b*theta)
    flag_reached = False
    point = Point()
    point.x = x
    point.y = y
    goal_x = x
    goal_y = y
    pub.publish(point)
    print("Published goal ({}, {})".format(x,y))

    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        if flag_reached:
            theta = theta + delta
            flag_reached = False
        x = A*cos(a*theta)
        y = B*sin(b*theta)
        point.x = x
        point.y = y 
        goal_x = x
        goal_y = y
        print("Published goal ({}, {})".format(x,y))
        pub.publish(point) 

        ###
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
