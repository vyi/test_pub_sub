#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from math import atan2, pi, acos, cos

x_r = 0
y_r = 0
theta_r = 0
x_destination, y_destination= 0,0

def update_goal(data):
    global x_destination, y_destination
    x_destination = data.x
    y_destination = data.y


def callback(data):
    '''
    Don't do long tasks here.
    Only store the data and go.
    '''
    global x_r, y_r, theta_r
    #print(data.pose.pose)
    temp = data.pose[1]

    x_r = temp.position.x
    y_r = temp.position.y

    x= temp.orientation.x
    y= temp.orientation.y
    z= temp.orientation.z
    w= temp.orientation.w
    roll, pitch, yaw = euler_from_quaternion([x,y,z,w])
    print("Roll {}, Pitch {}, Yaw {}".format(roll, pitch, yaw))
    #print("I received the following x:{}, y:{}, theta:{}".format( x_r, y_r, theta_r))
    theta_r = yaw 


#def callback(data):
#    '''
#    Don't do long tasks here.
#    Only store the data and go.
#    '''
#    global x_r, y_r, theta_r
#    #print(data.pose.pose)
#    x_r = data.pose.pose.position.x
#    y_r = data.pose.pose.position.y
#    x= data.pose.pose.orientation.x
#    y= data.pose.pose.orientation.y
#    z= data.pose.pose.orientation.z
#    w= data.pose.pose.orientation.w
#    roll, pitch, yaw = euler_from_quaternion([x,y,z,w])
#    #print("Roll {}, Pitch {}, Yaw {}".format(roll, pitch, yaw))
#    #print("I received the following x:{}, y:{}, theta:{}".format( x_r, y_r, theta_r))
#    theta_r = yaw

def saturate( x, limit):
    
    if x>limit:
        x = limit
    elif x<-limit:
        x = - limit
    return x

def listener():
    global x_r, y_r, theta_r, x_destination, y_destination

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('proportional_control_scheme')
    pubgoal = rospy.Publisher('/goal_reached', String, queue_size=1)
    #rospy.Subscriber('/odom', Odometry, callback)
    rospy.Subscriber('/gazebo/model_states', ModelStates, callback)
    rospy.Subscriber('/current_goal', Point, update_goal)
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
    log_ct = 0
    # spin() simply keeps python from exiting until this node is stopped
    try:
        while not rospy.is_shutdown():
            e_dist = ((y_destination-y_r)**2 + (x_destination-x_r)**2)**(0.5)
            #print("Current error in distance: {}".format(e_dist))
            
            if(e_dist > 0.2):
                ''' Do course correction '''
                
                ref_angle = atan2(y_destination-y_r, x_destination-x_r) 
                #e_angular = acos(cos(ref_angle - theta_r))

                # if the angles are of opposite signs
                if((ref_angle>=0) and (theta_r<=0)):
                    #handle case 1
                    theta_r += 2*pi
                    
                elif((ref_angle<0) and (theta_r>0)):
                    ref_angle += 2*pi
                
                e_angular = ref_angle - theta_r

                if (e_angular>pi):
                    e_angular = e_angular-2*pi
                elif (e_angular<-pi):
                    e_angular = e_angular+2*pi

                print("Errors :\t{:0.3f}\t{:0.2f}  {:0.2f}\t{:0.3f}".format(e_dist, theta_r, ref_angle, e_angular))
                my_msg = Twist()
                my_msg.linear.x = saturate(e_dist, 0.25)
                my_msg.angular.z = (saturate(e_angular, 0.3))
                pub.publish(my_msg)
            else:
                log_ct += 1
                if(log_ct > 15):    
                    log_ct =0
                    pubgoal.publish("{:0.2f},{:0.2f}".format(x_destination, y_destination))
                #print("Published a msg with x_dot :{}, theta_dot :{}".format(saturate(e_dist, 1.0), saturate(e_angular, 1.0)))
    except rospy.ROSInterruptException: 
        pass

    

if __name__ == '__main__':
    listener()
