#!/usr/bin/env python 
import rospy
from geometry_msgs.msg import Twist

def run():
    
    rospy.init_node('commandvel')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    vel = Twist()
    vel.linear.x = 2
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 1


    pub.publish(vel)




if __name__ == 'main':
    run()