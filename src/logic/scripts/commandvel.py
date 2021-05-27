#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist
 
def publish_velocity_commands():
  # Velocity publisher
  vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
  rospy.init_node('commandvel')
 
  msg = Twist()
  msg.linear.x = 0.1
  msg.linear.y = 0
  msg.linear.z = 0
  msg.angular.x = 0
  msg.angular.y = 0
  msg.angular.z = 0
 
  rate = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    vel_pub.publish(msg)
    rate.sleep()

if __name__ == '__main__':
    try:
      publish_velocity_commands()      
    except rospy.ROSInterruptException:
      pass

