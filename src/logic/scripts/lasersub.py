#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np


def publish_velocity_commands(angular_vel, linear_vel):
    # Velocity publisher
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    # rospy.init_node('commandvel')
    print("Angular vel " + str(angular_vel))
    print("Linear vel " + str(linear_vel))

    msg = Twist()
    msg.linear.x = linear_vel
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 1
    msg.angular.y = 0
    msg.angular.z = angular_vel

    # rate = rospy.Rate(10)  # 10hz
    # while not rospy.is_shutdown():
    vel_pub.publish(msg)
    # rate.sleep(1)
    # rospy.Subscriber("/mybot/laser/scan", LaserScan, get_distance)


def get_distance(msg):
    # 720 messages in range array
    range_array = msg.ranges
    # Distance of bot from the left (0 deg)
    left_distance = min(np.array(range_array[360: 720]))
    # Distance of bot from the front (90 deg)
    front_distance = min(np.array(range_array[180: 360]))
    # Distance of bot from the right (180 deg)
    right_distance = min(np.array(range_array[0: 180]))

    # Printing values at 0 degree
    print("Left", left_distance)
    # Printing values at 90 degree
    print("Front", front_distance)
    # Printing values at 180 degree
    print("Right", right_distance,)

    if front_distance > 2 and right_distance > 2:
        if left_distance > 0.9 and left_distance < 1.2:
            publish_velocity_commands(0, 0.3)
            print("Distance from left wall is = 1")
        if left_distance > 1.2:
            publish_velocity_commands(-0.8, 0.3)
            print("Distance from left wall is > 1")

        if left_distance < 0.9:
            publish_velocity_commands(0.6, 0.3)
            print("Distance from left wall is < 1")

    if front_distance <= 2 and front_distance > 1:
        if front_distance < left_distance:
            publish_velocity_commands(0.6, 0.05)
            print("Distance from left wall is < 1")
        if front_distance > left_distance:
            publish_velocity_commands(0.6, 0)
            print("Distance from left wall is < 1")

    if front_distance <= 1 and right_distance > 2:
        publish_velocity_commands(0.6, 0)
        print("Distance from left wall is < 1")

    # angle = i * m
    # if m < 0.8:
    #     print("M = " + str(m))
    #     publish_velocity_commands(-0.1, 0.1)

    # if m > 1.2:
    #     print("M = " + str(m))
    #     publish_velocity_commands(-0.1, 0.1)
    # elif str(m) == 'inf':
    #     publish_velocity_commands(0.2, 0.1)

    # if m == 1.2:
    #     print("M = " + str(m))
    #     publish_velocity_commands(-0.4, 0)

    # else:
    #     print("M = "+str(m))

    #     publish_velocity_commands(0.1, 0)

    # print(m, i, angle)


def main():
    # initializing node
    rospy.init_node('lasersub')
    # rate = rospy.Rate(10)
    # Subscribing to laserscanner
    rospy.Subscriber("/mybot/laser/scan", LaserScan, get_distance)

    rospy.spin()


if __name__ == '__main__':
    main()
