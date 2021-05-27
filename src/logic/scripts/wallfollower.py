#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


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
    print(msg)
    r = msg.ranges  # 720 messages
    # Laserbeam
    m = min(r)
    i = r.index(m)
    a = msg.angle_increment

    angle = i * m
    if m < 0.8:
        print("M = " + str(m))
        publish_velocity_commands(-0.1, 0.1)

    if m > 1.2:
        print("M = " + str(m))
        publish_velocity_commands(-0.1, 0.1)
    elif str(m) == 'inf':
        publish_velocity_commands(0.2, 0.1)

    if m == 1.2:
        print("M = " + str(m))
        publish_velocity_commands(-0.4, 0)

    else:
        print("M = "+str(m))

        publish_velocity_commands(0.1, 0)

    print(m, i, angle)


def main():
    # initializing node
    rospy.init_node('lasersub')
    # rate = rospy.Rate(10)
    # Subscribing to laserscanner
    rospy.Subscriber("/mybot/laser/scan", LaserScan, get_distance)

    rospy.spin()


if __name__ == '__main__':
    main()
