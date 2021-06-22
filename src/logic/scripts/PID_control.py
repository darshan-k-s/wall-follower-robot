#!/usr/bin/env python
import rospy
import roslib
import sys
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from math import pi
import time
start = time.time()


class pidVel():
    # Constructor for initialization
    def __init__(self):
        # Initialize node
        rospy.init_node('wallfollow')
        # Log info when node starts
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)

        # Setpoint distance from wall
        self.setpoint = 1.5

        # Constants and gains
        self.kpAngular = 10     #Proportional gain
        self.kiAngular = 0      #Integral gain
        self.kdAngular = 30     #Derivative gain
        self.dt = 0.16689300537109375
        self.linear = 1         # Base case Linear velocity
        self.errors = 0         # Integral value
        self.prevErr = 0        # Derivative value
        self.imax = 1           # Max integral value
        self.imin = -1          # Min integral value
        self.dis = [0, 0, 0]    # Array for storing distances from laserscan

        # Initializing publisher
        self.end = time.time()
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


    # Publisher to cmd_vel and statements to avoid collision and smooth functioning
    def publishVel(self):
        # Infinity case control
        if self.leftdistance > 10:
            self.angularResponse = -1 * pi/2
            self.linear = 0.5

        # Front obstacle avoidance
        if self.frontdistance < 2.5:
            self.angularResponse = 9
        # Optimization for U-turns
        elif (self.left2 > 5):
            self.angularResponse = -1 * pi/1.8

        # Linear velocity saturation
        if self.linear > 1:
            self.linear = 1
        if self.linear < 0.1:
            self.linear = 0.1

        # Printing values to cmd line
        print("A: " + str(self.angularResponse))
        print("L: " + str(self.linear))

        # Actual publishing
        self.vel = Twist()
        self.vel.linear.x = self.linear
        self.vel.angular.z = self.angularResponse
        self.vel_pub.publish(self.vel)


    def calculateLinearVel(self):
        # Only proportional control
        # Based on left distance
        if(self.error < 1.5 or self.error > -1.5):
            self.linear = 1 - (0.6) * abs(self.error)
            
        #Based on front distance
        if(self.frontdistance < 4):
            self.linear = 1 - (0.33) * abs(4-self.frontdistance)


    # Main PID Logic for angular control
    def calculateAngularVel(self):
        # Error term for angular PID
        self.error = (self.setpoint - self.leftdistance)

        # Proportional term
        self.pAngular = self.kpAngular * self.error

        # Integral term
        self.errors += self.error
        if(self.errors > self.imax):
            self.errors = self.imax
        elif(self.errors < self.imin):
            self.errors = self.imin
        self.iAngular = self.kiAngular * self.errors

        # Derivative Term
        self.dAngular = self.kdAngular * (self.error - self.prevErr) / self.dt
        self.prevErr = self.error

        # Total response
        self.angularResponse = self.pAngular + self.iAngular + self.dAngular

        # Saturating the angular responses
        if(self.angularResponse > pi/2):
            self.angularResponse = pi/2
        elif(self.angularResponse < -1 * pi/2):
            self.angularResponse = -1*pi/2


    def pid(self, msg):
        # Storing distance ranges
        # 720 points in reading
        self.dis[0] = min(msg.ranges[460:719])  # Left
        self.dis[1] = min(msg.ranges[350:370])  # Front
        self.dis[2] = min(msg.ranges[520:560])  # Left-2

        self.leftdistance = self.dis[0]   # Storing Left distance
        self.frontdistance = self.dis[1]  # Storing Front distance
        self.left2 = self.dis[2]          # left2 is used to optimize U-turns  

        print('Front : {}, Left : {}, Left-2 : {}'.format(self.frontdistance, self.leftdistance,self.left2))

        # Functions to calculate velocities
        self.calculateAngularVel()
        self.calculateLinearVel()
        # Publishing the velocities
        self.publishVel()


    def main(self):
        # Callback for subscriber
        rospy.Subscriber("/mybot/laser/scan", LaserScan, self.pid)


if __name__ == '__main__':
    # Creatind an object of class pidVel
    pidVelocity = pidVel()
    pidVelocity.main()
    rospy.spin()
