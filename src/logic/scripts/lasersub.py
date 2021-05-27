#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image



def callback(msg):
    r = msg.ranges
    m = min(r)
    i = r.index(m)
    a = msg.angle_increment

    angle = i * m

    print(m, i, angle)




def main():
    #initializing node
    rospy.init_node('lasersub')
    #setting uo subscriber
    rospy.Subscriber("/mybot/laser/scan", LaserScan, callback)
    
    
    rospy.spin()


if __name__ == '__main__':
    main()