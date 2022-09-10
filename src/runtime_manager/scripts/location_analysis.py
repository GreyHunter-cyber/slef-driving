#!/usr/bin/env python
# _*_ enconding=utf-8 _*_

import time
import rospy
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry


xs = []
ys = []

def laser_odom_callback(laser_odom_data):
    x = laser_odom_data.pose.pose.position.x
    y = laser_odom_data.pose.pose.position.y
    xs.append(x)
    ys.append(y)
    plt.plot(xs,ys)
    #plt.pause(0.000001)
    #plt.show()
    print(x,y)



if __name__ == '__main__':
    try:
        rospy.init_node("location_analysis")
        rospy.Subscriber('/integrated_to_init',Odometry,laser_odom_callback)
        plt.ion()
        plt.show()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('location analysis node init error!')
        pass
