#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import numpy as np

def callback(data):
    idx_min = np.argmin(data.ranges)
    inc_angle = idx_min*data.angle_increment
    min_angle = data.angle_min + inc_angle
    smallest_dist_msmt = data.ranges[idx_min]
    x_pos = smallest_dist_msmt*np.cos(min_angle)
    y_pos = smallest_dist_msmt*np.sin(min_angle)
    p = rospy.get_param('p')
    ang_spd_control = -p*min_angle

    rospy.loginfo("Smallest distance measurement: %s", smallest_dist_msmt)
    rospy.loginfo("X position: %s", x_pos)
    rospy.loginfo("Y position: %s", y_pos)
    rospy.loginfo("Ang Control: %s", ang_spd_control)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    pub.publish(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, ang_spd_control))

def listener():
    rospy.init_node('husky_highlevel_controller')
    if rospy.has_param('p'):
        rospy.get_param('p')
    else:
        rospy.set_param('p', 0.5)
    rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__' :
    listener()