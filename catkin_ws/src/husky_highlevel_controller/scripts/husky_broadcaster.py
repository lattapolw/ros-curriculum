#!/usr/bin/env python  
import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np

def callback(data):
    global marker_pub
    idx_min = np.argmin(data.ranges)
    inc_angle = idx_min*data.angle_increment
    min_angle = data.angle_min + inc_angle
    smallest_dist_msmt = data.ranges[idx_min]

    pillar_point = Point()
    pillar_point.x = smallest_dist_msmt*np.cos(min_angle)
    pillar_point.y = smallest_dist_msmt*np.sin(min_angle)
    pillar_point.z = 0

    pillar_marker = Marker()
    pillar_marker.header.frame_id = "/base_laser"
    pillar_marker.header.stamp    = rospy.get_rostime()
    pillar_marker.ns     = "/"
    pillar_marker.id     = 0
    pillar_marker.type   = 3
    pillar_marker.action = 0
    pillar_marker.pose.position = pillar_point
    pillar_marker.pose.orientation.x = 0
    pillar_marker.pose.orientation.y = 0
    pillar_marker.pose.orientation.z = 0
    pillar_marker.pose.orientation.w = 1
    pillar_marker.scale.x  = 1
    pillar_marker.scale.y  = 1
    pillar_marker.scale.z  = 1
    pillar_marker.color.r  = 1
    pillar_marker.color.g  = 0
    pillar_marker.color.b  = 0
    pillar_marker.color.a  = 1
    pillar_marker.lifetime = rospy.Duration(secs=5)
    marker_pub.publish(pillar_marker)

def listener(scan_topic, queue_size):
    rospy.Subscriber(scan_topic, LaserScan, callback, queue_size=queue_size)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('husky_broadcaster')
    marker_topic  = "pillar_pos" #rospy.get_param(rospy.get_name() + "/marker_topic", "visualization_marker")
    scan_topic  = rospy.get_param(rospy.get_name() + "/scan_topic", "scan")
    queue_size  = rospy.get_param(rospy.get_name() + "/queue_size", 1)
    marker_pub = rospy.Publisher(marker_topic, Marker)
    listener(scan_topic, queue_size)