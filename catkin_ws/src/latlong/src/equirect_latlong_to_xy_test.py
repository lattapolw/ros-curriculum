#!/usr/bin/env python
from math import radians, cos, sqrt
import rospy
from numpy import genfromtxt as gt
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

# constants w.r.t Mars Society MDRS
R     =  6378137.0      # earth equatorial radius, in meters
lat1  =       38.406419 # initial latitude
long0 =     -110.791920 # initial longitude

goal_count = 0
count = 0
send_goal = 1
do_once = 1

def setgoal(data):
  global goal_count, count, send_goal, coor_x, coor_y, do_once
  data_position = data.pose.pose.position
  cur_position = [data_position.x, data_position.y]

  global goal_pub
  dist = sqrt((coor_x-cur_position[0])**2+(coor_y-cur_position[1])**2)
  if dist <= 1.5:
    if count <= 250:
      count += 1
    elif count > 250:
      goal_count += 1
      count = 0
      send_goal = 1
  else:
    count = 0
  if goal_count == 7:
    goal_count = 0
    send_goal = 1
  [coor_x, coor_y] = equirect_latlong_to_xy(lat_cor[goal_count], long_cor[goal_count])

  print("dist = %f, count = %f, goal = %f" %(dist,count,goal_count))

  marker = Header()
  marker.seq = 0
  marker.frame_id = "odom"
  marker.stamp = rospy.get_rostime()

  coordinate = Point()
  coordinate.x = coor_x
  coordinate.y = coor_y
  coordinate.z = 0

  orientation = Quaternion()
  orientation.x = 0
  orientation.y = 0
  orientation.z = 1
  orientation.w = 0

  pose = Pose()
  pose.position = coordinate
  pose.orientation = orientation

  goal = PoseStamped()
  goal.header = marker
  goal.pose = pose
  if send_goal == 1:
    goal_pub.publish(goal)
    print("sent")
    rospy.sleep(1)
    if do_once == 1:
      goal_pub.publish(goal)
      print("sent")
      rospy.sleep(1)
      do_once = 0
    send_goal = 0

# converts input latitude and longitude coordinates 
#   (expressed as degrees!)
# into ROS x, y w.r.t /map coordinates
def equirect_latlong_to_xy(lat, long):
  global R, lat1, long0
  x = R * (radians(long) - radians(long0)) * cos(radians(lat1))
  y = R * (radians(lat) - radians(lat1))
  return x,y

def goal_list():
  global lat_cor, long_cor, coor_x, coor_y
  latlong_list = gt(fname='nav_targets.csv',skip_header=1,delimiter=',')
  lat_cor = [latlong_list[i][1] for i in range(latlong_list.shape[0])]
  long_cor = [latlong_list[i][2] for i in range(latlong_list.shape[0])]
  [coor_x,coor_y] = equirect_latlong_to_xy(lat_cor[0], long_cor[0])

def get_global_pos():
  rospy.Subscriber("/odometry/filtered/global", Odometry, setgoal, queue_size=1)
  rospy.spin()

if __name__ == '__main__':
  rospy.init_node('goal_publisher')
  goal_list()
  goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
  get_global_pos()