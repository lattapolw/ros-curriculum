#!/usr/bin/env python
import roslib
roslib.load_manifest('setgoal_actionlib')
from math import radians, cos, sqrt
import rospy
import actionlib
import actionlib_msgs.msg
from numpy import genfromtxt as gt
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseActionResult, MoveBaseGoal, MoveBaseAction

def send_to_client():
  client = actionlib.SimpleActionClient('latlong_server', MoveBaseAction)
  client.wait_for_server()
  goal = MoveBaseGoal()
  while True:
    print('Input LatLong...')
    [lat, lon] = receiveGoal()
    if lat==0 and lon==0:
      break
    [coor_x, coor_y] = equirect_latlong_to_xy(lat, lon)
    goal.target_pose = stamp(coor_x,coor_y)
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result
    print(result)


# constants w.r.t Mars Society MDRS
R     =  6378137.0      # earth equatorial radius, in meters
lat1  =       38.406419 # initial latitude
long0 =     -110.791920 # initial longitude

def stamp(coor_x,coor_y):
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
  return goal


# converts input latitude and longitude coordinates 
#   (expressed as degrees!)
# into ROS x, y w.r.t /map coordinates
def equirect_latlong_to_xy(lat, long):
  global R, lat1, long0
  x = R * (radians(long) - radians(long0)) * cos(radians(lat1))
  y = R * (radians(lat) - radians(lat1))
  return x,y

# def goal_list():
#   global lat_cor, long_cor, coor_x, coor_y
#   latlong_list = gt(fname='nav_targets.csv',skip_header=1,delimiter=',') #~/catkin_ws/src/latlong/
#   lat_cor = [latlong_list[i][1] for i in range(latlong_list.shape[0])]
#   long_cor = [latlong_list[i][2] for i in range(latlong_list.shape[0])]
#   [coor_x,coor_y] = equirect_latlong_to_xy(lat_cor[0], long_cor[0])

def receiveGoal():
  latlong_user = input()
  print(equirect_latlong_to_xy(latlong_user[0],latlong_user[1]))
  return latlong_user

if __name__ == '__main__':
  rospy.init_node('latlong_client')
  send_to_client()
  print("Done")

# test changing code 1