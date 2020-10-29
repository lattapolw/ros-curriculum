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

def setGoal_client():
  num_goal = 0
  client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  client.wait_for_server()
  goal = MoveBaseGoal()
  while num_goal <= 6:
    [coor_x, coor_y] = equirect_latlong_to_xy(lat_cor[num_goal], long_cor[num_goal])
    goal.target_pose = stamp(coor_x,coor_y)
    client.send_goal(goal)
    # client.wait_for_result()
    result = 0
    while result != 3:
      result = client.get_state()
      print(result)
    num_goal += 1


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
  coordinate.x = coor_x + 0.1
  coordinate.y = coor_y + 0.1
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

def goal_list():
  global lat_cor, long_cor, coor_x, coor_y
  latlong_list = gt(fname='nav_targets.csv',skip_header=1,delimiter=',') #~/catkin_ws/src/latlong/
  lat_cor = [latlong_list[i][1] for i in range(latlong_list.shape[0])]
  long_cor = [latlong_list[i][2] for i in range(latlong_list.shape[0])]
  [coor_x,coor_y] = equirect_latlong_to_xy(lat_cor[0], long_cor[0])

if __name__ == '__main__':
  rospy.init_node('setGoal_threading')
  goal_list()
  setGoal_client()
  print("Done")