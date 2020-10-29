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

class ReceiveGoal(object):
  feedback = MoveBaseFeedback()
  result = MoveBaseActionResult()

  def __init__(self, name):
    self.action_name = name
    self.aserver = actionlib.SimpleActionServer(self.action_name, MoveBaseAction, execute_cb=self.execute_cb, auto_start=False)
    self.aserver.start()

  def execute_cb(self, goal):
    r = rospy.Rate(1)
    
    
if __name__ == '__main__':
  rospy.init_node('goal_receive_server')
  server = ReceiveGoal(rospy.get_name())
  rospy.spin()