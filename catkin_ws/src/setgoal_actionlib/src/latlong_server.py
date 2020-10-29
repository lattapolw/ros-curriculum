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

class latlong_server(object):
  _feedback = MoveBaseFeedback()
  _result = MoveBaseActionResult()

  def __init__(self, name):
    self._action_name = name
    self.server = actionlib.SimpleActionServer(self._action_name, MoveBaseAction, execute_cb=self.send_goal, auto_start=False)
    self.server.start()
    print("latlong_server started...")

  def send_goal(self, goal):
    print(goal)
    # self._feedback = goal
    self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    self.client.wait_for_server()
    self.client.send_goal(goal)
    self.client.wait_for_result()

    self._result = self.client.get_state()
    # self.server.publish_feedback(self._result)
    self.server.set_succeeded(self._result)


if __name__ == '__main__':
  rospy.init_node('latlong_server')
  result = latlong_server(rospy.get_name())
  rospy.spin()
  print("Done")