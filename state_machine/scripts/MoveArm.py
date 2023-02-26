#!/usr/bin/env python3

# This script provides an action server by which to control the arm using MoveIt

import sys
import rospy
import signal
import time


import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from math import pi
import math
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from actionlib import SimpleActionServer
from rbx1_scripts.msg import executeJointGoalAction, executeJointGoalFeedback, executeJointGoalResult
from rbx1_scripts.msg import executePoseGoalAction, executePoseGoalFeedback, executePoseGoalResult
from rbx1_scripts.msg import executePositionGoalAction, executePositionGoalFeedback, executePositionGoalResult