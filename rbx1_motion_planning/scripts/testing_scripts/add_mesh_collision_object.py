#! /usr/bin/env python3

import moveit_commander
import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose

rospy.init_node("add_mesh_collision_object", anonymous=True)

# Initialize PlanningSceneInterface
scene = PlanningSceneInterface()

# Wait for the scene to initialize
rospy.sleep(2)

# Define the pose of the mesh
mesh_pose = Pose()
mesh_pose.position.x = 0.5
mesh_pose.position.y = 0.0
mesh_pose.position.z = 0.0
mesh_pose.orientation.w = 1.0

# Add the mesh to the scene
mesh_name = "chess_board"
mesh_path = "package://rbx1_motion_planning/meshes/chess_board.fbx"  # Path to your mesh
scene.add_mesh(mesh_name, mesh_pose, mesh_path, size=(1.0, 1.0, 1.0))