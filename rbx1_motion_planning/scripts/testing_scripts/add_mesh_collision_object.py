#!/usr/bin/env python3

import sys
import moveit_commander
import rospy
import os
import rospkg  # Used to resolve package paths
import geometry_msgs.msg

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("add_mesh_collision_object", anonymous=True)

    # Initialize the PlanningSceneInterface
    scene = moveit_commander.PlanningSceneInterface()

    # Wait for the scene to initialize
    rospy.sleep(2)

    # Get the package path using rospkg
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('rbx1_motion_planning')  # Replace with your package name

    # Define the STL file path
    mesh_file_path = os.path.join(package_path, "meshes", "chess_board.stl")

    # Define the pose of the collision object
    collision_pose = geometry_msgs.msg.PoseStamped()
    collision_pose.header.frame_id = 'world'
    collision_pose.pose.position.x = 1.0
    collision_pose.pose.position.y = 0.0
    collision_pose.pose.position.z = 0.0
    collision_pose.pose.orientation.w = 1.0  # Identity rotation
    rospy.loginfo(collision_pose)

    # Add the mesh as a collision object in the planning scene
    scene.add_mesh("chess_board", collision_pose, mesh_file_path)

    # Allow RViz to update and visualize the object
    rospy.sleep(2)

if __name__ == "__main__":
    main()