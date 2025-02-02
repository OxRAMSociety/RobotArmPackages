#!/usr/bin/env python3

# Adds the chess board and pieces in their starting positions to the environment as collision objects

import sys
import moveit_commander
import rospy
import os
import rospkg  # Used to resolve package paths
import geometry_msgs.msg

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("add_chess_collision_object", anonymous=True)

    # Initialize the PlanningSceneInterface
    scene = moveit_commander.PlanningSceneInterface()

    # Wait for the scene to initialize
    rospy.sleep(2)

    # Get the folder path and list the files
    rospack = rospkg.RosPack() 
    package_path = rospack.get_path('rbx1_motion_planning') # get the package path
    mesh_folder_path = os.path.join(package_path, "meshes", "chess_starting_positions") # get the full folder path
    mesh_files = os.listdir(mesh_folder_path) # list the files in the folder

    for file_name in mesh_files:

        # Get file name from piece name
        piece_name = file_name.replace('.stl','')

        # Define the pose of the collision object
        collision_pose = geometry_msgs.msg.PoseStamped()
        collision_pose.header.frame_id = 'world'
        collision_pose.pose.position.x = 0.22
        collision_pose.pose.position.y = 0
        collision_pose.pose.position.z = 0
        collision_pose.pose.orientation.w = 1.0
        rospy.loginfo(collision_pose)

        mesh_file_path = mesh_folder_path + "/" + file_name

        # Add the mesh as a collision object in the planning scene
        scene.add_mesh(piece_name, collision_pose, mesh_file_path, (0.4,0.4,0.4))

        # Allow RViz processing time
        rospy.sleep(0.05)
    # Allow RViz to update and visualize the objects
    rospy.sleep(1)
if __name__ == "__main__":
    main()