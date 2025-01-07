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

    # For asking user to provide a file name
    # piece_name = str(input("Piece Name: "))

    # Get the package path using rospkg
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('rbx1_motion_planning')  # Replace with your package name

    piece_names = ("chess_board", "king", "queen", "bishop", "knight", "rook", "pawn")
    for piece_name in piece_names:

        # Get file name from piece name
        file_name = piece_name + ".stl"

        # Define the STL file path
        mesh_file_path = os.path.join(package_path, "meshes", "scaled_stl", file_name)

        # Define the pose of the collision object
        collision_pose = geometry_msgs.msg.PoseStamped()
        collision_pose.header.frame_id = 'world'
        collision_pose.pose.position.x = 0.55
        collision_pose.pose.position.y = 0.0
        collision_pose.pose.position.z = 0.0
        collision_pose.pose.orientation.w = 1.0  # Identity rotation
        rospy.loginfo(collision_pose)

        # Add the mesh as a collision object in the planning scene
        scene.add_mesh(piece_name, collision_pose, mesh_file_path, (1.5,1.5,1.5) )

        # Allow RViz to update and visualize the object
        rospy.sleep(2)

if __name__ == "__main__":
    main()