#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from math import sqrt


def compute_grid_positions(corners, board_size):
    grid_positions = np.zeros((board_size[0]+2, board_size[1]+2, 2), dtype=np.float32)

    for i in range(1, board_size[0]+1):
        for j in range(1, board_size[1]+1):
            grid_positions[i, j] = corners[(j-1) + (i-1) * board_size[1]].ravel()
    for j in range(1, board_size[1]+1): # corners [0,1] to [0,7]
            diffh = (grid_positions[2,j][1] - grid_positions[1,j][1])
            diffw = (grid_positions[2,j][0] - grid_positions[1,j][0])
            grid_positions[0,j] = [grid_positions[1,j][0] - diffw, grid_positions[1,j][1] - diffh]
    for j in range(0, board_size[0]+1): # corners [0,0] to [7,0]
            diffh = (grid_positions[j,2][1] - grid_positions[j,1][1])
            diffw = (grid_positions[j,2][0] - grid_positions[j,1][0])
            grid_positions[j, 0] = [grid_positions[j,1][0] - diffw, grid_positions[j,1][1] - diffh]
    for j in range(0, board_size[1]+1): # corners [8,1] to [8,7]
            diffh = (grid_positions[6,j][1] - grid_positions[7,j][1])
            diffw = (grid_positions[6,j][0] - grid_positions[7,j][0])
            grid_positions[8,j] = [grid_positions[7,j][0] - diffw, grid_positions[7,j][1] - diffh]
    for j in range(0, board_size[0]+2): # corners [1,0] to [7,0]
            diffh = (grid_positions[j,6][1] - grid_positions[j,7][1])
            diffw = (grid_positions[j,6][0] - grid_positions[j,7][0])
            grid_positions[j, 8] = [grid_positions[j,7][0] - diffw, grid_positions[j,7][1] - diffh]
    return grid_positions


def process_image(cv_image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Chessboard size (width and height)
    board_size = (7, 7)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, board_size, None) # MatLike

    if ret:
        rospy.loginfo("Chessboard corners detected.")
    else:
        rospy.loginfo("Chessboard corners not detected.")

    
    
    # If the chessboard is detected, draw the grid and compute the grid positions
    if ret:
        rospy.loginfo(type(corners))
        corners = compute_grid_positions(corners, board_size)
        #cv2.drawChessboardCorners(cv_image, (9, 9), corners, ret)
        for i in range(0, 9):
            for j in range(0, 9):
                 cv2.circle(cv_image, (int(corners[i, j][0]), int(corners[i, j][1])), 10, (255, 0, 0), 1)
        rospy.loginfo(type(corners))
        # Compute the grid positions
        #corners = compute_grid_positions(corners, board_size)
        # Print the grid positions
        rospy.loginfo("Grid positions:")
        rospy.loginfo(corners)
    cv2.imshow("GRID1", cv_image)
    cv2.waitKey(0)
    
    return cv_image

def image_publisher():
    # Initiate the node
    rospy.init_node('image_publisher_node')

    # Create a publisher
    pub = rospy.Publisher('image_topic', Image, queue_size=10)

    # Load the image
    path = "/home/kirsten/tester_computer_vision/src/RobotArmPackages/rbx1_computer_vision/scripts/chessboard3.png"
    cv_image = cv2.imread(path)  # change this to your image file path - MatLike

    process_image(cv_image)
    
    if cv_image is None:    
        print("Could not open or find the image")
        return

    ros_image_msg = Image()
    ros_image_msg.height = cv_image.shape[0]
    ros_image_msg.width = cv_image.shape[1]
    ros_image_msg.encoding = "bgr8"
    ros_image_msg.is_bigendian = 0
    ros_image_msg.step = cv_image.shape[1] * 3  # Full row length in bytes
    ros_image_msg.data = np.array(cv_image, dtype=np.uint8).tobytes()

    # Publish the image
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        pub.publish(ros_image_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass