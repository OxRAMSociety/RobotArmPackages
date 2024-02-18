#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np


def compute_grid_positions(corners, board_size):
    grid_positions = np.zeros((board_size[0], board_size[1], 2), dtype=np.float32)

    for i in range(board_size[0]):
        for j in range(board_size[1]):
            grid_positions[i, j] = corners[j + i * board_size[1]].ravel()

    return grid_positions


def process_image(cv_image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Chessboard size (width and height)
    board_size = (7, 7)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, board_size, None)

    if ret:
        rospy.loginfo("Chessboard corners detected.")
    else:
        rospy.loginfo("Chessboard corners not detected.")

    
    
    # If the chessboard is detected, draw the grid and compute the grid positions
    if ret:
        cv2.drawChessboardCorners(cv_image, board_size, corners, ret)

        # Compute the grid positions
        grid_positions = compute_grid_positions(corners, board_size)

        # Print the grid positions
        rospy.loginfo("Grid positions:")
        rospy.loginfo(grid_positions)
    
    return cv_image

def image_publisher():
    # Initiate the node
    rospy.init_node('image_publisher_node')

    # Create a publisher
    pub = rospy.Publisher('image_topic', Image, queue_size=10)

    # Load the image
    path = "/home/gabo/Documents/OXRAM/WALLICE/chessboard.png"
    cv_image = cv2.imread(path)  # change this to your image file path

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

