#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np

# Function to compute the grid positions from the detected corners
def compute_grid_positions(corners, board_size):
    grid_positions = np.zeros((board_size[0], board_size[1], 2), dtype=np.float32)

    for i in range(board_size[0]):
        for j in range(board_size[1]):
            grid_positions[i, j] = corners[j + i * board_size[1]].ravel()

    return grid_positions

# Function to detect the chessboard and draw the grid
def process_image(cv_image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Chessboard size (width and height)
    board_size = (7, 7)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, board_size, None)

    
    
    # If the chessboard is detected, draw the grid and compute the grid positions
    if ret:
        cv2.drawChessboardCorners(cv_image, board_size, corners, ret)

        # Compute the grid positions
        grid_positions = compute_grid_positions(corners, board_size)

        # Print the grid positions
        rospy.loginfo("Grid positions:")
        rospy.loginfo(grid_positions)
    
    return cv_image
'''
# Callback function for the subscriber
def image_callback(msg):
    # Convert ROS Image message to OpenCV image
    width = msg.width
    height = msg.height
    data = np.frombuffer(msg.data, dtype=np.uint8)
    cv_image = np.reshape(data, (height, width, 3))

    processed_image = process_image(cv_image)

    # Convert the processed OpenCV image to a ROS Image message
    processed_msg = Image()
    processed_msg.height = processed_image.shape[0]
    processed_msg.width = processed_image.shape[1]
    processed_msg.encoding = "bgr8"
    processed_msg.is_bigendian = 0
    processed_msg.step = processed_image.shape[1] * 3
    processed_msg.data = np.array(processed_image, dtype=np.uint8).tobytes()

    # Publish the processed image
    pub.publish(processed_msg)
'''
'''
# Callback function for the subscriber
def image_callback(msg):
    # Convert ROS Image message to OpenCV image
    width = msg.width
    height = msg.height
    data = np.frombuffer(msg.data, dtype=np.uint8)
    cv_image = np.reshape(data, (height, width, 1)) # For a monochrome image

    # Convert to 3-channel image if necessary
    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

    processed_image = process_image(cv_image)

    # Convert the processed OpenCV image to a ROS Image message
    processed_msg = Image()
    processed_msg.height = processed_image.shape[0]
    processed_msg.width = processed_image.shape[1]
    processed_msg.encoding = "bgr8"
    processed_msg.is_bigendian = 0
    processed_msg.step = processed_image.shape[1] * 3
    processed_msg.data = np.array(processed_image, dtype=np.uint8).tobytes()

    # Publish the processed image
    pub.publish(processed_msg)
'''

# Callback function for the subscriber
def image_callback(msg):
    # Convert ROS Image message to OpenCV image
    width = msg.width
    height = msg.height
    data = np.frombuffer(msg.data, dtype=np.uint8)
    cv_image = np.reshape(data, (height, width, 3)) # For a colored image

    processed_image = process_image(cv_image)

    # Convert the processed OpenCV image to a ROS Image message
    processed_msg = Image()
    processed_msg.height = processed_image.shape[0]
    processed_msg.width = processed_image.shape[1]
    processed_msg.encoding = "bgr8"
    processed_msg.is_bigendian = 0
    processed_msg.step = processed_image.shape[1] * 3
    processed_msg.data = np.array(processed_image, dtype=np.uint8).tobytes()

    # Publish the processed image
    pub.publish(processed_msg)

if __name__ == '__main__':
    rospy.init_node('chessboard_detection')

    pub = rospy.Publisher("processed_image", Image, queue_size=10)

    rospy.Subscriber("/camera/color/image_rect_color", Image, image_callback)
    rospy.spin()
