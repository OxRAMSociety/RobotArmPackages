#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

# Takes locations of the inner corners, and the board size and calculates all corners
def compute_grid_positions(corners, board_size):
    grid_positions = np.zeros((board_size[0]+2, board_size[1]+2, 2), dtype=np.float32)
    # Inner corners
    for i in range(1, board_size[0]+1):
        for j in range(1, board_size[1]+1):
            grid_positions[i, j] = corners[(j-1) + (i-1) * board_size[1]].ravel()
    # corners [0,1] to [0,7]
    for j in range(1, board_size[1]+1): 
        diffh = (grid_positions[2,j][1] - grid_positions[1,j][1])
        diffw = (grid_positions[2,j][0] - grid_positions[1,j][0])
        grid_positions[0,j] = [grid_positions[1,j][0] - diffw, grid_positions[1,j][1] - diffh]
    # corners [0,0] to [7,0]
    for j in range(0, board_size[0]+1): 
        diffh = (grid_positions[j,2][1] - grid_positions[j,1][1])
        diffw = (grid_positions[j,2][0] - grid_positions[j,1][0])
        grid_positions[j, 0] = [grid_positions[j,1][0] - diffw, grid_positions[j,1][1] - diffh]
    # corners [8,1] to [8,7]
    for j in range(0, board_size[1]+1):
        diffh = (grid_positions[6,j][1] - grid_positions[7,j][1])
        diffw = (grid_positions[6,j][0] - grid_positions[7,j][0])
        grid_positions[8,j] = [grid_positions[7,j][0] - diffw, grid_positions[7,j][1] - diffh]
    # corners [1,0] to [7,0]
    for j in range(0, board_size[0]+2): 
        diffh = (grid_positions[j,6][1] - grid_positions[j,7][1])
        diffw = (grid_positions[j,6][0] - grid_positions[j,7][0])
        grid_positions[j, 8] = [grid_positions[j,7][0] - diffw, grid_positions[j,7][1] - diffh]
    return grid_positions


# Turns image gray scale, finds chessboard corners and draws on the corners
# Logs: Chessboard detected, type of corners, grid positions
def process_image(cv_image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # Chessboard size in squares(width and height)
    board_size = (7, 7)
    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, board_size, None) # MatLike     
    # If the chessboard is detected, draw the grid and compute the grid positions
    if ret:
        rospy.loginfo("Chessboard corners detected.")
        rospy.loginfo(type(corners))
        corners = cv2.cornerSubPix(gray, corners, (3,3), (-1, -1), (cv2.TERM_CRITERIA_EPS, 0, 0.00001)) # More accurate corner detection
        corners = compute_grid_positions(corners, board_size)
        # Draw on circle
        for i in range(0, 9):
            for j in range(0, 9):
                 cv2.circle(cv_image, (int(corners[i, j][0]), int(corners[i, j][1])), 5, (255, 0, 0), 1)
        rospy.loginfo(type(corners))
        # Print the grid positions
        rospy.loginfo("Grid positions:")
        rospy.loginfo(corners)
    else:
        rospy.loginfo("Chessboard corners not detected.")
    return cv_image


def video_publisher():
    # Initiate the node
    rospy.init_node('video_publisher_node')
    # Create a publisher
    pub = rospy.Publisher('video_topic', Image, queue_size=10)
    # Create an instance of CvBridge
    bridge = CvBridge()
    # Load the video
    video = cv2.VideoCapture(0)  # change this to your video file path
    rate = rospy.Rate(30)  # 30 Hz to match typical video frame rate
    while not rospy.is_shutdown():
        # Read a frame from the video
        ret, frame = video.read()
        if ret:
            frame = process_image(frame)
            if frame is None:    
                print("Could not open or find the image")
                return
            # Convert the frame to a ROS image message
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            # Publish the frame
            pub.publish(ros_image)
            rate.sleep()
        else:
            break
        cv2.imshow("GRID1", frame)
        cv2.waitKey(1)

    video.release()

if __name__ == '__main__':
    try:
        video_publisher()
    except rospy.ROSInterruptException:
        pass
