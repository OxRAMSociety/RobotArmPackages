#!/usr/bin/env python
#Temporary file storing the prototype for locating the boundary box

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
from random import random
from matplotlib.path import Path


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
        return corners
    else:
        rospy.loginfo("Chessboard corners not detected.")


# Takes a rectange and calculates which square its bottom middle is in
def point_locator(points, corners, cv_image):
    low1 = [0, 0]
    low2 = [0, 0]
    for i in range(0, 4):
        if points[i][1] > low1[1]:
            low1 = points[i]
            lowindex = i
    for i in range(0, 4):
        if points[i][1] > low2[1] and i != lowindex:
            low2 = points[i]
    mid_point = ((low1[0] + low2[0]) / 2, (low1[1] + low2[1]) / 2)
    column_translation = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
    cv2.drawMarker(cv_image, (int(mid_point[0]), int(mid_point[1])),  (0, 0, 255))
    # Iterate over each column first
    for j in range(0, 8):
        for i in range(0, 8):
            path = Path([corners[i, j], corners[i+1, j], corners[i+1, j+1], corners[i, j+1], corners[i, j]], 
                      [Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY], closed=True)
            if path.contains_point(mid_point):
                cv2.circle(cv_image, (int(corners[i, j][0]), int(corners[i, j][1])), 15, (255, 0, 255), 1)
                cv2.circle(cv_image, (int(corners[i+1, j][0]), int(corners[i+1, j][1])), 15, (255, 0, 255), 1)
                cv2.circle(cv_image, (int(corners[i+1, j+1][0]), int(corners[i+1, j+1][1])), 15, (255, 0, 255), 1)
                cv2.circle(cv_image, (int(corners[i, j+1][0]), int(corners[i, j+1][1])), 15, (255, 0, 255), 1)
                rospy.loginfo(column_translation[j] + str(8-i))


def video_publisher():
    # Initiate the node
    rospy.init_node('video_publisher_node')
    # Create a publisher
    pub = rospy.Publisher('video_topic', Image, queue_size=10)
    # Create an instance of CvBridge
    bridge = CvBridge()
    # Load the video
    video = cv2.VideoCapture(0)  # change this to your video file path
    rate = rospy.Rate(30)  # 30 Hz to match video frame rate
    (x1, y1) = (640 * random(), 480 * random())
    (x3, y3) = (640 * random(), 480 * random())
    (x2, y2) = (x1, y3)
    (x4, y4) = (x3, y1)
    while not rospy.is_shutdown():
        # Read a frame from the video
        ret, frame = video.read()
        if ret:
            corners = process_image(frame)
            if frame is None:    
                print("Could not open or find the image")
                return
            # Convert the frame to a ROS image message
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            # Publish the frame
            pub.publish(ros_image)
            if corners is not None:
                point_locator([(x1, y1), (x2, y2), (x3, y3), (x4, y4)], corners, frame)
            rate.sleep()
        else:
            break
        cv2.line(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
        cv2.line(frame, (int(x3), int(y3)), (int(x2), int(y2)), (0, 0, 255), 2)
        cv2.line(frame, (int(x3), int(y3)), (int(x4), int(y4)), (0, 0, 255), 2)
        cv2.line(frame, (int(x1), int(y1)), (int(x4), int(y4)), (0, 0, 255), 2)
        cv2.imshow("GRID1", frame)
        cv2.waitKey(1)

    video.release()

if __name__ == '__main__':
    try:
        video_publisher()
    except rospy.ROSInterruptException:
        pass