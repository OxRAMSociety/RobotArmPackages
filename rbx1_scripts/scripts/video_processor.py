#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

def video_publisher():
    # Initiate the node
    rospy.init_node('video_publisher_node')

    # Create a publisher
    pub = rospy.Publisher('video_topic', Image, queue_size=10)

    # Create an instance of CvBridge
    bridge = CvBridge()

    # Load the video
    video = cv2.VideoCapture('path_to_your_video_file.mp4')  # change this to your video file path

    rate = rospy.Rate(30)  # 30 Hz to match typical video frame rate

    while not rospy.is_shutdown():
        # Read a frame from the video
        ret, frame = video.read()

        if ret:
            # Convert the frame to a ROS image message
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")

            # Publish the frame
            pub.publish(ros_image)

            rate.sleep()
        else:
            break

    video.release()

if __name__ == '__main__':
    try:
        video_publisher()
    except rospy.ROSInterruptException:
        pass
