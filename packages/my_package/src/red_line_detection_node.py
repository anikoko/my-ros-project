#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

import cv2
from cv_bridge import CvBridge
import numpy as np

HOST = os.environ['VEHICLE_NAME']
TOPIC_NAME = f'/{HOST}/camera_node/image/compressed'

class RedLineDetectionNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(RedLineDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # static parameters
        # bridge between OpenCV and ROS
        self._bridge = CvBridge()
        # create window
        self.sub = rospy.Subscriber(TOPIC_NAME, CompressedImage, self.callback, queue_size=1, buff_size="10MB")

        # construct subscriber
        self.remote_pub = rospy.Publisher('red_line_detected', Bool,queue_size=1)

    def callback(self, msg):
        # convert JPEG bytes to CV image
        img = self._bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        # vehicle mask
        # detected,patterns = cv2.findCirclesGrid(img, (7, 3)) 
        red_lower = np.array([70, 50, 200], dtype="uint8")
        red_upper = np.array([110, 90, 255], dtype="uint8")

        mask_red = cv2.inRange(img, red_lower,red_upper)

        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        img = img*mask_red

        # nonZeros = cv2.countNonZero(mask_red)
        detected = False
        h, w = img.shape
        if np.any(img[int(h/3):, int(w/3):] > 0):
            detected = True
        # cv.drawChessboardCorners(img_cv2,(7,3), patterns, True)
            # converting filtered result to CompressedImage
        # img_filtered_compressed = self.bridge.cv2_to_compressed_imgmsg(img_cv2)
            # publishing to 'image_pub'
        # self.pub.publish(img_filtered_compressed)
        self.remote_pub.publish(detected)


if __name__ == '__main__':
    # create the node
    node = RedLineDetectionNode(node_name='red_line_detection_node')
    # keep spinning
    rospy.spin()



