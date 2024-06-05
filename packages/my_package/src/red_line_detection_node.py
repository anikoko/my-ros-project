#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
import numpy as np
import cv2

HOST = os.environ['VEHICLE_NAME']
TOPIC_NAME = f'/{HOST}/camera_node/image/compressed'


class RedLineDetectionNode(DTROS):
    def __init__(self, node_name):
        super(RedLineDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self._bridge = CvBridge()

        # Subscriber
        self.sub = rospy.Subscriber(TOPIC_NAME, CompressedImage, self.callback, queue_size=1, buff_size="10MB")

        # Publishers
        self.remote_pub = rospy.Publisher('red_line_detected', Bool, queue_size=1)
        self.distance_pub = rospy.Publisher('red_line_distance', Float32, queue_size=1)

    def callback(self, msg):
        img = self._bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

        red_lower = np.array([0, 0, 150], dtype="uint8")
        red_upper = np.array([100, 100, 255], dtype="uint8")

        mask_red = cv2.inRange(img, red_lower, red_upper)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_gray = img_gray * mask_red

        detected = False
        distance = float('inf')  # Default distance is infinity
        h, w = img_gray.shape

        # Only consider the lower third of the image for red line detection
        lower_third = img_gray[int(2 * h / 3):, :]
        # Get the coordinates of the red pixels
        red_pixels = np.where(lower_third > 0)

        if red_pixels[0].size > 0:
            detected = True
            distance = h - (red_pixels[0].min() + int(2 * h / 3)) # Calculate distance based on the nearest red pixel

        distance = max(0, distance)

        rospy.loginfo(f'Red line detected: {detected}, Distance: {distance}')
        self.remote_pub.publish(Bool(detected))
        self.distance_pub.publish(Float32(distance))


if __name__ == '__main__':
    # create the node
    node = RedLineDetectionNode(node_name='red_line_detection_node')
    # keep spinning
    rospy.spin()
