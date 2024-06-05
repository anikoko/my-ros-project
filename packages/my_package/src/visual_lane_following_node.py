#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
import cv2
from cv_bridge import CvBridge
import numpy as np
from typing import Tuple

HOST = os.environ['VEHICLE_NAME']
TOPIC_NAME = f'/{HOST}/camera_node/image/compressed'


def get_steer_matrix_left_lane_markings(shape: Tuple[int, int], scale_factor: float = 0.1) -> np.ndarray:
    height, width = shape
    mid_width = width // 2
    y_indices, x_indices = np.indices((height, mid_width))

    distance = np.sqrt((y_indices - height // 2) ** 2 + (x_indices - mid_width // 2) ** 2)
    weight = -scale_factor * np.exp(-distance / (height / 2))  # Higher reaction for closer pixels

    steer_matrix_left = np.zeros((height, width), dtype="float32")
    steer_matrix_left[:, :mid_width] = weight

    return steer_matrix_left


def get_steer_matrix_right_lane_markings(shape: Tuple[int, int], scale_factor: float = 0.1) -> np.ndarray:
    height, width = shape
    mid_width = width // 2
    y_indices, x_indices = np.indices((height, width - mid_width))

    distance = np.sqrt((y_indices - height // 2) ** 2 + (x_indices - (width - mid_width) // 2) ** 2)
    weight = scale_factor * np.exp(-distance / (height / 2))  # Higher reaction for closer pixels

    steer_matrix_right = np.zeros((height, width), dtype="float32")
    steer_matrix_right[:, mid_width:] = weight

    return steer_matrix_right


def detect_lane_markings(image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    # Convert image to HSV color space for color-based masking
    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # grayscale

    # Convolve the image with the Sobel operator (filter) to compute the numerical derivatives in the x and y directions
    sobelx = cv2.Sobel(img, cv2.CV_64F, 1, 0)
    sobely = cv2.Sobel(img, cv2.CV_64F, 0, 1)
    # Compute the magnitude of the gradients
    Gmag = np.sqrt(sobelx * sobelx + sobely * sobely)

    sigma = 10
    # Smooth the image using a Gaussian kernel
    img_gaussian_filter = cv2.GaussianBlur(img, (0, 0), sigma)

    # Convolve the image with the Sobel operator (filter) to compute the numerical derivatives in the x and y directions
    sobelx = cv2.Sobel(img_gaussian_filter, cv2.CV_64F, 1, 0)
    sobely = cv2.Sobel(img_gaussian_filter, cv2.CV_64F, 0, 1)

    # Compute the magnitude of the gradients
    Gmag = np.sqrt(sobelx * sobelx + sobely * sobely)
    threshold = 2
    mask_mag = (Gmag > threshold)

    white_lower_hsv = np.array([0, 0, 190.85])
    white_upper_hsv = np.array([179, 56.1, 255])
    yellow_lower_hsv = np.array([25.159, 99.45, 114.75])
    yellow_upper_hsv = np.array([27.84, 255, 255])

    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)

    # Let's create masks for the left- and right-halves of the image
    width = img.shape[1]
    mask_left = np.ones(sobelx.shape)
    mask_left[:, int(np.floor(width / 2)):width + 1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:, 0:int(np.floor(width / 2))] = 0

    # In the left-half image, we are interested in the right-half of the dashed yellow line, which corresponds to negative x- and y-derivatives
    # In the right-half image, we are interested in the left-half of the solid white line, which correspons to a positive x-derivative and a negative y-derivative
    # Generate a mask that identifies pixels based on the sign of their x-derivative
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)

    # Let's generate the complete set of masks, including those based on color
    mask_left_edge = mask_left * mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    mask_right_edge = mask_right * mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white

    return mask_left_edge, mask_right_edge

class VisualLaneFollowingNode(DTROS):
    def __init__(self, node_name):
        super(VisualLaneFollowingNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self._bridge = CvBridge()
        self.sub = rospy.Subscriber(TOPIC_NAME, CompressedImage, self.callback, queue_size=1, buff_size="10MB")

        # Parameters
        self.omega_max = 6.0
        self.v_0 = 0.1  # initial forward velocity
        self.steer_max = -1  # maximum steer value

        w, h = 640, 480

        left = 0.1
        right = 0.1
        self._roi = (int(left * w), int(right * w))

    def callback(self, msg):
        # convert JPEG bytes to CV image
        image = self._bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        # Resize the image to the desired dimensionsS
        height_original, width_original = image.shape[0:2]
        img_size = image.shape[0:2]
        if img_size[0] != width_original or img_size[1] != height_original:
            image = cv2.resize(image, tuple(reversed(img_size)), interpolation=cv2.INTER_NEAREST)

        (left, right) = self._roi
        image = image[:, left:-right, :]

        if self.is_shutdown:
            self.publish_command([0, 0])
            return

        shape = image.shape[0:2]
        steer_matrix_left_lm = get_steer_matrix_left_lane_markings(shape)
        steer_matrix_right_lm = get_steer_matrix_right_lane_markings(shape)
        # Call the user-defined function to get the masks for the left and right lane markings
        (lt_mask, rt_mask) = detect_lane_markings(image)
        steer = float(np.sum(lt_mask * steer_matrix_left_lm)) + float(np.sum(rt_mask * steer_matrix_right_lm))
        # now rescale from 0 to 1
        steer_scaled = np.sign(steer) * self.rescale(min(np.abs(steer), self.steer_max), 0, self.steer_max)
        u = [self.v_0, steer_scaled * self.omega_max]
        self.publish_command(u)

    def publish_command(self, u):
        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()
        # car_control_msg.v = u[0]  # v
        car_control_msg.omega = u[1]  # omega
        # rospy.loginfo(f"Published command: v={u[0]}, omega={u[1]}")

    def rescale(self, a: float, L: float, U: float):
        if np.allclose(L, U):
            return 0.0
        return (a - L) / (U - L)


if __name__ == '__main__':
    # create the node
    node = VisualLaneFollowingNode(node_name='visual_lane_following_node')
    # keep spinning
    rospy.spin()