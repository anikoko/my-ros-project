#!/usr/bin/env python3

import os
import time

import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

import cv2
import numpy as np

vehicle_name = os.environ['VEHICLE_NAME']
twist_topic = f"/{vehicle_name}/car_cmd_switch_node/cmd"
wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"

INITIAL_VELOCITY = 0.2
INITIAL_STEERING = 0
INITIAL_WHEEL_ROTATION = 0.2

class TwistControlNode(DTROS):
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(TwistControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        self._bridge = CvBridge()
        # Publishers
        self.publish_cmd = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        self.publish_wheels_cmd = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        # Subscribers
        self.red_line_detected_sub = rospy.Subscriber('red_line_detected', Bool, self.detect_red_line, queue_size=1)
        self.red_line_distance_sub = rospy.Subscriber('red_line_distance', Float32, self.update_distance, queue_size=1)

        # Variables
        self.visual_lane_following = True
        self.red_line_detected = False
        self.red_line_distance = float('inf')

        self.velocity = INITIAL_VELOCITY
        self.omega = INITIAL_STEERING
        self.wheel_rotation_left = INITIAL_WHEEL_ROTATION
        self.wheel_rotation_right = INITIAL_WHEEL_ROTATION
        self.has_stopped = False

        # PID controller parameters
        self.Kp = 0.005
        self.Ki = 0.001
        self.Kd = 0.001

        self.integral = 0
        self.previous_error = 0
        self.previous_time = time.time()

        

        # Register shutdown hook
        rospy.on_shutdown(self.on_shutdown)

    def run(self):
        # Publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)
        # wheels = self.getWheel(self.wheel_rotation_left, self.wheel_rotation_right)
        # twist = self.getTwist(self.velocity, self.omega)
        # self.publish_wheels_twist(wheels, twist)
        self.lane_following()

        while not rospy.is_shutdown():
            if self.red_line_detected and not self.has_stopped:
                self.visual_lane_following = False
                self.adjust_speed_pid()
                if self.red_line_distance <= 110:
                    self.has_stopped = True
                    print("Sleeping for 2 seconds")
                    self.stop()
                    rospy.sleep(2)
                    self.reset_bot_values()
            elif self.visual_lane_following:
                self.lane_following()
            else:
                self.reset_bot_values()
                self.reset_pid_values()
            rate.sleep()
            
            if self.has_stopped:
                self.has_stopped=False
                self.visual_lane_following = True
                self.turnRight()
                # self.lane_following()
                rospy.sleep(1)

    def getTwist(self, velocity, omega):
        twist = Twist2DStamped()
        twist.v = velocity
        twist.omega = omega
        return twist

    def getWheel(self, left, right):
        wheel = WheelsCmdStamped()
        wheel.vel_left = left
        wheel.vel_right = right
        return wheel

    def stop(self):
        self.velocity = 0
        self.omega = 0
        wheel = self.getWheel(0, 0)
        twist = self.getTwist(0, 0)
        self.publish_wheels_twist(wheel, twist)

    def publish_wheels_twist(self, wheels, twist):
        self.publish_wheels_cmd.publish(wheels)
        self.publish_cmd.publish(twist)

    # Subscriber function to detect red line
    def detect_red_line(self, msg):
        self.red_line_detected = msg.data

    # Subscriber function to get distance between red line and duckiebot
    def update_distance(self, msg):
        self.red_line_distance = msg.data

    # Subscriber function to get steering
    def update_steering(self, msg):
        self.omega = msg.data
        rospy.loginfo(f'Steering changed to {self.omega}')

    def adjust_speed_pid(self):
        current_time = time.time()
        time_delta = current_time - self.previous_time

        error = self.red_line_distance - 0  # Desired distance is 0
        self.integral += error * time_delta
        self.integral = max(min(self.integral, 2), -2)  # Prevent integral error to grow too much
        derivative = (error - self.previous_error) / time_delta

        self.velocity = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Ensure velocity does not go below zero
        self.velocity = max(0.0, self.velocity) / 10
        print(f"Velocity: {self.velocity}")
        # Update previous error and time
        self.previous_error = error
        self.previous_time = current_time

        twist = self.getTwist(self.velocity, self.omega)
        self.publish_cmd.publish(twist)

    def reset_bot_values(self):
        self.velocity = INITIAL_VELOCITY
        self.omega = INITIAL_STEERING
        self.wheel_rotation_left = INITIAL_WHEEL_ROTATION
        self.wheel_rotation_right = INITIAL_WHEEL_ROTATION
        twist = self.getTwist(self.velocity, self.omega)
        wheels = self.getWheel(self.wheel_rotation_left, self.wheel_rotation_right)
        self.publish_wheels_twist(wheels, twist)

    def reset_pid_values(self):
        self.integral = 0
        self.previous_error = 0
    
    def turnRight(self):
        # right 
        tw = self.getTwist(0.2, -3)
        wh = self.getWheel(0.6, 0.1)
        self.publish_wheels_twist(wh, tw)

        rospy.sleep(1)
        self.stop()
        # right 
        tw = self.getTwist(0.1, -2)
        wh = self.getWheel(0.5, 0.1)
        self.publish_wheels_twist(wh, tw)

        rospy.sleep(1)
        self.stop()

        # right        
        tw = self.getTwist(0.1, -1)
        wh = self.getWheel(0.2, 0.1)
        self.publish_wheels_twist(wh, tw)

        rospy.sleep(1)
        self.stop()

        # right        
        # tw = self.getTwist(0.1, -1)
        # wh = self.getWheel(0.2, 0.1)
        # self.publish_wheels_twist(wh, tw)

        # rospy.sleep(1)
        # self.stop()

        tw = self.getTwist(0.2, 0)
        wh = self.getWheel(0.2, 0.2)
        self.publish_wheels_twist(wh, tw)

        rospy.sleep(1)

    def lane_following(self):
        tw = self.getTwist(self.velocity, self.omega)
        wheels = self.getWheel(self.wheel_rotation_left, self.wheel_rotation_right)
        self.publish_wheels_twist(wheels, tw)

    def on_shutdown(self):
        self.stop()    

if __name__ == '__main__':
    print('started')
    node = TwistControlNode(node_name='twist_control_node')
    node.run()
    rospy.spin()