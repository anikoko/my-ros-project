#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped 
from std_msgs.msg import String, Bool


# Twist command for controlling the linear and angular velocity of the frame
# VELOCITY = 0.1  # linear vel    , in m/s    , forward (+)
# OMEGA = 0    # angular vel   , rad/s     , counter clock wise (+)

vehicle_name = os.environ['VEHICLE_NAME']
twist_topic = f"/{vehicle_name}/car_cmd_switch_node/cmd"
wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"


class TwistControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(TwistControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        # form the message
        # construct publisher
        wheels = self.getWheel(0.2, 0.2)
        twist = self.getTwist(0.1,0)
        # self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        self.publish_cmd = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)
        self.publish_wheels_cmd = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        self.red_line_detected = rospy.Subscriber(
            'red_line_detected',
            Bool,
            self.detect_red_line,
            queue_size=1
        )

        self.red_line_detect = False

    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)
        wheels = self.getWheel(0.2, 0.2)
        twist = self.getTwist(0.1,0)
        self.move(wheels, twist)

        while not rospy.is_shutdown():
            # self._publisher.publish(message)
            if self.red_line_detect:
                self.stop()
                rospy.sleep(1)
            rate.sleep()

    def on_shutdown(self):
        self.stop()

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
        wheel = self.getWheel(0,0)
        twist = self.getTwist(0,0)
        self.move(wheel, twist)
    
    def move(self, wheel, twist):
        # self.publish_wheels_cmd.publish(wheel)
        self.publish_cmd.publish(twist)
    
    def detect_red_line(self, msg):
        rospy.loginfo(f'detect: {msg}')
        self.red_line_detect = msg.data



if __name__ == '__main__':
    print('started')
    # create the node
    node = TwistControlNode(node_name='twist_control_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()