#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import math
import numpy as np
import matplotlib.pyplot as pl
import time


class Controller:

    def __init__(self):
        # Initialize the ROS node and create a subscriber and publisher
        rospy.init_node("pid", anonymous=True)
        self.sub = rospy.Subscriber("odom", Odometry, self.callback1)
        self.pub = rospy.Publisher("atom/cmd_vel", Twist, queue_size=10)

        # Initialize variables
        vel_msg = Twist()
        rate = rospy.Rate(10)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.max_omega = 0.2
        self.min_omega = -0.2
        self.max_speed = 0.2
        self.min_speed = -0.2
        self.rot_flag = True

        # PID controller gains for distance and angle control
        self.kp_dist = 0.002
        self.ki_dist = 0.001
        self.kd_dist = 0.001
        self.kp_ang = 0.01
        self.ki_ang = 0.001
        self.kd_ang = 0.01

        counter = 0

        while not rospy.is_shutdown():
            if counter % 20 == 0:
                # Print current velocity, angular velocity and coordinates every 2 seconds
                print("Current velocity: ", vel_msg.linear.x)
                print("Current angular velocity: ", vel_msg.angular.z)
                print("Current coordinates: ", self.x, self.y)

                # Stop the robot and prompt the user for new velocities every 2 seconds
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.pub.publish(vel_msg)
                vel_msg.linear.x = float(input("Enter linear velocity: "))
                vel_msg.angular.z = float(input("Enter angular velocity: "))

            # Publish the velocities to the cmd_vel topic and sleep for the specified rate
            print("cmd_vel vals", vel_msg.linear.x, vel_msg.angular.z)
            self.pub.publish(vel_msg)
            rate.sleep()
            counter += 1

    def callback1(self, data):
        # Extract the orientation and position from the Odometry message and convert the orientation to Euler angles (yaw_z)
        ox, oy, oz, ow = data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w
        x, y = data.pose.pose.position.x, data.pose.pose.position.y
        yaw_z = self.quaternion2euler(ox, oy, oz, ow)
        yaw_z = math.degrees(yaw_z)

        # Update the current position and orientation of the robot (self.x, self.y and self.theta)
        self.x, self.y, self.theta = x, y, yaw_z

    @staticmethod
    def quaternion2euler(ox, oy, oz, ow):
        # Convert quaternion to Euler angles (yaw_z)
        t3 = +2.0 * (ow * oz + ox * oy)
        t4 = +1.0 - 2.0 * (oy * oy + oz * oz)
        yaw_z = math.atan2(t3, t4)
        return yaw_z


if __name__ == '__main__':
    try:
        controller = Controller()
    except rospy.ROSInterruptException:
        pass
