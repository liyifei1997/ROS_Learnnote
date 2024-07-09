#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

def move_square():
    # Initialize the ROS node
    rospy.init_node('move_square_node', anonymous=True)

    # Create a publisher to publish Twist messages to control the robot's motion
    cmd_vel_pub = rospy.Publisher('/kmriiwa/base/command/cmd_vel', Twist, queue_size=10)

    # Set the rate at which to publish messages (10 Hz in this example)
    rate = rospy.Rate(10)

    # Create a Twist message to send velocity commands
    move_cmd = Twist()

    # Set the linear and angular velocities for moving in a square
    linear_velocity = 0.2  # Adjust as needed
    angular_velocity = 0.2  # Adjust as needed

    while not rospy.is_shutdown():
        # Move forward for a specified duration (adjust as needed)
        move_cmd.linear.x = linear_velocity
        move_cmd.angular.z = 0.0
        cmd_vel_pub.publish(move_cmd)
        rospy.sleep(5)  # Move forward for 2 seconds (adjust as needed)

        # Stop the robot
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        cmd_vel_pub.publish(move_cmd)
        rospy.sleep(5)  # Pause for 1 second (adjust as needed)

        # Rotate the robot by a specified angle (90 degrees for a square)
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = angular_velocity
        cmd_vel_pub.publish(move_cmd)
        rospy.sleep(5)  # Rotate for 90 degrees (adjust as needed)

        # Stop the robot
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        cmd_vel_pub.publish(move_cmd)
        rospy.sleep(5)  # Pause for 1 second (adjust as needed)

        # Repeat the square pattern or exit as needed
        # To exit, you can use a condition or set a specific number of repetitions

        rate.sleep()

if __name__ == '__main__':
    try:
        move_square()
    except rospy.ROSInterruptException:
        pass
