#!/usr/bin/env python3

import rospy
from kmriiwa_msgs.msg import JointPosition
from geometry_msgs.msg import Twist

def main():
    # Initialize the ROS node
    rospy.init_node('robot_control', anonymous=True)

    # Publishers for arm joints and mobile base
    arm_pub = rospy.Publisher('/kmriiwa/arm/command/JointPosition', JointPosition, queue_size=10)
    base_pub = rospy.Publisher('/kmriiwa/base/command/cmd_vel', Twist, queue_size=10)

    # Define the rate at which to publish (10 Hz)
    rate = rospy.Rate(10)

    # Create JointPosition message
    joint_pos = JointPosition()
    joint_pos.header.seq = 0
    joint_pos.header.stamp = rospy.Time.now()
    joint_pos.header.frame_id = ''
    joint_pos.a1 = 0.5
    joint_pos.a2 = 0.5
    joint_pos.a3 = -0.5
    joint_pos.a4 = 0.2
    joint_pos.a5 = 0.0
    joint_pos.a6 = 0.5
    joint_pos.a7 = -0.5

    # Create Twist message
    twist = Twist()
    twist.linear.x = -0.2
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

    while not rospy.is_shutdown():
        # Update the timestamp
        joint_pos.header.stamp = rospy.Time.now()

        # Publish the messages
        arm_pub.publish(joint_pos)
        base_pub.publish(twist)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
