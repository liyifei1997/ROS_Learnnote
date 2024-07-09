#!/usr/bin/env python3

import rospy
from kmriiwa_msgs.msg import JointPosition
from geometry_msgs.msg import Twist
import threading

def publish_arm_commands(rate, arm_pub, joint_pos):
        joint_pos.header.stamp = rospy.Time.now()
        arm_pub.publish(joint_pos)
        

def publish_base_commands(rate, base_pub, twist):
        base_pub.publish(twist)
        

def main():
    rospy.init_node('robot_control', anonymous=True)

    arm_pub = rospy.Publisher('/kmriiwa/arm/command/JointPosition', JointPosition, queue_size=10)
    base_pub = rospy.Publisher('/kmriiwa/base/command/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)

    joint_pos = JointPosition()
    joint_pos.header.seq = 0
    joint_pos.header.stamp = rospy.Time.now()
    joint_pos.header.frame_id = ''
    joint_pos.a1 = 0.6
    joint_pos.a2 = 0.5
    joint_pos.a3 = -0.5
    joint_pos.a4 = 0.2
    joint_pos.a5 = 0.0
    joint_pos.a6 = 0.5
    joint_pos.a7 = -0.6
    '''joint_pos.a1 = 0
    joint_pos.a2 = 0
    joint_pos.a3 = 0
    joint_pos.a4 = 0
    joint_pos.a5 = 0
    joint_pos.a6 = 0
    joint_pos.a7 = 0'''

    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 1
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0

    rospy.Timer(rospy.Duration(0.1), publish_arm_commands, (arm_pub, joint_pos))
    rospy.Timer(rospy.Duration(0.1), publish_base_commands, (base_pub, twist))

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
