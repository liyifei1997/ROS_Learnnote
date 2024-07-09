#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(data):
	rospy.longinfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subsrciber("/kmriiwa/robot_state_publisher", String, callback)
	rospy.spin()

if __name__ == '__main__':
	listener()
