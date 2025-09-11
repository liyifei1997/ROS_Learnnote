#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_base(linear_x, angular_z, duration):
    pub = rospy.Publisher('/kmriiwa/base/command/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)  # Ensure the publisher is properly registered
    rate = rospy.Rate(100)  # 10 Hz control loop

    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z

    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < duration:
        pub.publish(twist)
        rate.sleep()

    # Stop the robot
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)

if __name__ == '__main__':
    try:
        rospy.init_node('base_control_script', anonymous=True)
        rospy.sleep(1)  # Ensure ROS setup is complete

        # Define movement parameters
        linear_speed = 1  # meters per second 0.2 /0.4/ 0.6/ 0.8/1.0
        distance = 0.5  # meters
        duration = distance / linear_speed  # Time required to move

        # Parameters for rotation
        angular_speed = 1       # rad/s (adjust if needed)
        angle = 3.1416           # 90 degrees in radians
        duration_rotate = abs(angle) / angular_speed  # time to rotate 90°

        # Define movement parameters
        linear_speed_4 = 0.4  # meters per second 0.2 /0.4/ 0.6/ 0.8/1.0
        distance_4 = 0.5  # meters
        duration_4 = distance_4 / linear_speed_4  # Time required to move


        total_distance = 0    #total linear distance traveled
        total_angulars = 0    #total angulars traveled

        for i in range(50):
            
            rospy.loginfo(f"Total meters traveled so far: {total_distance:.2f} m")
            rospy.loginfo(f"Iteration {i+1}: Moving +x")
            move_base(linear_speed, 0, duration+0.4)
            total_distance += distance

            rospy.loginfo(f"Total meters traveled so far: {total_distance:.2f} m")
            rospy.loginfo(f"Iteration {i+1}: Moving +x")
            move_base(linear_speed_4, 0, duration_4*2+0.3)
            total_distance += distance*2


            rospy.loginfo(f"Total meters traveled so far: {total_distance:.2f} m")
            rospy.loginfo(f"Iteration {i+1}: Moving +x")
            move_base(linear_speed, 0, duration*3+0.3)
            total_distance += distance*3

            rospy.sleep(1)  # Pause 

            rospy.loginfo(f"Total angulars traveled so far: {total_angulars:.2f} rad")
            rospy.loginfo(f"Iteration {i+1}: Rotate 90° left")
            move_base(0, angular_speed, duration_rotate)
            total_angulars += angle

            rospy.sleep(1)  # Pause 

            rospy.loginfo(f"Total meters traveled so far: {total_distance:.2f} m")
            rospy.loginfo(f"Iteration {i+1}: Moving +x")
            move_base(linear_speed, 0, duration*2+0.5)
            total_distance += distance*2


            rospy.loginfo(f"Total meters traveled so far: {total_distance:.2f} m")
            rospy.loginfo(f"Iteration {i+1}: Moving +x")
            move_base(linear_speed_4, 0, duration_4*3+0.3)
            total_distance += distance*3


            rospy.loginfo(f"Total meters traveled so far: {total_distance:.2f} m")
            rospy.loginfo(f"Iteration {i+1}: Moving +x")
            move_base(linear_speed, 0, duration+0.3)
            total_distance += distance


            rospy.loginfo(f"Total meters traveled so far: {total_distance:.2f} m")
            rospy.loginfo(f"Iteration {i+1}: Moving +x")
            move_base(-linear_speed, 0, duration+0.4)
            total_distance += distance


            rospy.loginfo(f"Total meters traveled so far: {total_distance:.2f} m")
            rospy.loginfo(f"Iteration {i+1}: Moving +x")
            move_base(-linear_speed_4, 0, duration_4*3+0.3)
            total_distance += distance*3


            rospy.loginfo(f"Total meters traveled so far: {total_distance:.2f} m")
            rospy.loginfo(f"Iteration {i+1}: Moving +x")
            move_base(-linear_speed, 0, duration*2+0.4)
            total_distance += distance*2

            rospy.sleep(1)  # Pause 

            rospy.loginfo(f"Total angulars traveled so far: {total_angulars:.2f} rad")
            rospy.loginfo(f"Iteration {i+1}: Rotate 90° right")
            move_base(0, -angular_speed, duration_rotate)
            total_angulars += angle

            rospy.sleep(1)  # Pause 

            rospy.loginfo(f"Total meters traveled so far: {total_distance:.2f} m")
            rospy.loginfo(f"Iteration {i+1}: Moving +x")
            move_base(-linear_speed, 0, duration*3+0.3)
            total_distance += distance*3


            rospy.loginfo(f"Total meters traveled so far: {total_distance:.2f} m")
            rospy.loginfo(f"Iteration {i+1}: Moving +x")
            move_base(-linear_speed_4, 0, duration_4*2+0.3)
            total_distance += distance*2


            rospy.loginfo(f"Total meters traveled so far: {total_distance:.2f} m")
            rospy.loginfo(f"Iteration {i+1}: Moving +x")
            move_base(-linear_speed, 0, duration+0.4)
            total_distance += distance

            rospy.sleep(1)  # Pause 


    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted, shutting down.")
