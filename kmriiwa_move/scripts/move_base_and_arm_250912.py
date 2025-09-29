#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kmriiwa_msgs.msg import JointPosition
import math
import time

def deg2rad(d): return d * math.pi / 180.0

class BaseArmSequencer:
    def __init__(self):
        # pubs/subs
        self.odom_sub = rospy.Subscriber("/kmriiwa/base/state/odom", Odometry, self.odom_cb)
        self.cmd_pub  = rospy.Publisher("/kmriiwa/base/command/cmd_vel", Twist, queue_size=10)
        self.arm_pub  = rospy.Publisher("/kmriiwa/arm/command/JointPosition", JointPosition, queue_size=10)

        # odom state
        self.has_odom = False
        self.x = 0.0
        self.y = 0.0
        self.prev_x = None
        self.prev_y = None
        self.total_dist = 0.0

        # stage control
        self.stage = 1
        self.stage_start_x = None
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.stop_robot)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if self.prev_x is None:
            self.prev_x, self.prev_y = self.x, self.y
            self.has_odom = True
            return
        dx = self.x - self.prev_x
        dy = self.y - self.prev_y
        self.total_dist += math.hypot(dx, dy)
        self.prev_x, self.prev_y = self.x, self.y
        self.has_odom = True

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    # ---- arm helpers ----
    def send_arm(self, a1, a2, a3, a4, a5, a6, repeat=1, hz=20.0):
        """
        发布关节目标。repeat>1 用于“持续一小段时间”保证命令被控制器接收。
        """
        r = rospy.Rate(hz)
        for _ in range(max(1, repeat)):
            msg = JointPosition()
            msg.header.stamp = rospy.Time.now()
            msg.a1 = float(a1); msg.a2 = float(a2); msg.a3 = float(a3)
            msg.a4 = float(a4); msg.a5 = float(a5); msg.a6 = float(a6)
            self.arm_pub.publish(msg)
            r.sleep()

    def arm_pose_deg(self, j1, j2, j3, j4, j5, j6, hold_s=1.5):
        """
        用角度(°)指定姿态；内部转弧度再发。保持 hold_s 秒以确保到位。
        """
        a1 = deg2rad(j1); a2 = deg2rad(j2); a3 = deg2rad(j3)
        a4 = deg2rad(j4); a5 = deg2rad(j5); a6 = deg2rad(j6)
        t0 = time.time()
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and (time.time() - t0) < hold_s:
            self.send_arm(a1, a2, a3, a4, a5, a6)
            r.sleep()

    def arm_home(self, hold_s=1.5):
        self.arm_pose_deg(0, 0, 0, 0, 0, 0, hold_s=hold_s)

    # ---- main sequence ----
    def run(self):
        rospy.loginfo("Waiting for /kmriiwa/base/state/odom ...")
        while not rospy.is_shutdown() and not self.has_odom:
            self.rate.sleep()
        rospy.loginfo("Odom received. Start sequence.")

        twist = Twist()
        eps = 1e-3

        while not rospy.is_shutdown():
            if self.stage_start_x is None:
                self.stage_start_x = self.x

            rospy.loginfo("Pos:(%.3f, %.3f)  Total:%.3f m  Stage:%d",
                          self.x, self.y, self.total_dist, self.stage)

            if self.stage == 1:
                # +x 方向 1 m @ 0.2 m/s
                moved = self.x - self.stage_start_x
                if moved < 1.0 - eps:
                    twist.linear.x = 0.2
                else:
                    twist.linear.x = 0.0
                    self.cmd_pub.publish(twist)
                    rospy.loginfo("Stage 1 done. Moved: %.3f m", moved)
                    self.stage = 2
                    self.stage_start_x = None
                    rospy.sleep(0.3)

            elif self.stage == 2:
                # Arm: joint1=+90°, joint2=+90°, joint4=-90°（其余 0）
                rospy.loginfo("Stage 2: arm pose (j1=+90°, j2=+90°, j4=-90°)")
                self.arm_pose_deg(90, 90, 0, -90, 0, 0, hold_s=2.0)
                self.stage = 3
                self.stage_start_x = None
                rospy.sleep(0.2)

            elif self.stage == 3:
                # Arm 归位
                rospy.loginfo("Stage 3: arm home")
                self.arm_home(hold_s=2.0)
                self.stage = 4
                self.stage_start_x = None
                rospy.sleep(0.2)

            elif self.stage == 4:
                # −x 方向 1 m @ 0.1 m/s
                moved_back = self.stage_start_x - self.x
                if moved_back < 1.0 - eps:
                    twist.linear.x = -0.1
                else:
                    twist.linear.x = 0.0
                    self.cmd_pub.publish(twist)
                    rospy.loginfo("Stage 4 done. Moved back: %.3f m", moved_back)
                    self.stage = 5

            else:
                # 完成
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)
                rospy.loginfo("Sequence complete. Final Pos:(%.3f, %.3f)  Total: %.3f m",
                              self.x, self.y, self.total_dist)
                break

            # 发布底座速度
            self.cmd_pub.publish(twist)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("kmr_base_arm_sequence_pose", anonymous=True)
    node = BaseArmSequencer()
    node.run()
