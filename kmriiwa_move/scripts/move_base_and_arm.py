#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kmriiwa_msgs.msg import JointPosition
import math
import time

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
        twist = Twist()
        self.cmd_pub.publish(twist)

    # ---- arm helpers ----
    def send_arm(self, a1, a2, a3, a4, a5, a6):
        msg = JointPosition()
        msg.header.stamp = rospy.Time.now()
        msg.a1 = float(a1); msg.a2 = float(a2); msg.a3 = float(a3)
        msg.a4 = float(a4); msg.a5 = float(a5); msg.a6 = float(a6)
        self.arm_pub.publish(msg)

    def arm_small_swing(self, duration_s=2.0):
        """
        在 A2、A3 上做小幅摆动（±0.1 rad），持续 duration_s 秒。
        """
        amp = 0.1
        f = 1.0  # 1 Hz
        t0 = time.time()
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and (time.time() - t0) < duration_s:
            t = time.time() - t0
            a2 = amp * math.sin(2*math.pi*f*t)
            a3 = amp * math.sin(2*math.pi*f*t + math.pi/2.0)
            self.send_arm(0.0, a2, a3, 0.0, 0.0, 0.0)
            r.sleep()

    def arm_home(self, hold_s=2.0):
        """
        回到全零位，维持 hold_s 秒。
        """
        t0 = time.time()
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and (time.time() - t0) < hold_s:
            self.send_arm(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            r.sleep()

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

            rospy.loginfo("Pos:(%.3f, %.3f)  Total:%.3f m  Stage:%d", self.x, self.y, self.total_dist, self.stage)

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
                    # 进入阶段2前先停一下
                    rospy.sleep(0.3)
            elif self.stage == 2:
                # arm 小幅摆动
                rospy.loginfo("Stage 2: arm small swing")
                self.arm_small_swing(duration_s=2.0)
                self.stage = 3
                self.stage_start_x = None
                rospy.sleep(0.2)
            elif self.stage == 3:
                # arm 归位
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
    rospy.init_node("kmr_base_arm_sequence", anonymous=True)
    node = BaseArmSequencer()
    node.run()
