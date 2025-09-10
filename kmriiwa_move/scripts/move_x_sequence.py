#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class XSequencer:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("/kmriiwa/base/state/odom", Odometry, self.odom_cb)
        self.cmd_pub  = rospy.Publisher("/kmriiwa/base/command/cmd_vel", Twist, queue_size=10)

        self.has_odom = False
        self.x = 0.0
        self.y = 0.0

        # 累计里程（欧氏距离积分）
        self.total_distance = 0.0
        self.prev_x = None
        self.prev_y = None

        # 阶段控制
        self.stage = 1  # 1: 前进1m; 2: 后退1m; 3: 完成
        self.stage_start_x = None  # 每阶段的起点x
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.stop_robot)

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.prev_x is None:
            self.prev_x, self.prev_y = self.x, self.y
            self.has_odom = True
            return

        # 增量里程
        dx = self.x - self.prev_x
        dy = self.y - self.prev_y
        self.total_distance += math.hypot(dx, dy)

        self.prev_x, self.prev_y = self.x, self.y
        self.has_odom = True

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def run(self):
        rospy.loginfo("Waiting for /kmriiwa/base/state/odom ...")
        while not rospy.is_shutdown() and not self.has_odom:
            self.rate.sleep()

        rospy.loginfo("Odom received. Start sequence.")

        twist = Twist()
        eps = 1e-3

        while not rospy.is_shutdown():
            # 初始化每阶段起点
            if self.stage_start_x is None:
                self.stage_start_x = self.x

            # 实时打印
            rospy.loginfo("Pos: (%.3f, %.3f) | Total distance: %.3f m | Stage: %d",
                          self.x, self.y, self.total_distance, self.stage)

            if self.stage == 1:
                # 目标：+x 方向 1 m
                moved = self.x - self.stage_start_x
                if moved < 1.0 - eps:
                    twist.linear.x = 0.2  # m/s
                else:
                    twist.linear.x = 0.0
                    self.cmd_pub.publish(twist)
                    rospy.loginfo("Stage 1 done. Moved: %.3f m", moved)
                    # 切到下一阶段
                    self.stage = 2
                    self.stage_start_x = None  # 让下一阶段重新记录起点
            elif self.stage == 2:
                # 目标：-x 方向 1 m
                moved_back = self.stage_start_x - self.x  # 往负向走，起点x - 当前x
                if moved_back < 1.0 - eps:
                    twist.linear.x = -0.1  # m/s
                else:
                    twist.linear.x = 0.0
                    self.cmd_pub.publish(twist)
                    rospy.loginfo("Stage 2 done. Moved back: %.3f m", moved_back)
                    self.stage = 3
            else:
                # 完成：停止并退出
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)
                rospy.loginfo("Sequence complete. Final Pos: (%.3f, %.3f), Total distance: %.3f m",
                              self.x, self.y, self.total_distance)
                break

            # 发布速度
            self.cmd_pub.publish(twist)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("move_x_sequence")
    node = XSequencer()
    node.run()
