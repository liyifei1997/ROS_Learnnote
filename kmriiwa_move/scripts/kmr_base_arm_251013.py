#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math, time, actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kmriiwa_msgs.msg import JointPosition
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ==== 配置（如需要可修改）====
TOPIC_ODOM   = "/kmriiwa/base/state/odom"
TOPIC_CMDVEL = "/kmriiwa/base/command/cmd_vel"
TOPIC_JP_CMD = "/kmriiwa/arm/command/JointPosition"
ACTION_FJT   = "/kmriiwa/arm/manipulator_controller/follow_joint_trajectory"
JOINT_NAMES  = ["A1","A2","A3","A4","A5","A6","A7"]  # 如果是6轴，删掉A7并相应修改
RATE_HZ      = 10
# =================================

def deg2rad(d): return d * math.pi / 180.0
def rad2deg(r): return r * 180.0 / math.pi

class KMRSeq:
    def __init__(self):
        # pub/sub
        self.pub_cmd = rospy.Publisher(TOPIC_CMDVEL, Twist, queue_size=10)
        self.pub_jp  = rospy.Publisher(TOPIC_JP_CMD, JointPosition, queue_size=10)
        self.sub_odom= rospy.Subscriber(TOPIC_ODOM, Odometry, self.odom_cb)

        # action client
        self.fjt_client = actionlib.SimpleActionClient(ACTION_FJT, FollowJointTrajectoryAction)

        # odom state
        self.have_odom = False
        self.x = 0.0; self.y = 0.0
        self.ref_x = 0.0; self.ref_y = 0.0

        self.rate = rospy.Rate(RATE_HZ)
        rospy.on_shutdown(self.stop_base)

    # ---------------- ODOM ----------------
    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.have_odom = True

    def zero_odom(self):
        # 相对归零：把当前坐标记为原点
        while not rospy.is_shutdown() and not self.have_odom:
            self.rate.sleep()
        self.ref_x = self.x
        self.ref_y = self.y
        rospy.loginfo("Odom zeroed at (%.3f, %.3f)", self.ref_x, self.ref_y)

    def rel_xy(self):
        return (self.x - self.ref_x, self.y - self.ref_y)

    # ---------------- BASE ----------------
    def stop_base(self):
        self.pub_cmd.publish(Twist())

    def move_x(self, distance_m, speed_mps, print_every=0.2):
        """
        沿 x 轴移动给定距离；distance_m 可正可负；恒速；实时打印相对坐标
        """
        start_x_rel, _ = self.rel_xy()
        direction = 1.0 if distance_m >= 0 else -1.0
        speed = abs(speed_mps) * direction

        tw = Twist()
        last_print = time.time()

        while not rospy.is_shutdown():
            moved = (self.rel_xy()[0] - start_x_rel)  # 相对位移
            if direction > 0 and moved >= distance_m: break
            if direction < 0 and moved <= distance_m: break

            tw.linear.x = speed
            self.pub_cmd.publish(tw)

            now = time.time()
            if now - last_print >= print_every:
                rx, ry = self.rel_xy()
                rospy.loginfo("Current pose (relative): x=%.3f, y=%.3f", rx, ry)
                last_print = now

            self.rate.sleep()

        # 停车
        self.stop_base()
        rx, ry = self.rel_xy()
        rospy.loginfo("Final pose (relative): x=%.3f, y=%.3f", rx, ry)

    # ---------------- ARM: point command ----------------
    def send_joint_position_deg(self, degs, hold_s=1.5):
        """
        通过 JointPosition 连续发布一段时间，帮助控制器接收
        degs: [a1..a7] (度)；如果是6轴，长度6
        """
        # 兼容 6/7 轴
        vals = list(degs) + [0.0]*(max(0, 7 - len(degs)))
        r = rospy.Rate(20)
        t0 = time.time()
        while not rospy.is_shutdown() and (time.time() - t0) < hold_s:
            msg = JointPosition()
            msg.header.stamp = rospy.Time.now()
            # 角度转弧度发送；若驱动期望度，改成直接赋值
            msg.a1 = deg2rad(vals[0]); msg.a2 = deg2rad(vals[1]); msg.a3 = deg2rad(vals[2])
            msg.a4 = deg2rad(vals[3]); msg.a5 = deg2rad(vals[4]); msg.a6 = deg2rad(vals[5])
            msg.a7 = deg2rad(vals[6])
            self.pub_jp.publish(msg)
            r.sleep()

    # ---------------- ARM: trajectory (Action) ----------------
    def send_trajectory_deg(self, points_deg, times_s):
        """
        points_deg: [[deg...], ...]  每个点 6或7轴（不足补0）
        times_s:    [t1, t2, ...]    递增
        """
        rospy.loginfo("Waiting for FollowJointTrajectory server...")
        if not self.fjt_client.wait_for_server(rospy.Duration(10.0)):
            rospy.logwarn("Action server not available, fallback to point commands.")
            # 兜底：逐点发
            for p in points_deg:
                self.send_joint_position_deg(p, hold_s=1.5)
            return

        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES  # 若6轴，请改成6个名字

        for p_deg, t in zip(points_deg, times_s):
            pt = JointTrajectoryPoint()
            # 兼容 6/7 轴
            vals = list(p_deg) + [0.0]*(max(0, len(JOINT_NAMES) - len(p_deg)))
            pt.positions = [deg2rad(v) for v in vals]
            pt.time_from_start = rospy.Duration(float(t))
            traj.points.append(pt)

        goal.trajectory = traj
        self.fjt_client.send_goal(goal)
        self.fjt_client.wait_for_result()
        rospy.loginfo("Trajectory result: %s", str(self.fjt_client.get_result()))

    # ---------------- SEQUENCE ----------------
    def run(self):
        # 等待里程计
        rospy.loginfo("Waiting for odom messages...")
        while not rospy.is_shutdown() and not self.have_odom:
            self.rate.sleep()

        # (1) 里程计归0
        self.zero_odom()
        rospy.loginfo("Task 1 done")

        # (2) 机械臂到安全位
        safe = [-120, 60, 0, -60, 0, 60, 0]
        self.send_joint_position_deg(safe, hold_s=2.0)
        rospy.loginfo("Task 2 done")

        # (3) +x 方向 2m，0.2 m/s，实时打印坐标
        self.move_x(distance_m=+2.0, speed_mps=0.2, print_every=0.2)
        rospy.loginfo("Task 3 done")

        # (4) 机械臂到家，再到安全位
        home = [0, 0, 0, 0, 0, 0, 0]
        self.send_joint_position_deg(home, hold_s=2.0)
        self.send_joint_position_deg(safe, hold_s=2.0)
        rospy.loginfo("Task 4 done")

        # (5) −x 方向 1m，0.4 m/s，实时打印坐标
        self.move_x(distance_m=-1.0, speed_mps=0.4, print_every=0.2)
        rospy.loginfo("Task 5 done")

        # (6) Action：p1 -> p2 -> 安全位
        p1 = [90, 90, 0, -90, 0, 0, 0]
        p2 = [90, 90, 0,  90, 0, 0, 0]
        self.send_trajectory_deg([p1, p2, safe], [2.0, 4.0, 6.0])
        rospy.loginfo("Task 6 done")

        # (7) −x 方向 1m，0.1 m/s，实时打印坐标
        self.move_x(distance_m=-1.0, speed_mps=0.1, print_every=0.2)
        rospy.loginfo("Task 7 done")

        rospy.loginfo("All tasks finished.")

if __name__ == "__main__":
    rospy.init_node("kmr_sequence", anonymous=True)
    KMRSeq().run()
