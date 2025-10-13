#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math, time, actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from kmriiwa_msgs.msg import JointPosition
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ==== 配置（如需要可修改）====
TOPIC_ODOM     = "/kmriiwa/base/state/odom"
TOPIC_CMDVEL   = "/kmriiwa/base/command/cmd_vel"
TOPIC_JP_CMD   = "/kmriiwa/arm/command/JointPosition"
TOPIC_JOINTSTS = "/kmriiwa/arm/joint_states"
ACTION_FJT     = "/kmriiwa/arm/manipulator_controller/follow_joint_trajectory"

JOINT_NAMES    = ["A1","A2","A3","A4","A5","A6","A7"]  # 如果是6轴，改为6个名字
RATE_HZ        = 10
ANGLE_TOL_DEG  = 2.0   # 到达判定容差（度）
PRINT_HZ       = 10.0  # 关节角打印频率（Hz）
# =================================

def deg2rad(d): return d * math.pi / 180.0
def rad2deg(r): return r * 180.0 / math.pi

class KMRSeqValidated:
    def __init__(self):
        # pubs
        self.pub_cmd = rospy.Publisher(TOPIC_CMDVEL, Twist, queue_size=10)
        self.pub_jp  = rospy.Publisher(TOPIC_JP_CMD, JointPosition, queue_size=10)
        # subs
        self.sub_odom  = rospy.Subscriber(TOPIC_ODOM, Odometry, self.odom_cb)
        self.sub_jstat = rospy.Subscriber(TOPIC_JOINTSTS, JointState, self.joint_cb)
        # action
        self.fjt_client = actionlib.SimpleActionClient(ACTION_FJT, FollowJointTrajectoryAction)

        # odom
        self.have_odom = False
        self.x = 0.0; self.y = 0.0
        self.ref_x = 0.0; self.ref_y = 0.0

        # joints
        self.have_joints = False
        self.joint_map = {}  # name -> rad

        self.rate = rospy.Rate(RATE_HZ)
        rospy.on_shutdown(self.stop_base)

    # ---------------- ODOM ----------------
    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.have_odom = True

    def zero_odom(self):
        while not rospy.is_shutdown() and not self.have_odom:
            self.rate.sleep()
        self.ref_x = self.x
        self.ref_y = self.y
        rospy.loginfo("Odom zeroed at (%.3f, %.3f)", self.ref_x, self.ref_y)

    def rel_xy(self):
        return (self.x - self.ref_x, self.y - self.ref_y)

    # ---------------- JOINT STATES ----------------
    def joint_cb(self, msg: JointState):
        for n, p in zip(msg.name, msg.position):
            self.joint_map[n] = p
        if len(self.joint_map) > 0:
            self.have_joints = True

    def current_joints_deg(self):
        # 按 JOINT_NAMES 顺序返回角度（度）；若缺数据返回 None
        vals = []
        for n in JOINT_NAMES:
            if n not in self.joint_map:
                return None
            vals.append(rad2deg(self.joint_map[n]))
        return vals

    def print_joints_10hz_once(self):
        vals = self.current_joints_deg()
        if vals is not None:
            txt = ", ".join(["%s:%6.2f°" % (n, v) for n, v in zip(JOINT_NAMES, vals)])
            rospy.loginfo("[Joints] %s", txt)

    def wait_until_reached_deg(self, target_deg, tol_deg=ANGLE_TOL_DEG, timeout_s=10.0, keep_print=True):
        """
        阻塞等待到达目标角（度）。每 1/PRINT_HZ 秒打印一次关节角。
        target_deg: list(len=6或7)，不足补0
        """
        t_end = time.time() + timeout_s
        target = list(target_deg) + [0.0]*(max(0, len(JOINT_NAMES) - len(target_deg)))
        next_print = 0.0
        r = rospy.Rate(PRINT_HZ)

        while not rospy.is_shutdown():
            cur = self.current_joints_deg()
            now = time.time()

            if cur is not None:
                # 到达判定
                diffs = [abs(c - t) for c, t in zip(cur, target)]
                reached = all(d <= tol_deg for d in diffs)
                if keep_print and now >= next_print:
                    self.print_joints_10hz_once()
                    next_print = now + 1.0/PRINT_HZ
                if reached:
                    return True

            if now > t_end:
                rospy.logwarn("Wait timeout (%.1fs). Last diff(deg)=%s",
                              timeout_s,
                              ["%.2f" % d for d in (diffs if cur is not None else [])])
                return False

            r.sleep()

    # ---------------- BASE ----------------
    def stop_base(self):
        self.pub_cmd.publish(Twist())

    def move_x(self, distance_m, speed_mps, print_every=0.2):
        start_x_rel, _ = self.rel_xy()
        direction = 1.0 if distance_m >= 0 else -1.0
        speed = abs(speed_mps) * direction
        tw = Twist(); last_print = time.time()

        while not rospy.is_shutdown():
            moved = (self.rel_xy()[0] - start_x_rel)
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

        self.stop_base()
        rx, ry = self.rel_xy()
        rospy.loginfo("Final pose (relative): x=%.3f, y=%.3f", rx, ry)

    # ---------------- ARM: JointPosition ----------------
    def move_arm_point_deg(self, degs, hold_cmd_s=1.0, wait_tol_deg=ANGLE_TOL_DEG, wait_timeout_s=12.0):
        """
        通过 JointPosition 移动到目标；在发送期间持续打印 10Hz，
        并等待到达（达到才返回）。
        """
        vals = list(degs) + [0.0]*(max(0, len(JOINT_NAMES) - len(degs)))
        send_rate = rospy.Rate(20)
        t_end_send = time.time() + hold_cmd_s
        next_print = 0.0

        # 持续发送一小段时间，确保控制器接收，同时打印关节角
        while not rospy.is_shutdown() and time.time() < t_end_send:
            msg = JointPosition()
            msg.header.stamp = rospy.Time.now()
            msg.a1, msg.a2, msg.a3, msg.a4, msg.a5, msg.a6, msg.a7 = [deg2rad(v) for v in vals]
            self.pub_jp.publish(msg)

            now = time.time()
            if now >= next_print:
                self.print_joints_10hz_once()
                next_print = now + 1.0/PRINT_HZ

            send_rate.sleep()

        # 等待到位（阻塞）
        ok = self.wait_until_reached_deg(vals, tol_deg=wait_tol_deg, timeout_s=wait_timeout_s, keep_print=True)
        if not ok:
            rospy.logwarn("Arm did not reach target within tolerance (%.1f°).", wait_tol_deg)
        return ok

    # ---------------- ARM: Trajectory (Action) ----------------
    def move_arm_traj_deg(self, points_deg, times_s, wait_tol_deg=ANGLE_TOL_DEG, final_hold_check=True):
        """
        发送轨迹，并在执行过程中 10Hz 打印关节角。完成后再用到达判定校验末点。
        """
        # 等待 action server
        rospy.loginfo("Waiting for FollowJointTrajectory server...")
        if not self.fjt_client.wait_for_server(rospy.Duration(10.0)):
            rospy.logwarn("Action server not available, fallback to point-to-point.")
            for p in points_deg:
                self.move_arm_point_deg(p)
            return

        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        for p_deg, t in zip(points_deg, times_s):
            vals = list(p_deg) + [0.0]*(max(0, len(JOINT_NAMES) - len(p_deg)))
            pt = JointTrajectoryPoint()
            pt.positions = [deg2rad(v) for v in vals]
            pt.time_from_start = rospy.Duration(float(t))
            traj.points.append(pt)

        goal.trajectory = traj
        self.fjt_client.send_goal(goal)

        # 等待结果期间，10Hz 打印关节角
        r = rospy.Rate(PRINT_HZ)
        while not rospy.is_shutdown():
            state = self.fjt_client.get_state()
            # 1: ACTIVE, 0: PENDING, 3: SUCCEEDED, etc.
            self.print_joints_10hz_once()
            if state in [2,3,4,5,8,9]:  # done states
                break
            r.sleep()

        self.fjt_client.wait_for_result()
        rospy.loginfo("Trajectory result: %s", str(self.fjt_client.get_result()))

        # 末点到达校验
        if final_hold_check and len(points_deg) > 0:
            last = points_deg[-1]
            ok = self.wait_until_reached_deg(last, tol_deg=wait_tol_deg, timeout_s=8.0, keep_print=True)
            if not ok:
                rospy.logwarn("Final trajectory pose not within tolerance (%.1f°).", wait_tol_deg)

    # ---------------- SEQUENCE ----------------
    def run(self):
        # 等待里程计与关节状态
        rospy.loginfo("Waiting for odom and joint_states...")
        while not rospy.is_shutdown() and (not self.have_odom or not self.have_joints):
            self.rate.sleep()

        # (1) 里程计归0
        self.zero_odom()
        rospy.loginfo("Task 1 done")

        # (2) 机械臂到安全位
        safe = [-120, 60, 0, -60, 0, 60, 0]
        self.move_arm_point_deg(safe, hold_cmd_s=1.0)
        rospy.loginfo("Task 2 done")

        # (3) +x 方向 2m，0.2 m/s，实时打印坐标
        self.move_x(distance_m=+2.0, speed_mps=0.2, print_every=0.2)
        rospy.loginfo("Task 3 done")

        # (4) 机械臂到家 -> 再到安全位（每步都等待到达）
        home = [0, 0, 0, 0, 0, 0, 0]
        self.move_arm_point_deg(home, hold_cmd_s=1.0)
        self.move_arm_point_deg(safe, hold_cmd_s=1.0)
        rospy.loginfo("Task 4 done")

        # (5) −x 方向 1m，0.4 m/s
        self.move_x(distance_m=-1.0, speed_mps=0.4, print_every=0.2)
        rospy.loginfo("Task 5 done")

        # (6) Action：p1 -> p2 -> 安全位（执行中打印 + 末点到达校验）
        p1 = [90, 90, 0, -90, 0, 0, 0]
        p2 = [90, 90, 0,  90, 0, 0, 0]
        self.move_arm_traj_deg([p1, p2, safe], [2.0, 4.0, 6.0])
        rospy.loginfo("Task 6 done")

        # (7) −x 方向 1m，0.1 m/s
        self.move_x(distance_m=-1.0, speed_mps=0.1, print_every=0.2)
        rospy.loginfo("Task 7 done")

        rospy.loginfo("All tasks finished.")

if __name__ == "__main__":
    rospy.init_node("kmr_sequence_validated", anonymous=True)
    KMRSeqValidated().run()
