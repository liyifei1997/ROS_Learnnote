#!/usr/bin/env python3
import rospy, math, time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kmriiwa_msgs.msg import JointPosition

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def deg2rad(d): return d * math.pi / 180.0

class BaseArmSequencer:
    def __init__(self):
        # pubs/subs
        self.odom_sub = rospy.Subscriber("/kmriiwa/base/state/odom", Odometry, self.odom_cb)
        self.cmd_pub  = rospy.Publisher("/kmriiwa/base/command/cmd_vel", Twist, queue_size=10)
        self.arm_pub  = rospy.Publisher("/kmriiwa/arm/command/JointPosition", JointPosition, queue_size=10)

        # action client for FollowJointTrajectory
        self.traj_cli = actionlib.SimpleActionClient(
            "/kmriiwa/arm/manipulator_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )

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

    # ---------- simple position (JointPosition) ----------
    def send_arm_pos_rad(self, a1, a2, a3, a4, a5, a6, a7, repeat=1, hz=20.0):
        r = rospy.Rate(hz)
        for _ in range(max(1, repeat)):
            msg = JointPosition()
            msg.header.stamp = rospy.Time.now()
            msg.a1 = float(a1); msg.a2 = float(a2); msg.a3 = float(a3)
            msg.a4 = float(a4); msg.a5 = float(a5); msg.a6 = float(a6); msg.a7 = float(a7)
            self.arm_pub.publish(msg)
            r.sleep()

    def arm_pose_deg(self, j1, j2, j3, j4, j5, j6, j7=0.0, hold_s=1.5):
        a = [deg2rad(j) for j in [j1,j2,j3,j4,j5,j6,j7]]
        t0 = time.time()
        r = rospy.Rate(20)
        while not rospy.is_shutdown() and (time.time() - t0) < hold_s:
            self.send_arm_pos_rad(*a)
            r.sleep()

    def arm_home(self, hold_s=1.5):
        self.arm_pose_deg(-120,60,0,-60,0,60,0, hold_s=hold_s)

    # ---------- trajectory control (FollowJointTrajectory) ----------
    def wait_traj_server(self, timeout=10.0):
        ok = self.traj_cli.wait_for_server(rospy.Duration(timeout))
        if not ok:
            rospy.logwarn("FollowJointTrajectory server not available.")
        return ok

    def send_traj_deg(self, joint_names, points_deg, times_s, wait=True):
        """
        joint_names: list[str] 如 ["A1","A2","A3","A4","A5","A6","A7"]
        points_deg:  list[list[deg]]，每个点的关节角(度)
        times_s:     list[float]，每个点的 t_from_start（必须严格递增）
        """
        assert len(points_deg) == len(times_s) and len(points_deg) > 0
        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = joint_names

        for p_deg, t in zip(points_deg, times_s):
            pt = JointTrajectoryPoint()
            # 角度转弧度（若控制器期望度，删掉下面一行的转换）
            pt.positions = [deg2rad(v) for v in p_deg]
            pt.time_from_start = rospy.Duration(float(t))
            traj.points.append(pt)

        goal.trajectory = traj
        self.traj_cli.send_goal(goal)
        if wait:
            self.traj_cli.wait_for_result()
            res = self.traj_cli.get_result()
            rospy.loginfo("Trajectory result: %s", str(res))

    # ---------- main sequence ----------
    def run(self):
        rospy.loginfo("Waiting for /kmriiwa/base/state/odom ...")
        while not rospy.is_shutdown() and not self.has_odom:
            self.rate.sleep()
        rospy.loginfo("Odom received.")

        # try connect to traj server
        self.wait_traj_server(timeout=10.0)

        twist = Twist()
        eps = 1e-3

        while not rospy.is_shutdown():
            if self.stage_start_x is None:
                self.stage_start_x = self.x

            rospy.loginfo("Pos:(%.3f, %.3f)  Total:%.3f m  Stage:%d",
                          self.x, self.y, self.total_dist, self.stage)

            # 1) +x 方向 1 m @ 0.2 m/s
            if self.stage == 1:
                moved = self.x - self.stage_start_x
                if moved < 1.0 - eps:
                    twist.linear.x = 0.2
                else:
                    twist.linear.x = 0.0
                    self.cmd_pub.publish(twist)
                    rospy.loginfo("Stage 1 done. Moved: %.3f m", moved)
                    self.stage = 2
                    self.stage_start_x = None
                    rospy.sleep(1)

            # 2) Arm: 
            elif self.stage == 2:
                rospy.loginfo("Stage 2: arm pose")
                self.arm_pose_deg(-120, 60, 0, -60, 0, -60, 0, hold_s=2.0)
                self.stage = 3
                self.stage_start_x = None
                rospy.sleep(1)

            # 3) Arm 归位
            elif self.stage == 3:
                rospy.loginfo("Stage 3: arm home")
                self.arm_home(hold_s=2.0)
                self.stage = 4
                self.stage_start_x = None
                rospy.sleep(1)

            # 4) −x 方向 1 m @ 0.1 m/s
            elif self.stage == 4:
                moved_back = self.stage_start_x - self.x
                if moved_back < 1.0 - eps:
                    twist.linear.x = -0.1
                else:
                    twist.linear.x = 0.0
                    self.cmd_pub.publish(twist)
                    rospy.loginfo("Stage 4 done. Moved back: %.3f m", moved_back)
                    self.stage = 5
                    self.stage_start_x = None
                    rospy.sleep(1)

            # 5) 轨迹控制：点1(…,-90°) → 点2(…, +90°) → 复原
            elif self.stage == 5:
                rospy.loginfo("Stage 5: trajectory control (2 poses) then home")

                # 关节名（请按你的控制器实际 joint_names 调整）
                joint_names = ["A1","A2","A3","A4","A5","A6","A7"]

                # 目标点（度）
                p1 = [90, 90, 0, -90, 0, 0, 0]
                p2 = [90, 90, 0,  90, 0, 0, 0]
                home = [-120, 60, 0, -60, 0, 60, 0]

                # 时间（相对轨迹起点，需递增）
                t1, t2, t3 = 2.0, 4.0, 6.0

                # 发送轨迹：p1@2s -> p2@4s -> home@6s
                if self.wait_traj_server(timeout=3.0):
                    self.send_traj_deg(joint_names, [p1, p2, home], [t1, t2, t3], wait=True)
                else:
                    rospy.logwarn("Trajectory server not ready, skip trajectory step.")
                    # 兜底：逐点用 JointPosition 发
                    self.arm_pose_deg(*p1, hold_s=1.5)
                    self.arm_pose_deg(*p2, hold_s=1.5)
                    self.arm_home(hold_s=1.5)

                self.stage = 6

            else:
                # 完成
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)
                rospy.loginfo("Sequence complete. Final Pos:(%.3f, %.3f)  Total: %.3f m",
                              self.x, self.y, self.total_dist)
                break

            # 发布底座速度（若有）
            self.cmd_pub.publish(twist)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("kmr_base_arm_sequence_traj", anonymous=True)
    node = BaseArmSequencer()
    node.run()