#!/usr/bin/env python3
import rospy, math, time, actionlib
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from kmriiwa_msgs.msg import JointPosition
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def deg2rad(d): return d * math.pi / 180.0
def rad2deg(r): return r * 180.0 / math.pi

SAFE_POSE_DEG = [-120, 60, 0, -60, 0, 60, 0]  # A1..A7，度
SAFE_TOL_DEG  = 3.0                           # 每关节容差

class BaseArmSequencer:
    def __init__(self):
        # pubs/subs
        self.odom_sub   = rospy.Subscriber("/kmriiwa/base/state/odom", Odometry, self.odom_cb)
        self.joint_sub  = rospy.Subscriber("/kmriiwa/arm/joint_states", JointState, self.joint_cb)
        self.cmd_pub    = rospy.Publisher("/kmriiwa/base/command/cmd_vel", Twist, queue_size=10)
        self.arm_pub    = rospy.Publisher("/kmriiwa/arm/command/JointPosition", JointPosition, queue_size=10)

        # FollowJointTrajectory client
        self.traj_cli = actionlib.SimpleActionClient(
            "/kmriiwa/arm/manipulator_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )

        # states
        self.has_odom = False
        self.x = 0.0; self.y = 0.0
        self.prev_x = None; self.prev_y = None
        self.total_dist = 0.0

        self.has_js = False
        self.joint_names = []
        self.joint_pos_rad = []

        # control
        self.stage = 1
        self.stage_start_x = None
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.stop_robot)

    # -------- callbacks --------
    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if self.prev_x is None:
            self.prev_x, self.prev_y = self.x, self.y
            self.has_odom = True
            return
        dx = self.x - self.prev_x; dy = self.y - self.prev_y
        self.total_dist += (dx*dx + dy*dy) ** 0.5
        self.prev_x, self.prev_y = self.x, self.y
        self.has_odom = True

    def joint_cb(self, msg: JointState):
        self.joint_names = list(msg.name)
        self.joint_pos_rad = list(msg.position)
        self.has_js = True

    # -------- utils --------
    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def wait_traj_server(self, timeout=5.0):
        return self.traj_cli.wait_for_server(rospy.Duration(timeout))

    def current_pose_deg_by_A(self):
        """
        返回按 A1..A7 排好序的当前角度(度)。
        如果 joint_states 名称不是 A1..A7，会尽力按 A1..A7 映射，缺失则用 0。
        """
        if not self.has_js: return None
        idx = {name: i for i, name in enumerate(self.joint_names)}
        ordered = []
        for key in ["A1","A2","A3","A4","A5","A6","A7"]:
            if key in idx:
                ordered.append(rad2deg(self.joint_pos_rad[idx[key]]))
            else:
                # 兼容名称不同：尝试小写
                k2 = key.lower()
                if k2 in idx:
                    ordered.append(rad2deg(self.joint_pos_rad[idx[k2]]))
                else:
                    # 没找到就填 0（并提示）
                    ordered.append(0.0)
        return ordered

    def is_in_pose_deg(self, target_deg, tol_deg=SAFE_TOL_DEG):
        cur = self.current_pose_deg_by_A()
        if cur is None: return False
        for c, t in zip(cur, target_deg):
            if abs(c - t) > tol_deg:
                return False
        return True

    def send_jointposition_deg(self, pose_deg, hold_s=1.5):
        """兜底：用 JointPosition 连续发布到指定姿态（度->弧度转换可在驱动内或此处处理）。"""
        a = [deg2rad(v) for v in pose_deg]
        t0 = time.time(); r = rospy.Rate(20)
        msg = JointPosition()
        while not rospy.is_shutdown() and (time.time() - t0) < hold_s:
            msg.header.stamp = rospy.Time.now()
            msg.a1, msg.a2, msg.a3, msg.a4, msg.a5, msg.a6, msg.a7 = a
            self.arm_pub.publish(msg)
            r.sleep()

    def go_pose_via_trajectory_deg(self, pose_deg, duration_s=2.0, wait=True):
        """优先用 FollowJointTrajectory 去既定姿态；失败则回退 JointPosition。"""
        if self.wait_traj_server(3.0):
            goal = FollowJointTrajectoryGoal()
            traj = JointTrajectory()
            traj.joint_names = ["A1","A2","A3","A4","A5","A6","A7"]
            pt = JointTrajectoryPoint()
            pt.positions = [deg2rad(v) for v in pose_deg]
            pt.time_from_start = rospy.Duration(duration_s)
            traj.points = [pt]
            goal.trajectory = traj
            self.traj_cli.send_goal(goal)
            if wait:
                self.traj_cli.wait_for_result()
                rospy.loginfo("Traj to pose done.")
        else:
            rospy.logwarn("Trajectory server not available, fallback to JointPosition.")
            self.send_jointposition_deg(pose_deg, hold_s=max(1.5, duration_s))

    def ensure_safe_pose(self, timeout_s=8.0):
        """确保手臂在安全姿态，不在则移动到位，超时仍未到位则返回 False。"""
        start = time.time()
        while not rospy.is_shutdown():
            if self.is_in_pose_deg(SAFE_POSE_DEG):
                return True
            # 发送到安全姿态
            self.go_pose_via_trajectory_deg(SAFE_POSE_DEG, duration_s=2.0, wait=True)
            # 小等一会儿，再检查
            rospy.sleep(0.2)
            if time.time() - start > timeout_s:
                return self.is_in_pose_deg(SAFE_POSE_DEG)
        return False

    def guarded_drive_x(self, speed, distance, eps=1e-3):
        """
        带守护的直行：先确保在安全姿态，再发底座速度，走指定距离。
        speed>0 前进，<0 后退；distance>0 绝对值距离。
        """
        # 先确保 joint_states/odom 都就绪
        while not rospy.is_shutdown() and not (self.has_js and self.has_odom):
            rospy.loginfo("Waiting joint_states and odom ...")
            self.rate.sleep()

        # 确保安全姿态
        if not self.ensure_safe_pose():
            rospy.logwarn("Cannot reach SAFE pose, abort drive.")
            return False

        # 开始走
        twist = Twist()
        start_x = self.x
        target = abs(distance)
        rospy.loginfo("Start guarded drive: v=%.3f m/s, L=%.3f m", speed, target)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            moved = abs(self.x - start_x)
            if moved < target - eps:
                twist.linear.x = speed
            else:
                break
            # 每个循环再次检查安全姿态；若偏出安全姿态则立即停
            if not self.is_in_pose_deg(SAFE_POSE_DEG):
                rospy.logwarn("Left SAFE pose during drive! stopping.")
                break
            self.cmd_pub.publish(twist)
            r.sleep()

        # 停止
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
        rospy.loginfo("Drive done. moved=%.3f m", abs(self.x - start_x))
        return True

    # -------- main sequence --------
    def run(self):
        rospy.loginfo("Waiting for odom & joint_states ...")
        while not rospy.is_shutdown() and not (self.has_odom and self.has_js):
            self.rate.sleep()
        rospy.loginfo("Ready. Start sequence.")

        # 1) +x 方向 0.2 m/s 走 1 m（带安全姿态守护）
        self.guarded_drive_x(speed=0.2, distance=1.0)

        # 2) arm: 90, 90, 0, -90, 0, 0, 0
        rospy.loginfo("Move arm to [90,90,0,-90,0,0,0]")
        self.go_pose_via_trajectory_deg([90,90,0,-90,0,0,0], duration_s=2.0, wait=True)

        # 3) arm 归位
        rospy.loginfo("Arm home")
        self.go_pose_via_trajectory_deg([0,0,0,0,0,0,0], duration_s=2.0, wait=True)

        # 4) −x 方向 0.1 m/s 走 1 m（带安全姿态守护）
        self.guarded_drive_x(speed=-0.1, distance=1.0)

        # 5) 轨迹控制：点1 -> 点2 -> 复原
        rospy.loginfo("Trajectory: p1 -> p2 -> home")
        if self.wait_traj_server(3.0):
            goal = FollowJointTrajectoryGoal()
            traj = JointTrajectory()
            traj.joint_names = ["A1","A2","A3","A4","A5","A6","A7"]
            p1 = [90, 90, 0, -90, 0, 0, 0]
            p2 = [90, 90, 0,  90, 0, 0, 0]
            home = [0,0,0,0,0,0,0]
            t1, t2, t3 = 2.0, 4.0, 6.0
            for pose_deg, t in zip([p1,p2,home],[t1,t2,t3]):
                pt = JointTrajectoryPoint()
                pt.positions = [deg2rad(v) for v in pose_deg]
                pt.time_from_start = rospy.Duration(t)
                traj.points.append(pt)
            goal.trajectory = traj
            self.traj_cli.send_goal(goal)
            self.traj_cli.wait_for_result()
            rospy.loginfo("Trajectory sequence done.")
        else:
            rospy.logwarn("Trajectory server not available, fallback point-by-point.")
            self.go_pose_via_trajectory_deg([90,90,0,-90,0,0,0], 2.0, True)
            self.go_pose_via_trajectory_deg([90,90,0, 90,0,0,0], 2.0, True)
            self.go_pose_via_trajectory_deg([0,0,0,0,0,0,0], 2.0, True)

        rospy.loginfo("Sequence complete. Final Pos:(%.3f, %.3f), Total: %.3f m",
                      self.x, self.y, self.total_dist)

if __name__ == "__main__":
    rospy.init_node("kmr_safe_guarded_motion", anonymous=True)
    BaseArmSequencer().run()
