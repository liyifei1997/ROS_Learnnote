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

class BaseArmSequencer:
    def __init__(self):
        # === 按你的控制器配置修改 ===
        self.joint_names   = ["A1","A2","A3","A4","A5","A6","A7"]  # 若 6 轴，请删去 A7 并相应改代码
        self.safe_pose_deg = [-120, 60, 0, -60, 0, 60, 0]           # 安全位（度）
        self.safe_tol_deg  = 10.0                                   # 允许误差 ±10°
        # ============================

        # pubs/subs
        self.odom_sub   = rospy.Subscriber("/kmriiwa/base/state/odom", Odometry, self.odom_cb)
        self.cmd_pub    = rospy.Publisher("/kmriiwa/base/command/cmd_vel", Twist, queue_size=10)
        self.arm_pub    = rospy.Publisher("/kmriiwa/arm/command/JointPosition", JointPosition, queue_size=10)
        self.jstate_sub = rospy.Subscriber("/kmriiwa/arm/joint_states", JointState, self.jstate_cb)

        # action client
        self.traj_cli = actionlib.SimpleActionClient(
            "/kmriiwa/arm/manipulator_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )

        # odom state
        self.has_odom = False
        self.x = 0.0; self.y = 0.0
        self.prev_x = None; self.prev_y = None
        self.total_dist = 0.0

        # joint state
        self.joint_pos_map = {}      # name -> rad
        self.last_warn_time = rospy.Time(0)

        # stage control
        self.stage = 1
        self.stage_start_x = None
        self.rate = rospy.Rate(10)

        rospy.on_shutdown(self.stop_robot)

    # ---------- Odometry ----------
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

    # ---------- JointStates ----------
    def jstate_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_pos_map[name] = pos  # rad

    def get_current_joints_deg(self):
        vals = []
        for n in self.joint_names:
            if n not in self.joint_pos_map:
                return None
            vals.append(rad2deg(self.joint_pos_map[n]))
        return vals

    def is_arm_in_safe_pose(self):
        cur = self.get_current_joints_deg()
        if cur is None:  # 没有关节数据当作不安全
            return False
        for c, s in zip(cur, self.safe_pose_deg):
            if abs(c - s) > self.safe_tol_deg:
                return False
        return True

    def warn_if_not_safe(self, context=""):
        if not self.is_arm_in_safe_pose():
            now = rospy.Time.now()
            if (now - self.last_warn_time).to_sec() > 1.0:  # 1s 节流
                cur = self.get_current_joints_deg()
                rospy.logwarn(
                    "[SAFEPOSE] Arm NOT in safe pose %s. Current(deg)=%s, Expect=%s, tol=±%.1f°",
                    ("("+context+")" if context else ""),
                    ["{:.1f}".format(v) for v in (cur or [])],
                    self.safe_pose_deg, self.safe_tol_deg
                )
                self.last_warn_time = now

    # ---------- Base ----------
    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    # ---------- Arm: point command ----------
    def send_arm_pos_rad(self, a_list, repeat=1, hz=20.0):
        # a_list: [a1..a7] in rad
        r = rospy.Rate(hz)
        for _ in range(max(1, repeat)):
            msg = JointPosition()
            msg.header.stamp = rospy.Time.now()
            msg.a1, msg.a2, msg.a3, msg.a4, msg.a5, msg.a6, msg.a7 = [float(v) for v in a_list]
            self.arm_pub.publish(msg)
            r.sleep()

    def arm_pose_deg(self, deg_list, hold_s=1.5):
        # deg_list: [j1..j7] in deg
        rad_list = [deg2rad(v) for v in deg_list]
        t0 = time.time(); r = rospy.Rate(20)
        while not rospy.is_shutdown() and (time.time() - t0) < hold_s:
            self.send_arm_pos_rad(rad_list)
            r.sleep()

    def arm_to_safe(self, hold_s=2.0):
        self.arm_pose_deg(self.safe_pose_deg, hold_s=hold_s)

    # ---------- Trajectory ----------
    def wait_traj_server(self, timeout=10.0):
        ok = self.traj_cli.wait_for_server(rospy.Duration(timeout))
        if not ok:
            rospy.logwarn("FollowJointTrajectory server not available.")
        return ok

    def send_traj_deg(self, points_deg, times_s, wait=True):
        assert len(points_deg) == len(times_s) and len(points_deg) > 0
        goal = FollowJointTrajectoryGoal()
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        for p_deg, t in zip(points_deg, times_s):
            pt = JointTrajectoryPoint()
            pt.positions = [deg2rad(v) for v in p_deg]
            pt.time_from_start = rospy.Duration(float(t))
            traj.points.append(pt)

        goal.trajectory = traj
        self.traj_cli.send_goal(goal)
        if wait:
            self.traj_cli.wait_for_result()
            rospy.loginfo("Trajectory result: %s", str(self.traj_cli.get_result()))

    # ---------- Main ----------
    def run(self):
        rospy.loginfo("Waiting for /kmriiwa/base/state/odom ...")
        while not rospy.is_shutdown() and not self.has_odom:
            self.rate.sleep()
        rospy.loginfo("Odom received.")

        self.wait_traj_server(timeout=10.0)

        twist = Twist(); eps = 1e-3

        while not rospy.is_shutdown():
            if self.stage_start_x is None:
                self.stage_start_x = self.x

            rospy.loginfo("Pos:(%.3f, %.3f)  Total:%.3f m  Stage:%d",
                          self.x, self.y, self.total_dist, self.stage)

            # 1) +x 方向 1 m @ 0.2 m/s（动前检查安全位并打印）
            if self.stage == 1:
                self.warn_if_not_safe(context="before +x move")
                moved = self.x - self.stage_start_x
                if moved < 1.0 - eps:
                    twist.linear.x = 0.2
                else:
                    twist.linear.x = 0.0
                    self.cmd_pub.publish(twist)
                    rospy.loginfo("Stage 1 done. Moved: %.3f m", moved)
                    self.stage = 2; self.stage_start_x = None
                    rospy.sleep(0.3)

            # 2) Arm 到位：j1=+90°, j2=+90°, j4=-90°
            elif self.stage == 2:
                rospy.loginfo("Stage 2: arm pose (90,90,0,-90,0,0,0)")
                self.arm_pose_deg([90, 90, 0, -90, 0, 0, 0], hold_s=2.0)
                self.stage = 3; self.stage_start_x = None
                rospy.sleep(0.2)

            # 3) Arm 到安全位置
            elif self.stage == 3:
                rospy.loginfo("Stage 3: arm to SAFE pose %s", str(self.safe_pose_deg))
                self.arm_to_safe(hold_s=2.0)
                self.stage = 4; self.stage_start_x = None
                rospy.sleep(0.2)

            # 4) −x 方向 1 m @ 0.1 m/s（动前检查安全位并打印）
            elif self.stage == 4:
                self.warn_if_not_safe(context="before -x move")
                moved_back = self.stage_start_x - self.x
                if moved_back < 1.0 - eps:
                    twist.linear.x = -0.1
                else:
                    twist.linear.x = 0.0
                    self.cmd_pub.publish(twist)
                    rospy.loginfo("Stage 4 done. Moved back: %.3f m", moved_back)
                    self.stage = 5; self.stage_start_x = None
                    rospy.sleep(0.3)

            # 5) 轨迹控制：点1 -> 点2 -> 安全位置
            elif self.stage == 5:
                rospy.loginfo("Stage 5: trajectory p1 -> p2 -> SAFE")

                p1   = [90, 90, 0, -90, 0, 0, 0]
                p2   = [90, 90, 0,  90, 0, 0, 0]
                safe = self.safe_pose_deg[:]  # 复制安全位

                t1, t2, t3 = 2.0, 4.0, 6.0  # 轨迹时间点（严格递增）

                if self.wait_traj_server(timeout=3.0):
                    self.send_traj_deg([p1, p2, safe], [t1, t2, t3], wait=True)
                else:
                    rospy.logwarn("Trajectory server not ready, fallback to point commands.")
                    self.arm_pose_deg(p1, hold_s=1.5)
                    self.arm_pose_deg(p2, hold_s=1.5)
                    self.arm_to_safe(hold_s=1.5)

                self.stage = 6

            else:
                # 完成
                twist.linear.x = 0.0
                self.cmd_pub.publish(twist)
                rospy.loginfo("Sequence complete. Final Pos:(%.3f, %.3f)  Total: %.3f m",
                              self.x, self.y, self.total_dist)
                break

            # 发布速度（如有）
            self.cmd_pub.publish(twist)
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("kmr_base_arm_safe_traj", anonymous=True)
    node = BaseArmSequencer()
    node.run()
