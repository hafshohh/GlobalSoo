#!/usr/bin/env python3
"""
D* Lite Global Path Planning + ILOS Path Following
USV Path Following Controller — lss_autonomy
USV Length: 1.6 m (FIXED)
"""

import os
import sys
import numpy as np
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist, PoseArray
from std_msgs.msg import Float32, Float64, String, Bool
from tf.transformations import euler_from_quaternion

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dstar_lite import DStarLiteGrid, OccupancyGridMap
from pid_controller import PIDController
from ilos_follower import ILOSFollower
from path_smoother import PathSmoother


def _wrap(a):
    while a > np.pi:
        a -= 2 * np.pi
    while a < -np.pi:
        a += 2 * np.pi
    return a


class PathPlannerNode:
    """Integrated Path Planner: D* Lite (Grid) + PID + ILOS"""

    def __init__(self):
        rospy.init_node('path_planner_dstar_apf', anonymous=False)

        # USV params (FIXED)
        self.usv_length = rospy.get_param('~usv_length',  1.6)
        self.control_dt = rospy.get_param('~control_dt',  0.1)

        # Occupancy grid params
        self.cell_m   = rospy.get_param('~cell_m',       2.0)
        self.pad_m    = rospy.get_param('~padding_m',    5.0)
        safe_dist        = rospy.get_param('~safe_dist_m',  0.6)
        self.safe_plan   = rospy.get_param('~safe_plan_m',  0.3)
        self.inflate     = self.safe_plan + safe_dist   # 0.9 m default

        # Smoothing params
        self.n_per_seg      = rospy.get_param('~n_per_seg',      25)
        self.eps_rdp        = rospy.get_param('~eps_rdp',        0.6)
        self.heuristic_w    = rospy.get_param('~heuristic_w',    1.2)

        # Planners — imported from separate modules
        self.dstar    = DStarLiteGrid()
        self.smoother = PathSmoother()
        # ILOS params
        k_integral          = rospy.get_param('~k_integral',          0.1)
        lookahead           = rospy.get_param('~lookahead',           5.0)
        cte_reset_threshold = rospy.get_param('~cte_reset_threshold', 1.5)
        lookahead_k         = rospy.get_param('~lookahead_k',         1.5)
        psi_filter_tau      = rospy.get_param('~psi_filter_tau',      0.5)
        # PID heading
        kp_psi              = rospy.get_param('~kp_psi',              2.0)
        ki_psi              = rospy.get_param('~ki_psi',              0.05)
        kd_psi              = rospy.get_param('~kd_psi',              0.5)
        # PID speed
        kp_u                = rospy.get_param('~kp_u',                0.8)
        ki_u                = rospy.get_param('~ki_u',                0.02)
        kd_u                = rospy.get_param('~kd_u',                0.3)

        self.pid_heading = PIDController(kp=kp_psi, ki=ki_psi, kd=kd_psi, dt=self.control_dt)
        self.pid_speed   = PIDController(kp=kp_u,   ki=ki_u,   kd=kd_u,   dt=self.control_dt)
        self.ilos        = ILOSFollower(
            k_integral=k_integral, lookahead=lookahead,
            cte_reset_threshold=cte_reset_threshold,
            lookahead_k=lookahead_k, psi_filter_tau=psi_filter_tau)

        # Publishers
        self.pub_global = rospy.Publisher('/planned_path_global', Path,    queue_size=1, latch=True)
        self.pub_smooth = rospy.Publisher('/planned_path_smooth', Path,    queue_size=1, latch=True)
        self.pub_local  = rospy.Publisher('/planned_path_local',  Path,    queue_size=1, latch=True)
        self.pub_hdg    = rospy.Publisher('/desired_heading',     Float32, queue_size=1)
        self.pub_cmd    = rospy.Publisher('/autonomy/pathfollowing', Twist,   queue_size=1)

        # Subscribers
        rospy.Subscriber('/odom',               Odometry,  self._odom_cb,      queue_size=1)
        rospy.Subscriber('/tf_simple/yaw',      Float64,   self._yaw_cb,       queue_size=1)
        rospy.Subscriber('/waypoints',          Path,      self._waypoints_cb, queue_size=1)
        rospy.Subscriber('/detected_obstacles', PoseArray, self._obstacles_cb, queue_size=1)
        rospy.Subscriber('/water_boundary',     Path,      self._boundary_cb,  queue_size=1)
        rospy.Subscriber('/autonomy/mission_manager/state_cmd',
                         String, self._cmd_cb, queue_size=1)
        rospy.Subscriber('/autonomy/mission_manager/speed',
                         Float64, self._speed_cb, queue_size=1)

        rospy.Subscriber('/autonomy/obstacle_status',
                         Bool, self._obstacle_cb, queue_size=1)

        # State
        self.pos               = np.array([0.0, 0.0])
        self.heading           = 0.0
        self.yaw_rate          = 0.0   # r dari odom, untuk derivative PID
        self.speed             = 0.0
        self.u_des             = rospy.get_param('~u_des', 1.5)  # m/s, dari web UI
        self.u_max             = rospy.get_param('~u_max', 2.0)  # m/s, maksimum kapal

        self.waypoints         = []   # [(x,y), ...]
        self.obstacles         = []   # [[cx, cy, r], ...]
        self.global_path       = []   # [(x,y), ...] raw D* Lite
        self.smooth_path       = []   # [(x,y), ...] setelah smoothing
        self.mission_active    = False
        self.pending_start     = False  # Start diterima sebelum path siap
        self.obstacle_detected = False
        self.boundary_polygon  = []   # [(x,y), ...] batas perairan (dari Web UI)
        self.mission_start_time = None  # untuk fade-in saat misi mulai

        rospy.loginfo("=" * 60)
        rospy.loginfo("PathPlanner: D*Lite(Grid) + PID + ILOS")
        rospy.loginfo("USV: %.1f m | cell: %.1f m | inflate: %.2f m",
                      self.usv_length, self.cell_m, self.inflate)
        rospy.loginfo("=" * 60)

    # ─── Callbacks ───────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.pos = np.array([msg.pose.pose.position.x,
                             msg.pose.pose.position.y])
        # heading TIDAK dibaca dari sini — odom orientation di boat_tf.py salah (lidar quaternion)
        # heading diupdate oleh _yaw_cb dari /tf_simple/yaw (langsung dari kompas)
        self.speed    = np.linalg.norm([msg.twist.twist.linear.x,
                                        msg.twist.twist.linear.y])
        self.yaw_rate = msg.twist.twist.angular.z

    def _yaw_cb(self, msg: Float64):
        self.heading = float(msg.data)

    def _speed_cb(self, msg: Float64):
        self.u_des = float(np.clip(msg.data, 0.0, self.u_max))
        rospy.loginfo('PathPlanner: u_des → %.2f m/s', self.u_des)


    def _obstacle_cb(self, msg: Bool):
        self.obstacle_detected = msg.data

    def _cmd_cb(self, msg: String):
        cmd = msg.data
        if cmd == 'start':
            if self.global_path or self.smooth_path:
                self.mission_active     = True
                self.pending_start      = False
                self.mission_start_time = rospy.Time.now()
                rospy.loginfo("PathPlanner: START")
            else:
                self.pending_start = True
                rospy.logwarn("PathPlanner: START diterima tapi path belum ada — akan auto-start setelah path siap")
        elif cmd == 'stop':
            self.mission_active     = False
            self.pending_start      = False
            self.mission_start_time = None
            self.pid_heading.reset()
            self.pid_speed.reset()
            self.ilos.reset()
            rospy.loginfo("PathPlanner: STOP")
        elif cmd == 'restart':
            self.mission_active     = True
            self.pending_start      = False
            self.mission_start_time = rospy.Time.now()
            self.ilos.reset()
            rospy.loginfo("PathPlanner: RESTART")

    def _waypoints_cb(self, msg: Path):
        self.waypoints = [(p.pose.position.x, p.pose.position.y)
                          for p in msg.poses]
        rospy.loginfo("PathPlanner: %d waypoints diterima → plan seluruh rute",
                      len(self.waypoints))
        self._plan_global()

    def _boundary_cb(self, msg: Path):
        self.boundary_polygon = [(p.pose.position.x, p.pose.position.y)
                                 for p in msg.poses]
        rospy.loginfo("PathPlanner: %d titik batas perairan diterima → replan",
                      len(self.boundary_polygon))
        if self.waypoints:
            self._plan_global()

    def _obstacles_cb(self, msg: PoseArray):
        # position.z = radius; default 0.25 m if not set
        new = [[p.position.x, p.position.y,
                p.position.z if p.position.z > 0 else 0.25]
               for p in msg.poses]
        if new != self.obstacles:
            self.obstacles = new
            if self.mission_active:
                rospy.loginfo("PathPlanner: %d obstacles updated → replan",
                              len(new))
                self._plan_global()

    # ─── Planning ────────────────────────────────────────────────

    def _build_map(self, seg_start=None, seg_goal=None):
        start = seg_start if seg_start is not None else tuple(self.pos)
        goal  = seg_goal  if seg_goal  is not None else self.waypoints[-1]

        # Bounding box hanya dari seg_start dan seg_goal — tidak semua waypoints.
        # Menyertakan semua waypoints bisa membuat grid meledak jika pos GPS salah skala.
        xs = [start[0], goal[0]] + [o[0] for o in self.obstacles]
        ys = [start[1], goal[1]] + [o[1] for o in self.obstacles]

        if self.boundary_polygon:
            xs += [p[0] for p in self.boundary_polygon]
            ys += [p[1] for p in self.boundary_polygon]

        p   = self.pad_m
        ox  = min(xs) - p;  oy  = min(ys) - p
        w_m = max(xs) - ox + p;  h_m = max(ys) - oy + p

        # Guard: grid >2000 sel berarti ada koordinat yang salah skala
        nC_est = int(np.ceil(w_m / self.cell_m))
        nR_est = int(np.ceil(h_m / self.cell_m))
        if nC_est > 2000 or nR_est > 2000:
            rospy.logwarn(
                "_build_map: grid %dx%d terlalu besar! "
                "start=(%.1f,%.1f) goal=(%.1f,%.1f) xs=[%.1f,%.1f] ys=[%.1f,%.1f]. "
                "Cek konversi GPS/odom. Fallback ke grid minimal.",
                nC_est, nR_est,
                start[0], start[1], goal[0], goal[1],
                min(xs), max(xs), min(ys), max(ys))
            ox  = min(start[0], goal[0]) - p
            oy  = min(start[1], goal[1]) - p
            w_m = abs(goal[0] - start[0]) + 2 * p
            h_m = abs(goal[1] - start[1]) + 2 * p

        occ = OccupancyGridMap((ox, oy), (w_m, h_m), self.cell_m)
        if self.obstacles:
            occ.add_obstacles(self.obstacles, self.inflate)

        # Tandai sel di luar batas perairan sebagai obstacle
        if len(self.boundary_polygon) >= 3:
            self._apply_boundary_mask(occ)

        return occ

    def _plan_global(self):
        if not self.waypoints:
            return

        full_path = []
        seg_start = tuple(self.pos)

        for i, wp in enumerate(self.waypoints):
            occ     = self._build_map(seg_start, wp)
            start_g = occ.world_to_grid(*seg_start)
            goal_g  = occ.world_to_grid(*wp)

            rospy.loginfo("D*Grid: WP%d (%.1f,%.1f)→(%.1f,%.1f) | grid %dx%d",
                          i + 1, seg_start[0], seg_start[1],
                          wp[0], wp[1], occ.nC, occ.nR)

            t0 = rospy.Time.now()
            path_g, expanded = self.dstar.plan(occ, start_g, goal_g, w=self.heuristic_w)
            dt = (rospy.Time.now() - t0).to_sec()

            if path_g is None:
                rospy.logwarn("D*Grid: WP%d no path → fallback garis lurus", i + 1)
                seg_path = [seg_start, wp]
            else:
                seg_path = [occ.grid_to_world(c, r) for c, r in path_g]
                seg_path[0]  = seg_start
                seg_path[-1] = wp
                rospy.loginfo("D*Grid: WP%d %d pts | %d expanded | %.3fs",
                              i + 1, len(seg_path), expanded, dt)

            full_path.extend(seg_path if not full_path else seg_path[1:])
            seg_start = wp

        if not full_path:
            return

        self.global_path = full_path
        self._publish_path(self.pub_global, self.global_path)

        # Smoothing: G2-CBS C² + clearance enforcement
        self.smooth_path = self._smooth_path(self.global_path)
        self._publish_path(self.pub_smooth, self.smooth_path)

        self.ilos.reset()
        rospy.loginfo("D*Grid: rute lengkap %d titik total", len(full_path))

        # Auto-start jika user sudah klik Start sebelum path siap
        if self.pending_start and not self.mission_active:
            self.mission_active     = True
            self.pending_start      = False
            self.mission_start_time = rospy.Time.now()
            rospy.loginfo("PathPlanner: AUTO-START (pending_start terpenuhi setelah path siap)")

    def _smooth_path(self, raw_path):
        if len(raw_path) < 2:
            return raw_path
        pts   = np.array(raw_path, dtype=float)
        start, goal = pts[0].copy(), pts[-1].copy()
        n_raw = len(pts)

        # LOS shortcutting dihapus — RDP di dalam smooth() menangani
        # staircase grid secara geometris tanpa risiko memotong detour
        # yang dibuat D* Lite untuk menghindari obstacle.

        # 1) G2-CBS C² spline (RDP internal sudah mereduksi staircase)
        t1     = rospy.Time.now()
        smooth = self.smoother.smooth(pts, self.n_per_seg, self.eps_rdp)
        dt_s   = (rospy.Time.now() - t1).to_sec()

        # 2) Clearance enforcement
        if self.obstacles:
            smooth, info = self.smoother.enforce_clearance(
                smooth, self.obstacles, self.safe_plan)
            rospy.loginfo("Smoother: raw%d→%d pts | spl %.3fs"
                          " | min_clr=%.2fm (%d iter)",
                          n_raw, len(smooth), dt_s,
                          info['min_clearance'], info['iterations'])
        else:
            rospy.loginfo("Smoother: raw%d→%d pts | spl %.3fs",
                          n_raw, len(smooth), dt_s)

        smooth[0] = start
        smooth[-1] = goal
        return [tuple(p) for p in smooth]

    # ─── Control ─────────────────────────────────────────────────

    def _compute_control(self):
        path = self.smooth_path if len(self.smooth_path) >= 2 else self.global_path
        if len(path) < 2 or not self.mission_active:
            self.pub_cmd.publish(Twist())
            return
        if self.obstacle_detected:
            return

        # Fade-in 2 detik pertama sejak misi start — cegah spike aktuator
        if self.mission_start_time is not None:
            t_elapsed = (rospy.Time.now() - self.mission_start_time).to_sec()
            fade_in = min(1.0, t_elapsed / 2.0)
        else:
            fade_in = 1.0

        goal      = np.array(path[-1])
        dist_goal = float(np.linalg.norm(goal - self.pos))

        # Arrival detection
        arrive_dist = rospy.get_param('~arrive_dist', 2.0)
        if dist_goal < arrive_dist:
            self.mission_active = False
            self.pub_cmd.publish(Twist())
            rospy.loginfo('PathPlanner: goal reached (d=%.2fm)', dist_goal)
            return

        # Terminal guidance: saat d_goal < lookahead, heading langsung ke WP terakhir
        if dist_goal < self.ilos.lookahead:
            psi_des = _wrap(float(np.arctan2(goal[1] - self.pos[1],
                                             goal[0] - self.pos[0])))
        else:
            psi_des = self.ilos.compute_desired_heading(self.pos, path, self.heading)
        self.pub_hdg.publish(Float32(psi_des))

        e_psi = _wrap(psi_des - self.heading)

        # Clamp heading error ke ±20° sebelum masuk PID — cegah integral windup
        # dan spike TN saat kapal sangat menyimpang (misal setelah replan)
        e_psi_clamped = float(np.clip(e_psi, -np.deg2rad(20.0), np.deg2rad(20.0)))

        # Derivative on measurement: pakai yaw rate r dari sensor (bukan Δe/Δt)
        yaw_rate = np.clip(
            self.pid_heading.update_with_rate(e_psi_clamped, self.yaw_rate),
            -1.0, 1.0) * fade_in

        # Heading-first dengan minimum surge untuk efektivitas rudder.
        # Rudder butuh aliran air (forward motion) agar bisa membelokkan kapal.
        # Tanpa minimum ini: saat error > 90°, surge = 0 → rudder mati → deadlock.
        # min_surge = 0.15 memberi cukup aliran untuk rudder bekerja.
        cos_factor    = max(0.15, float(np.cos(e_psi)))
        goal_tol      = rospy.get_param('~goal_tol', 4.0)
        desired_speed = (self.u_des / self.u_max) * min(1.0, dist_goal / goal_tol) * cos_factor
        current_speed = self.speed / self.u_max
        surge         = np.clip(
            self.pid_speed.update(desired_speed - current_speed),
            -1.0, 1.0) * fade_in

        _RUDDER_MAX = np.radians(40.0)
        cmd = Twist()
        cmd.linear.x  = surge
        cmd.angular.z = float(yaw_rate) * _RUDDER_MAX
        self.pub_cmd.publish(cmd)

    # ─── Boundary mask ───────────────────────────────────────────

    def _apply_boundary_mask(self, occ):
        """Tandai semua sel di luar polygon batas perairan sebagai obstacle."""
        poly = np.array(self.boundary_polygon)
        wx = occ.ox + (np.arange(occ.nC) + 0.5) * occ.cell
        wy = occ.oy + (np.arange(occ.nR) + 0.5) * occ.cell
        WX, WY = np.meshgrid(wx, wy)
        outside = ~self._points_in_polygon(WX.ravel(), WY.ravel(), poly)
        occ.grid |= outside.reshape(occ.nR, occ.nC).astype(np.uint8)

    def _points_in_polygon(self, xs, ys, polygon):
        """Ray casting vectorized: True jika titik di dalam polygon."""
        n = len(polygon)
        inside = np.zeros(len(xs), dtype=bool)
        px, py = polygon[:, 0], polygon[:, 1]
        j = n - 1
        for i in range(n):
            dpy = py[j] - py[i]
            cond = (py[i] > ys) != (py[j] > ys)
            with np.errstate(divide='ignore', invalid='ignore'):
                cross = np.where(np.abs(dpy) > 1e-10,
                                 (px[j] - px[i]) * (ys - py[i]) / dpy + px[i],
                                 np.inf)
            inside ^= cond & (xs < cross)
            j = i
        return inside

    # ─── Helper ──────────────────────────────────────────────────

    def _publish_path(self, pub, path_list):
        msg = Path()
        msg.header.stamp    = rospy.Time.now()
        msg.header.frame_id = 'map'
        for wp in path_list:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = wp[0]
            ps.pose.position.y = wp[1]
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        pub.publish(msg)

    # ─── Local path (sisa path dari posisi kapal saat ini) ───────

    def _plan_local(self):
        path = self.smooth_path if len(self.smooth_path) >= 2 else self.global_path
        if len(path) < 2:
            return
        pts = np.array(path)
        dists = np.linalg.norm(pts - self.pos, axis=1)
        idx = int(np.argmin(dists))
        self._publish_path(self.pub_local, path[idx:])

    # ─── Main loop ───────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(int(round(1.0 / self.control_dt)))
        while not rospy.is_shutdown():
            self._plan_local()
            self._compute_control()
            rate.sleep()


if __name__ == '__main__':
    try:
        PathPlannerNode().run()
    except rospy.ROSInterruptException:
        rospy.loginfo("PathPlanner shut down")
