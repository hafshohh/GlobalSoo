#!/usr/bin/env python3
"""
Integral Line-of-Sight (ILOS) Path Following
Guidance law for Unmanned Surface Vehicles (USVs)
References: Fossen & Pettersen (2014)
"""

import os
import sys
import numpy as np

# Pastikan folder scripts ada di path agar pid_controller bisa diimport
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pid_controller import PIDController

try:
    import rospy
    from nav_msgs.msg import Odometry, Path
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String, Float64
    from tf.transformations import euler_from_quaternion
    _ROS_OK = True
except ImportError:
    _ROS_OK = False


def _wrap(a):
    while a > np.pi:
        a -= 2 * np.pi
    while a < -np.pi:
        a += 2 * np.pi
    return a


class ILOSFollower:
    """
    Integral Line-of-Sight Path Following Controller
    
    Computes desired heading based on:
    - Cross-track error (perpendicular distance to path)
    - Integral of cross-track error
    - Path tangent direction
    """
    
    def __init__(self, k_cross=0.5, k_integral=0.1, lookahead=5.0,
                 cte_reset_threshold=1.5, lookahead_k=1.5, psi_filter_tau=0.5):
        """
        Initialize ILOS controller.

        Args:
            k_cross:              Unused (kept for API compatibility).
            k_integral:           Integral gain κ — compensates steady-state
                                  cross-track bias (current, wind). (default: 0.1)
            lookahead:            Minimum look-ahead distance Δ_min [m].
                                  (default: 5.0)
            cte_reset_threshold:  Cross-track error threshold [m] beyond which
                                  the integral is reset to prevent windup, e.g.
                                  after path replanning or obstacle avoidance.
                                  (default: 1.5)
            lookahead_k:          Adaptive lookahead multiplier. Effective Δ =
                                  max(lookahead, |e_y| × lookahead_k). Larger CTE
                                  → larger Δ → smoother, less aggressive correction.
                                  (default: 1.5)
            psi_filter_tau:       Time constant [s] untuk low-pass filter heading
                                  reference. Koefisien α = dt/τ dihitung otomatis
                                  dari self.dt sehingga perilaku filter sama di
                                  simulasi (dt=0.02s) maupun real USV (dt=0.1s).
                                  (default: 0.5 s)
        """
        self.k_integral          = k_integral
        self.lookahead           = lookahead
        self.cte_reset_threshold = cte_reset_threshold
        self.lookahead_k         = lookahead_k
        self.psi_filter_tau      = psi_filter_tau

        self.cross_track_integral = 0.0
        self.psi_d_filtered       = None   # inisialisasi saat pertama dipanggil
        self.dt = 0.1

    def compute_desired_heading(self, position, path, current_heading):
        """
        Compute desired heading using ILOS (Fossen & Pettersen 2014).

        Guidance law:
            ψ_d = α_k + arctan2(-(e_y + κ·σ), Δ_eff)
        Integral update (rigorous ISS form):
            σ̇ = Δ_eff·e_y / (Δ_eff² + (e_y + κ·σ)²)
        Adaptive lookahead:
            Δ_eff = max(Δ_min, |e_y| × lookahead_k)
        Heading filter (prevents sudden ψ_d jumps on replan/segment switch):
            ψ_d_filtered += α · wrap(ψ_d_raw − ψ_d_filtered)

        Args:
            position:        Current position [x, y]
            path:            List of waypoints [(x, y), ...]
            current_heading: Current heading in radians

        Returns:
            Filtered desired heading in radians
        """
        if len(path) < 2:
            return current_heading

        pos = np.array(position, dtype=float)

        closest_idx = self._find_closest_segment(pos, path)

        p1 = np.array(path[closest_idx], dtype=float)
        p2 = np.array(path[min(closest_idx + 1, len(path) - 1)], dtype=float)

        segment_vec = p2 - p1
        segment_len = np.linalg.norm(segment_vec)

        if segment_len < 1e-6:
            return current_heading

        # Path tangent heading α_k
        alpha_k = np.arctan2(segment_vec[1], segment_vec[0])

        # Cross-track error e_y: signed perpendicular distance
        # positive = kapal di sebelah KIRI path
        segment_normal = np.array([-segment_vec[1], segment_vec[0]]) / segment_len
        e_y = np.dot(pos - p1, segment_normal)

        # Adaptive lookahead: Δ_eff membesar saat CTE besar
        # → koreksi lebih smooth, cegah overshoot saat jauh dari jalur
        delta_eff = max(self.lookahead, abs(e_y) * self.lookahead_k)

        # Reset integral saat CTE melampaui threshold (cegah windup setelah
        # replan D*Lite atau setelah obstacle avoidance)
        if abs(e_y) > self.cte_reset_threshold:
            self.cross_track_integral = 0.0

        # Integral update — rigorous ILOS form (Fossen & Pettersen 2014, eq. 10.75)
        # Integral hanya diupdate saat CTE kecil (di dalam threshold)
        nu = e_y + self.k_integral * self.cross_track_integral
        denom = delta_eff ** 2 + nu ** 2
        self.cross_track_integral += self.dt * delta_eff * e_y / denom

        # ILOS heading command (raw)
        nu = e_y + self.k_integral * self.cross_track_integral
        psi_d_raw = alpha_k + np.arctan2(-nu, delta_eff)

        # Low-pass filter pada heading reference — cegah lonjakan ψ_d
        # saat D*Lite replan menghasilkan path baru atau saat pindah segmen.
        # α = dt/τ → perilaku filter sama di semua dt (simulasi 50Hz / real 10Hz)
        alpha = min(1.0, self.dt / self.psi_filter_tau)
        if self.psi_d_filtered is None:
            self.psi_d_filtered = psi_d_raw
        self.psi_d_filtered += alpha * _wrap(psi_d_raw - self.psi_d_filtered)

        return _wrap(self.psi_d_filtered)
    
    def _find_closest_segment(self, pos, path):
        """
        Find closest segment in path to current position
        
        Args:
            pos: Current position array [x, y]
            path: List of waypoints [(x, y), ...]
            
        Returns:
            Index of closest segment start point
        """
        min_dist = float('inf')
        closest_idx = 0
        
        for i in range(len(path) - 1):
            p1 = np.array(path[i], dtype=float)
            p2 = np.array(path[i+1], dtype=float)
            
            # Project position onto segment
            segment_vec = p2 - p1
            segment_len = np.linalg.norm(segment_vec)
            
            if segment_len > 1e-6:
                # Parameter t on segment [0, 1]
                t = np.dot(pos - p1, segment_vec) / (segment_len ** 2)
                t = np.clip(t, 0, 1)
                
                # Closest point on segment
                closest_point = p1 + t * segment_vec
                dist = np.linalg.norm(pos - closest_point)
                
                if dist < min_dist:
                    min_dist = dist
                    closest_idx = i
        
        return closest_idx
    
    def reset(self):
        """Reset semua state ILOS (integral + heading filter)."""
        self.cross_track_integral = 0.0
        self.psi_d_filtered       = None
    
    def set_gains(self, k_cross=None, k_integral=None, lookahead=None,
                  cte_reset_threshold=None, lookahead_k=None, psi_filter_tau=None):
        """
        Update ILOS gains dan parameter.

        Args:
            k_cross:              Ignored (kept for API compatibility).
            k_integral:           Integral gain κ.
            lookahead:            Minimum look-ahead distance Δ_min [m].
            cte_reset_threshold:  CTE threshold [m] untuk reset integral.
            lookahead_k:          Multiplier adaptive lookahead.
            psi_filter_tau:       Time constant [s] filter heading reference.
        """
        if k_integral is not None:
            self.k_integral = k_integral
        if lookahead is not None:
            self.lookahead = lookahead
        if cte_reset_threshold is not None:
            self.cte_reset_threshold = cte_reset_threshold
        if lookahead_k is not None:
            self.lookahead_k = lookahead_k
        if psi_filter_tau is not None:
            self.psi_filter_tau = psi_filter_tau
    
    def get_state(self):
        """Get current ILOS state."""
        return {
            'cross_track_integral': self.cross_track_integral,
            'psi_d_filtered':       self.psi_d_filtered,
        }

    def set_state(self, state):
        """Set ILOS state."""
        self.cross_track_integral = state.get('cross_track_integral', 0.0)
        self.psi_d_filtered       = state.get('psi_d_filtered', None)


class ILOSFollowerNode:
    """
    ROS node: subscribe /planned_path_smooth + /odom,
    run ILOS guidance, publish /cmd_vel to usv_simulator_4dof.
    """

    def __init__(self):
        rospy.init_node('ilos_follower', anonymous=False)

        dt                  = rospy.get_param('~dt',                   0.02)
        k_cross             = rospy.get_param('~k_cross',              0.5)
        k_integral          = rospy.get_param('~k_integral',           0.1)
        lookahead           = rospy.get_param('~lookahead',            5.0)
        cte_reset_threshold = rospy.get_param('~cte_reset_threshold',  1.5)
        lookahead_k         = rospy.get_param('~lookahead_k',          1.5)
        psi_filter_tau      = rospy.get_param('~psi_filter_tau',       0.5)
        kp_psi              = rospy.get_param('~kp_psi',               2.0)
        ki_psi              = rospy.get_param('~ki_psi',               0.05)
        kd_psi              = rospy.get_param('~kd_psi',               0.5)
        u_des               = rospy.get_param('~u_des',                1.5)
        goal_tol            = rospy.get_param('~goal_tol',             2.0)
        arrive_dist         = rospy.get_param('~arrive_dist',          0.5)
        fade_in_sec         = rospy.get_param('~fade_in_sec',          4.0)

        self.dt          = dt
        self.u_des       = u_des
        self.goal_tol    = goal_tol
        self.arrive_dist = arrive_dist
        self.fade_in_sec = fade_in_sec

        self.ilos = ILOSFollower(
            k_integral=k_integral, lookahead=lookahead,
            cte_reset_threshold=cte_reset_threshold,
            lookahead_k=lookahead_k, psi_filter_tau=psi_filter_tau)
        self.ilos.dt = dt

        self.pid_heading = PIDController(kp=kp_psi, ki=ki_psi, kd=kd_psi, dt=dt)

        self.path             = []
        self.pos              = np.array([0.0, 0.0])
        self.psi              = 0.0
        self.yaw_rate         = 0.0   # r dari odom, untuk derivative damping
        self.active           = False
        self.started          = False
        self.mission_start_t  = None  # untuk fade-in

        self.pub_cmd   = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub_path  = rospy.Subscriber('/planned_path_smooth', Path,
                                          self._cb_path, queue_size=1)
        self.sub_odom  = rospy.Subscriber('/odom', Odometry,
                                          self._cb_odom, queue_size=1)
        self.sub_cmd   = rospy.Subscriber('/autonomy/mission_manager/state_cmd',
                                          String, self._cb_cmd, queue_size=1)
        self.sub_speed = rospy.Subscriber('/autonomy/mission_manager/speed',
                                          Float64, self._cb_speed, queue_size=1)

        rospy.loginfo('ILOSFollowerNode ready — waiting for /planned_path_smooth + start cmd')

    def _cb_path(self, msg):
        if len(msg.poses) < 2:
            return
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.ilos.reset()
        # Mulai ikut path hanya jika sudah di-start
        if self.started:
            self.active = True
        rospy.loginfo('ILOSFollower: path baru %d titik%s',
                      len(self.path), '' if self.started else ' (tunggu start)')

    def _cb_cmd(self, msg):
        cmd = msg.data
        if cmd == 'start':
            self.started = True
            if self.path:
                self.active          = True
                self.mission_start_t = rospy.Time.now()
                self.ilos.reset()
                rospy.loginfo('ILOSFollower: START — follow %d titik', len(self.path))
        elif cmd == 'stop':
            self.started         = False
            self.active          = False
            self.mission_start_t = None
            self.pid_heading.reset()
            self.ilos.reset()
            self.pub_cmd.publish(Twist())
            rospy.loginfo('ILOSFollower: STOP')
        elif cmd == 'restart':
            self.started         = True
            self.mission_start_t = rospy.Time.now()
            if self.path:
                self.active = True
                self.ilos.reset()
                rospy.loginfo('ILOSFollower: RESTART')

    def _cb_speed(self, msg):
        self.u_des = float(np.clip(msg.data, 0.0, 3.0))
        rospy.loginfo('ILOSFollower: u_des → %.2f m/s', self.u_des)

    def _cb_odom(self, msg):
        self.pos      = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        q             = msg.pose.pose.orientation
        _, _, self.psi = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.yaw_rate = msg.twist.twist.angular.z   # r [rad/s] untuk D-term

    def _control(self):
        if not self.active or len(self.path) < 2:
            self.pub_cmd.publish(Twist())
            return

        goal   = np.array(self.path[-1])
        d_goal = np.linalg.norm(goal - self.pos)

        if d_goal < self.arrive_dist:
            self.active          = False
            self.started         = False
            self.mission_start_t = None
            self.pub_cmd.publish(Twist())
            rospy.loginfo('ILOSFollower: goal reached')
            return

        # Fade-in 4 detik pertama — cegah spike aktuator saat misi baru start
        if self.mission_start_t is not None:
            t_elapsed = (rospy.Time.now() - self.mission_start_t).to_sec()
            fade_in   = min(1.0, t_elapsed / self.fade_in_sec)
        else:
            fade_in = 1.0

        # Terminal guidance: saat d_goal < lookahead, ILOS tidak stabil karena
        # lookahead melewati ujung path → heading langsung ke WP terakhir.
        if d_goal < self.ilos.lookahead:
            psi_des = _wrap(np.arctan2(goal[1] - self.pos[1],
                                       goal[0] - self.pos[0]))
        else:
            psi_des = self.ilos.compute_desired_heading(
                self.pos, self.path, self.psi)

        # Clamp heading error ke ±20° — cegah integral windup & spike kemudi
        # saat kapal menyimpang jauh (misalnya setelah replan D*Lite)
        e_psi = _wrap(psi_des - self.psi)
        e_psi_clamped = float(np.clip(e_psi, -np.deg2rad(20.0), np.deg2rad(20.0)))

        # PID heading: update_with_rate pakai yaw rate sensor untuk D-term
        r_cmd = np.clip(
            self.pid_heading.update_with_rate(e_psi_clamped, self.yaw_rate),
            -1.0, 1.0) * fade_in

        # Heading-first: kurangi surge proporsional terhadap heading error
        # cos(e_psi) → 1.0 saat aligned, 0 saat 90° off, cegah lateral drift
        cos_factor = max(0.0, float(np.cos(e_psi)))
        u = self.u_des * min(1.0, d_goal / self.goal_tol) * fade_in * cos_factor

        cmd = Twist()
        cmd.linear.x  = u
        cmd.angular.z = r_cmd
        self.pub_cmd.publish(cmd)

    def run(self):
        rate = rospy.Rate(int(round(1.0 / self.dt)))
        while not rospy.is_shutdown():
            self._control()
            rate.sleep()


if _ROS_OK and __name__ == '__main__':
    try:
        ILOSFollowerNode().run()
    except rospy.ROSInterruptException:
        pass
