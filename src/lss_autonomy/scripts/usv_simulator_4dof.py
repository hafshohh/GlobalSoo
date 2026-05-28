#!/usr/bin/env python3
"""
USV 4-DOF Simulator — port langsung dari MATLAB cobadin.m
Model kapal LSS-01: surge, sway, yaw, roll
Dinamika internal dalam satuan grid (grid units, 1 cell = CELL_M meter),
konversi ke meter hanya pada interface ROS.

Subscribe : /cmd_vel (Twist)   — linear.x [m/s], angular.z [rad/s]
Publish   : /odom  (Odometry)  — posisi [m], kecepatan [m/s]
            /usv/roll (Float32) — sudut roll [rad]
"""

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler
import tf2_ros

# 1 grid cell = CELL_M meter (sama dengan MATLAB cobadin.m: cell_m = 2)
CELL_M = 2.0


class USV4DOFSimulator:

    def __init__(self):
        rospy.init_node('usv_simulator_4dof', anonymous=False)

        # ---- Initial pose (meter, configurable via ROS params) ----
        x_m  = rospy.get_param('~x_init',   1.0)   # MATLAB start_m = [1, 8]
        y_m  = rospy.get_param('~y_init',   8.0)
        yaw0 = rospy.get_param('~yaw_init', 0.0)

        # Posisi dalam grid units (menggunakan konversi m2g dari MATLAB)
        self.x_g = x_m  / CELL_M + 0.5
        self.y_g = y_m  / CELL_M + 0.5
        self.psi = float(yaw0)   # rad
        self.phi = 0.0           # roll rad

        # nu = [u_g, v_g, r, p]  — u_g/v_g dalam grid/s, r/p dalam rad/s
        self.nu = np.zeros(4)

        # ---- Parameter kapal LSS-01 (identik dengan MATLAB) ----
        # Surge
        self.A1  =  1.5066
        self.A2  = -0.7405
        self.A3  =  0.4219
        self.A4  = -0.1397
        # Sway
        self.A5  = -0.1464
        self.A6  = -3.1952
        self.A7  =  4.1189
        self.A8  =  0.0
        self.A9  =  0.0
        # Input gains
        self.A18 =  0.0178
        self.A19 =  0.02
        # Relasi internal yaw (sama persis dengan MATLAB)
        self.A10 = (self.A1 / self.A18) * self.A19
        self.A11 = (1.0     / self.A18) * self.A19
        # Yaw fitted
        self.A12 = -0.35
        self.A13 =  1.4038
        self.A14 = -2.0764
        self.A15 =  0.0010
        self.A16 =  0.9671
        self.A17 =  0.0021
        self.A20 =  0.0
        self.A21 =  0.0
        self.A22 =  0.0
        # Roll
        self.KpLin  =  0.0
        self.KpAbs  =  0.0
        self.KpCub  =  0.0
        self.Kphi   =  13.5523
        self.Kfy    = -0.0175
        self.Kv_phi = -3.3096   # Kv untuk persamaan roll (beda dari yaw)
        self.Kr_phi = -2.7576   # Kr untuk persamaan roll
        self.Kbias  = -0.3631
        self.g_accel = 9.81

        # ---- Batas aktuator (dari MATLAB lims) ----
        self.lim_TX = rospy.get_param('~lim_TX', 200.0)
        self.lim_TY = rospy.get_param('~lim_TY',  60.0)
        self.lim_TN = rospy.get_param('~lim_TN',  80.0)
        self.lim_TK = rospy.get_param('~lim_TK',  60.0)

        # ---- Controller gains (dari MATLAB controller_guard_4dof_ilos) ----
        self.Ku   = rospy.get_param('~Ku',   80.0)
        self.Kr   = rospy.get_param('~Kr',   55.0)
        self.Kd_r = rospy.get_param('~Kd_r',  4.0)

        # ---- Banking / roll coupling (dari MATLAB) ----
        self.phi_max     = np.deg2rad(rospy.get_param('~phi_max_deg', 10.0))
        self.tau_phi     = 0.25
        self.phi_des_prev = 0.0
        self.eInt_phi    = 0.0
        self.Kphi_p = 6.0
        self.Kphi_i = 0.5
        self.Kphi_d = 3.0

        # ---- Batas state (dari MATLAB) ----
        u_max_ms = rospy.get_param('~u_max_ms', 3.0)   # m/s
        v_max_ms = rospy.get_param('~v_max_ms', 3.0)
        self.u_max_g = u_max_ms / CELL_M   # grid/s
        self.v_max_g = v_max_ms / CELL_M
        self.r_max   = 0.7    # rad/s
        self.p_max   = 2.0    # rad/s

        # ---- Command (diisi oleh callback) ----
        self.u_cmd_g = 0.0   # desired surge [grid/s]
        self.r_cmd   = 0.0   # desired yaw rate [rad/s]

        # ---- Timing ----
        self.dt = rospy.get_param('~dt', 0.02)   # 50 Hz (sama dengan MATLAB)

        # ---- ROS interface ----
        self.sub_cmd  = rospy.Subscriber('/cmd_vel', Twist,
                                         self._cmd_cb, queue_size=1)
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.pub_roll = rospy.Publisher('/usv/roll', Float32, queue_size=1)
        self.tf_br    = tf2_ros.TransformBroadcaster()

        self.timer = rospy.Timer(rospy.Duration(self.dt), self._step)

        rospy.loginfo("[USV 4-DOF] Started | pos=(%.1f, %.1f) m | dt=%.3f s",
                      x_m, y_m, self.dt)

    # ------------------------------------------------------------------
    def _cmd_cb(self, msg):
        """Terima cmd_vel: konversi surge m/s → grid/s."""
        self.u_cmd_g = np.clip(msg.linear.x / CELL_M,
                               -self.u_max_g, self.u_max_g)
        self.r_cmd   = np.clip(msg.angular.z, -self.r_max, self.r_max)

    # ------------------------------------------------------------------
    def _dynamics(self, nu, Fx, Fy):
        """
        4-DOF equations of motion dalam grid units.
        Port langsung dari fungsi usv4dof() di MATLAB cobadin.m.
        nu = [u_g, v_g, r, p]
        Returns: Vdot [4], eta_dot_g [2] = [xdot_g, ydot_g]
        """
        u, v, r, p = nu[0], nu[1], nu[2], nu[3]

        # Surge
        udot = (self.A1 * v * r
                + self.A2 * u
                + self.A3 * abs(u) * u
                + self.A4 * (abs(u) ** 2) * u
                + self.A18 * Fx)

        # Sway
        vdot = (-(1.0 / self.A1) * u * r
                + self.A5 * v
                + self.A6 * abs(v) * v
                + self.A7 * (abs(v) ** 2) * v
                + self.A8 * abs(r) * v
                + self.A9 * abs(v) * r)

        # Yaw
        rdot = (-self.A10 * v * u
                + self.A11 * u * v
                + self.A12 * r
                + self.A13 * abs(r) * r
                + self.A14 * (abs(r) ** 2) * r
                + self.A15 * abs(r) * u
                + self.A16 * abs(u) * r
                + self.A17 * abs(u) * u
                + self.A20 * abs(r) * u
                + self.A21 * abs(u) * r
                + self.A22 * abs(u) * u
                + self.A19 * Fy)

        # Roll
        pdot = (-self.KpLin * p
                - self.KpAbs * abs(p) * p
                - self.KpCub * (abs(p) ** 2) * p
                - self.Kphi  * np.sin(self.phi)
                + self.Kfy   * Fy
                + self.Kv_phi * v
                + self.Kr_phi * r
                + self.Kbias)

        Vdot = np.array([udot, vdot, rdot, pdot])

        # Kinematics → posisi dalam grid/s
        xdot_g = u * np.cos(self.psi) - v * np.sin(self.psi)
        ydot_g = u * np.sin(self.psi) + v * np.cos(self.psi)

        return Vdot, np.array([xdot_g, ydot_g])

    # ------------------------------------------------------------------
    def _compute_forces(self):
        """
        Konversi cmd_vel → forces dalam grid units.
        Port dari controller_guard_4dof_ilos() MATLAB (bagian P/PD control).
        """
        u_g = self.nu[0]
        r   = self.nu[2]

        Fx = self.Ku * (self.u_cmd_g - u_g)
        Fx = np.clip(Fx, -self.lim_TX, self.lim_TX)

        Fy = self.Kr * (self.r_cmd - r) - self.Kd_r * r
        Fy = np.clip(Fy, -self.lim_TY, self.lim_TY)

        return Fx, Fy

    # ------------------------------------------------------------------
    def _compute_banking(self):
        """
        Banking / roll control.
        Port dari bagian BANKING CONTROL di MATLAB cobadin.m.
        """
        u, v = self.nu[0], self.nu[1]
        pp   = self.nu[3]

        # Estimasi lateral acceleration: a_y ≈ U_phys * r  (m/s²)
        U_phys  = max(0.3, np.hypot(u, v) * CELL_M)   # m/s
        a_y_cmd = U_phys * self.r_cmd                  # m/s²

        phi_cmd = (5.0 * np.arctan(a_y_cmd / self.g_accel)
                   + 0.3 * self.phi)
        phi_cmd = np.clip(phi_cmd, -self.phi_max, self.phi_max)

        alpha_phi     = self.dt / (self.tau_phi + self.dt)
        phi_des       = (self.phi_des_prev
                         + alpha_phi * (phi_cmd - self.phi_des_prev))
        self.phi_des_prev = phi_des

        e_phi         = phi_des - self.phi
        self.eInt_phi += e_phi * self.dt

        Tk = (self.Kphi_p * e_phi
              + self.Kphi_i * self.eInt_phi
              - self.Kphi_d * pp)
        Tk = np.clip(Tk, -self.lim_TK, self.lim_TK)
        return Tk

    # ------------------------------------------------------------------
    def _step(self, event):
        """Main simulation step, dipanggil oleh ROS Timer."""
        Fx, Fy = self._compute_forces()
        self._compute_banking()   # update phi_des_prev & eInt_phi

        Vdot, eta_dot = self._dynamics(self.nu, Fx, Fy)

        # Euler integration
        self.nu   = self.nu + self.dt * Vdot
        self.x_g += self.dt * eta_dot[0]
        self.y_g += self.dt * eta_dot[1]
        self.psi += self.dt * self.nu[2]
        self.phi += self.dt * self.nu[3]

        # Clamp states
        self.nu[0] = np.clip(self.nu[0], -self.u_max_g, self.u_max_g)
        self.nu[1] = np.clip(self.nu[1], -self.v_max_g, self.v_max_g)
        self.nu[2] = np.clip(self.nu[2], -self.r_max,   self.r_max)
        self.nu[3] = np.clip(self.nu[3], -self.p_max,   self.p_max)

        # Normalize psi ke [-π, π]
        self.psi = np.arctan2(np.sin(self.psi), np.cos(self.psi))

        if not all(np.isfinite([self.x_g, self.y_g, self.psi, self.phi])):
            rospy.logerr_throttle(2.0,
                "[USV 4-DOF] State non-finite! Resetting velocity.")
            self.nu[:] = 0.0
            return

        self._publish_odom()
        self._publish_tf()
        self.pub_roll.publish(Float32(self.phi))

    # ------------------------------------------------------------------
    def _grid_to_meter(self):
        """Konversi posisi grid → meter (g2m dari MATLAB)."""
        x_m = (self.x_g - 0.5) * CELL_M
        y_m = (self.y_g - 0.5) * CELL_M
        return x_m, y_m

    # ------------------------------------------------------------------
    def _publish_odom(self):
        x_m, y_m = self._grid_to_meter()

        odom = Odometry()
        odom.header.stamp    = rospy.Time.now()
        odom.header.frame_id = 'map'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x = x_m
        odom.pose.pose.position.y = y_m
        odom.pose.pose.position.z = 0.0

        # Orientasi termasuk roll
        quat = quaternion_from_euler(self.phi, 0.0, self.psi)
        odom.pose.pose.orientation = Quaternion(*quat)

        # Kecepatan dalam m/s (konversi grid/s → m/s untuk surge/sway)
        odom.twist.twist.linear.x  = self.nu[0] * CELL_M
        odom.twist.twist.linear.y  = self.nu[1] * CELL_M
        odom.twist.twist.angular.z = self.nu[2]   # yaw rate rad/s
        odom.twist.twist.angular.x = self.nu[3]   # roll rate rad/s

        odom.pose.covariance  = [0.0] * 36
        odom.twist.covariance = [0.0] * 36

        self.pub_odom.publish(odom)

    # ------------------------------------------------------------------
    def _publish_tf(self):
        x_m, y_m = self._grid_to_meter()

        t = TransformStamped()
        t.header.stamp    = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id  = 'base_link'

        t.transform.translation.x = x_m
        t.transform.translation.y = y_m
        t.transform.translation.z = 0.0

        quat = quaternion_from_euler(self.phi, 0.0, self.psi)
        t.transform.rotation = Quaternion(*quat)

        self.tf_br.sendTransform(t)


if __name__ == '__main__':
    try:
        USV4DOFSimulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
