#!/usr/bin/env python3
"""
Path Smoother — G2-CBS C² + Obstacle Clearance Enforcement
Port dari smooth_path_g2cbs_c2() dan enforce_safe_clearance() di MATLAB cobadin.m

Alur (sama persis dengan MATLAB cobadin.m):
  1. Hapus titik duplikat
  2. Sederhanakan dengan RDP (opsional)
  3. Arc-length parameterization
  4. Hitung second derivative natural spline (Thomas algorithm)
  5. Hitung first derivative di setiap knot (G2-CBS matching)
  6. Bangun segmen Bezier kubik → path halus
  7. enforce_clearance: dorong titik path menjauhi obstacle

Kelas utama:
  PathSmoother  — pure Python, tidak butuh ROS

Jika dijalankan langsung (python3 path_smoother.py), berjalan sebagai ROS node:
  Subscribe : /planned_path_grid  (nav_msgs/Path)  — raw path dari D* Lite
              /detected_obstacles (geometry_msgs/PoseArray) — obstacle runtime
  Publish   : /planned_path_smooth (nav_msgs/Path) — path setelah smoothing
"""

import numpy as np

try:
    import rospy
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped, PoseArray
    _ROS_OK = True
except ImportError:
    _ROS_OK = False


# ================================================================== #
class PathSmoother:
    """
    G2-CBS C² path smoothing + obstacle clearance enforcement.
    Semua operasi dalam satuan meter. Tidak ada dependensi ROS.

    Contoh penggunaan:
        smoother = PathSmoother()

        # Haluskan path hasil D* Lite
        path_smooth = smoother.smooth(path_xy, n_per_seg=25, eps_rdp=0.6)

        # Paksa jarak aman dari obstacle
        path_final, info = smoother.enforce_clearance(
            path_smooth, obstacles_m, safe_dist_m=0.3)

        print(f"Min clearance: {info['min_clearance']:.3f} m")
    """

    def smooth(self, path_xy, n_per_seg=25, eps_rdp=0.6):
        """
        G2-continuous cubic Bezier spline (C²).
        Port dari smooth_path_g2cbs_c2() MATLAB cobadin.m.

        Parameters
        ----------
        path_xy   : array-like (N, 2) dalam meter
        n_per_seg : jumlah sampel per segmen Bezier (MATLAB default 25)
        eps_rdp   : threshold RDP dalam meter
                    (MATLAB epsRDP=0.3 grid unit = 0.6 m dengan cell_m=2)

        Returns
        -------
        Ps : ndarray (M, 2) path halus dalam meter
        """
        P = self._rm_dups(np.asarray(path_xy, dtype=float))

        if len(P) <= 2:
            return P

        # Sederhanakan dengan RDP
        if eps_rdp > 0 and len(P) > 3:
            P = self._rdp(P, eps_rdp)
            P = self._rm_dups(P)
            if len(P) <= 2:
                return P

        N = len(P)

        # Arc-length parameterization
        t = np.zeros(N)
        for i in range(1, N):
            t[i] = t[i-1] + np.linalg.norm(P[i] - P[i-1])
        if t[-1] < 1e-9:
            return P[:1]

        h = np.diff(t)

        # Natural spline second derivatives (Thomas algorithm)
        Mx = self._spline_M(t, P[:, 0])
        My = self._spline_M(t, P[:, 1])

        # First derivatives di setiap knot (G2 matching — identik MATLAB)
        sx = np.diff(P[:, 0]) / h
        sy = np.diff(P[:, 1]) / h
        mx = np.zeros(N)
        my = np.zeros(N)

        mx[0] = sx[0]  - h[0]  * (2*Mx[0]  + Mx[1])   / 6
        my[0] = sy[0]  - h[0]  * (2*My[0]  + My[1])   / 6
        for i in range(1, N - 1):
            mx[i] = 0.5 * (
                (sx[i-1] + h[i-1] * (Mx[i-1] + 2*Mx[i]) / 6) +
                (sx[i]   - h[i]   * (2*Mx[i] + Mx[i+1]) / 6)
            )
            my[i] = 0.5 * (
                (sy[i-1] + h[i-1] * (My[i-1] + 2*My[i]) / 6) +
                (sy[i]   - h[i]   * (2*My[i] + My[i+1]) / 6)
            )
        mx[-1] = sx[-1] + h[-1] * (Mx[-2] + 2*Mx[-1]) / 6
        my[-1] = sy[-1] + h[-1] * (My[-2] + 2*My[-1]) / 6

        # Bangun segmen Bezier kubik C²
        Ps = P[:1].copy()
        for i in range(N - 1):
            hi = h[i]
            b0 = P[i];     b3 = P[i+1]
            b1 = b0 + (hi / 3) * np.array([mx[i],     my[i]])
            b2 = b3 - (hi / 3) * np.array([mx[i + 1], my[i + 1]])

            tau = np.linspace(0, 1, n_per_seg)[:, None]
            B   = ((1 - tau)**3       * b0
                   + 3*(1 - tau)**2*tau * b1
                   + 3*(1 - tau)*tau**2 * b2
                   + tau**3             * b3)

            # Sambung antar segmen (lewati titik junction yang duplikat)
            Ps = np.vstack([Ps, B[1:] if i > 0 else B])

        return self._rm_dups(Ps)

    def enforce_clearance(self, path_xy, obstacles_m, safe_dist_m,
                           max_iter=80, gain=0.6, max_step=0.2,
                           lam=0.15, ds=0.5):
        """
        Dorong titik path menjauhi obstacle (iteratif).
        Port dari enforce_safe_clearance() MATLAB cobadin.m.

        Parameters
        ----------
        path_xy     : array-like (N, 2) meter
        obstacles_m : list/array of [cx, cy, radius] meter
        safe_dist_m : jarak aman = safePlan_m dari MATLAB (default 0.3 m)
        max_iter    : iterasi maksimum
        gain        : gain repulsi (MATLAB default 0.6)
        max_step    : batas pergeseran per step (MATLAB 0.1–0.2)
        lam         : gain smoothing (MATLAB lambda 0.05–0.15)
        ds          : spasi resampling meter (MATLAB ds=0.25 grid = 0.5 m)

        Returns
        -------
        P    : ndarray (M, 2) path dengan clearance terjaga
        info : dict dengan 'min_clearance' [m] dan 'iterations'
        """
        P = self._resample(np.asarray(path_xy, dtype=float), ds)
        N = len(P)

        if N <= 2 or not obstacles_m:
            return P, {'min_clearance': float('inf'), 'iterations': 0}

        obs = np.asarray(obstacles_m, dtype=float)   # (K, 3)

        it = 0
        for it in range(max_iter):
            dP  = np.zeros_like(P)
            vio = False

            for i in range(1, N - 1):
                # Repulsi dari semua obstacle secara vektorisasi
                v    = P[i] - obs[:, :2]              # (K, 2)
                d    = np.linalg.norm(v, axis=1)      # (K,)
                R    = obs[:, 2] + safe_dist_m        # (K,)
                mask = d < R

                push = np.zeros(2)
                if mask.any():
                    dirs = v[mask] / (d[mask, None] + 1e-9)
                    push = (gain * (R[mask] - d[mask])[:, None] * dirs).sum(axis=0)
                    vio  = True

                smooth  = lam * ((P[i-1] + P[i+1]) / 2 - P[i])
                delta   = push + smooth
                nm = np.linalg.norm(delta)
                if nm > max_step:
                    delta *= max_step / nm
                dP[i] = delta

            P[1:-1] += dP[1:-1]

            if not vio:
                break

        P   = self._rm_dups(P)
        clr = self._min_clearance(P, obs, safe_dist_m)
        return P, {'min_clearance': float(clr), 'iterations': it + 1}

    def shortcut(self, path_xy, obstacles, safe_dist_m, step=0.5):
        """
        Greedy line-of-sight shortcutting.
        Hapus titik perantara yang bisa dijangkau langsung tanpa menabrak obstacle.
        Efektif mengurangi staircase pattern dari D* Lite grid (cell=2m → deviasi ~1m).

        Parameters
        ----------
        path_xy    : (N,2) array meter
        obstacles  : list of [cx, cy, radius] meter
        safe_dist_m: inflate yang sama dengan planning (safe_plan + safe_dist)
        step       : resolusi pengecekan LOS dalam meter (default 0.5)

        Returns
        -------
        ndarray (M,2) dengan M <= N
        """
        P = np.asarray(path_xy, dtype=float)
        if len(P) <= 2 or not obstacles:
            return P
        obs = np.asarray(obstacles, dtype=float)  # (K,3)

        result = [P[0]]
        i = 0
        while i < len(P) - 1:
            # Cari titik terjauh yang bisa dicapai langsung dari P[i]
            j = len(P) - 1
            while j > i + 1:
                if self._los_clear(P[i], P[j], obs, safe_dist_m, step):
                    break
                j -= 1
            result.append(P[j])
            i = j
        return np.array(result)

    def _los_clear(self, p1, p2, obs, safe_dist_m, step=0.5):
        """True jika segmen p1→p2 bebas dari semua obstacle (dengan clearance)."""
        dist = float(np.linalg.norm(p2 - p1))
        if dist < 1e-6:
            return True
        n   = max(3, int(np.ceil(dist / step)))
        ts  = np.linspace(0, 1, n)
        pts = p1[None, :] + ts[:, None] * (p2 - p1)[None, :]  # (n,2)
        for o in obs:
            if np.any(np.linalg.norm(pts - o[:2], axis=1) < o[2] + safe_dist_m):
                return False
        return True

    # ----------------------------------------------------------------
    # Helper methods (private)
    # ----------------------------------------------------------------

    @staticmethod
    def _rm_dups(P, tol=1e-8):
        """Hapus titik duplikat berurutan."""
        if len(P) == 0:
            return P
        keep = np.concatenate(
            [[True], np.linalg.norm(np.diff(P, axis=0), axis=1) > tol])
        return P[keep]

    @staticmethod
    def _resample(P, ds):
        """Resample polyline dengan spasi arc-length = ds."""
        if len(P) < 2 or ds <= 0:
            return P
        s  = np.concatenate(
            [[0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
        ss = np.arange(0, s[-1], ds)
        if len(ss) == 0 or ss[-1] < s[-1]:
            ss = np.append(ss, s[-1])
        return np.column_stack([
            np.interp(ss, s, P[:, 0]),
            np.interp(ss, s, P[:, 1]),
        ])

    @staticmethod
    def _rdp(P, eps):
        """Ramer-Douglas-Peucker simplification."""
        def _max_dist(pts):
            A, B = pts[0], pts[-1]
            AB   = B - A
            L2   = max(1e-12, float(np.dot(AB, AB)))
            dm, ix = -1.0, 0
            for i in range(1, len(pts) - 1):
                t = float(np.clip(np.dot(pts[i] - A, AB) / L2, 0, 1))
                d = float(np.linalg.norm(pts[i] - (A + t * AB)))
                if d > dm:
                    dm, ix = d, i
            return dm, ix

        def _inner(pts):
            if len(pts) <= 2:
                return pts
            dm, ix = _max_dist(pts)
            if dm > eps:
                L = _inner(pts[:ix + 1])
                R = _inner(pts[ix:])
                return np.vstack([L[:-1], R])
            return np.array([pts[0], pts[-1]])

        return _inner(np.asarray(P, dtype=float))

    @staticmethod
    def _spline_M(t, y):
        """
        Second derivative untuk natural cubic spline via Thomas algorithm.
        Port dari natural_spline_second_derivs() MATLAB cobadin.m.
        Natural BC: M[0] = M[N-1] = 0.
        """
        N = len(y)
        h = np.diff(t)
        if N <= 2:
            return np.zeros(N)

        # Tridiagonal system: a[i]*M[i-1] + b[i]*M[i] + c[i]*M[i+1] = d[i]
        a = np.zeros(N); b = np.ones(N)
        c = np.zeros(N); d = np.zeros(N)

        for i in range(1, N - 1):
            a[i] = h[i - 1]
            b[i] = 2.0 * (h[i - 1] + h[i])
            c[i] = h[i]
            d[i] = 6.0 * ((y[i+1] - y[i]) / h[i] -
                           (y[i]   - y[i-1]) / h[i-1])

        # Thomas forward sweep
        for i in range(1, N):
            if abs(b[i - 1]) < 1e-14:
                continue
            m     = a[i] / b[i - 1]
            b[i] -= m * c[i - 1]
            d[i] -= m * d[i - 1]

        # Back substitution
        M = np.zeros(N)
        M[-1] = d[-1] / b[-1] if abs(b[-1]) > 1e-14 else 0.0
        for i in range(N - 2, -1, -1):
            if abs(b[i]) > 1e-14:
                M[i] = (d[i] - c[i] * M[i + 1]) / b[i]

        return M

    @staticmethod
    def _min_clearance(P, obs, sd):
        """Minimum clearance dari semua titik path ke semua obstacle."""
        mc = float('inf')
        for o in obs:
            d  = np.linalg.norm(P - o[:2], axis=1) - (o[2] + sd)
            mc = min(mc, float(d.min()))
        return mc


# ================================================================== #
# ROS Node — aktif saat dijalankan langsung: python3 path_smoother.py
# ================================================================== #

if _ROS_OK and __name__ == '__main__':

    class PathSmootherNode:
        """
        ROS node yang membaca path raw dari /planned_path_grid,
        menghaluskannya, lalu mempublikasikan ke /planned_path_smooth.

        Subscribe:
          /planned_path_grid   (nav_msgs/Path)         — raw D* Lite path
          /detected_obstacles  (geometry_msgs/PoseArray) — obstacle runtime

        Publish:
          /planned_path_smooth (nav_msgs/Path) — path setelah G2-CBS C²
                                                  + clearance enforcement
        """

        def __init__(self):
            rospy.init_node('path_smoother', anonymous=False)

            # Param smoothing
            self.n_per_seg      = rospy.get_param('~n_per_seg',       25)
            self.eps_rdp        = rospy.get_param('~eps_rdp',         0.6)
            self.safe_dist_m    = rospy.get_param('~safe_dist_m',     0.3)
            # safe_shortcut_m: clearance untuk LOS shortcutter.
            # Harus = safe_plan_m + safe_dist_m dari D* Lite (default 0.9 m)
            # agar shortcutter tidak memotong path yang D* Lite hindari.
            self.safe_shortcut_m = rospy.get_param('~safe_shortcut_m', 0.9)
            self.max_iter       = rospy.get_param('~max_iter',         80)
            self.gain           = rospy.get_param('~gain',             0.6)
            self.max_step       = rospy.get_param('~max_step',         0.2)
            self.lam            = rospy.get_param('~lambda',           0.15)
            self.ds             = rospy.get_param('~ds',               0.5)

            # Obstacle statis default = MATLAB obstacles_static_m
            default_obs = [
                [20.0, 20.0, 0.25], [40.0, 20.0, 0.25],
                [10.0, 10.0, 0.25], [30.0, 10.0, 0.25],
                [17.0, 16.5, 0.25], [41.0, 16.0, 0.25],
            ]
            self.static_obs = rospy.get_param('~static_obstacles', default_obs)
            self.extra_obs  = []

            self.smoother   = PathSmoother()
            self.raw_path   = None

            rospy.Subscriber('/planned_path_grid', Path,
                             self._raw_path_cb, queue_size=1)
            rospy.Subscriber('/detected_obstacles', PoseArray,
                             self._obs_cb, queue_size=1)

            self.pub = rospy.Publisher('/planned_path_smooth', Path,
                                       queue_size=1, latch=True)

            rospy.loginfo("[Smoother] Ready | eps_rdp=%.2fm | safe=%.2fm | shortcut=%.2fm",
                          self.eps_rdp, self.safe_dist_m, self.safe_shortcut_m)

        def _raw_path_cb(self, msg):
            if len(msg.poses) < 2:
                return
            self.raw_path = np.array(
                [(p.pose.position.x, p.pose.position.y) for p in msg.poses])
            self._smooth_and_publish()

        def _obs_cb(self, msg):
            self.extra_obs = [
                [p.position.x, p.position.y,
                 p.position.z if p.position.z > 0 else 0.25]
                for p in msg.poses
            ]
            if self.raw_path is not None:
                self._smooth_and_publish()

        def _smooth_and_publish(self):
            start = self.raw_path[0].copy()
            goal  = self.raw_path[-1].copy()

            all_obs = list(self.static_obs) + list(self.extra_obs)
            n_raw   = len(self.raw_path)

            # LOS shortcutting dihapus — RDP di dalam smooth() menangani
            # staircase grid secara geometris tanpa risiko memotong detour
            # yang dibuat D* Lite untuk menghindari obstacle.

            # 1) G2-CBS C² smoothing (RDP internal sudah mereduksi staircase)
            t1   = rospy.Time.now()
            P    = self.smoother.smooth(self.raw_path, self.n_per_seg, self.eps_rdp)
            dt_s = (rospy.Time.now() - t1).to_sec()

            # 2) Clearance enforcement
            t2      = rospy.Time.now()
            P, info = self.smoother.enforce_clearance(
                P, all_obs, self.safe_dist_m,
                max_iter=self.max_iter, gain=self.gain,
                max_step=self.max_step, lam=self.lam, ds=self.ds)
            dt_c = (rospy.Time.now() - t2).to_sec()

            # Paksa endpoint tepat
            P[0]  = start
            P[-1] = goal

            rospy.loginfo(
                "[Smoother] raw%d → out%d | spl %.3fs | "
                "clr %.3fs | min_clr=%.3fm (%d iter)",
                n_raw, len(P),
                dt_s, dt_c,
                info['min_clearance'], info['iterations'])

            # Publish
            out             = Path()
            out.header.stamp    = rospy.Time.now()
            out.header.frame_id = 'map'
            for (x, y) in P:
                ps = PoseStamped()
                ps.header = out.header
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.orientation.w = 1.0
                out.poses.append(ps)
            self.pub.publish(out)

    PathSmootherNode()
    rospy.spin()
