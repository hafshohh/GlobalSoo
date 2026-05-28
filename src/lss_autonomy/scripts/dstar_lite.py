#!/usr/bin/env python3
"""
D* Lite Algorithm for Global Path Planning
Implements the D* Lite incremental heuristic search algorithm

Ditambahkan:
  - OccupancyGridMap  : membangun occupancy grid dari obstacle list (port MATLAB)
  - DStarLiteGrid     : D* Lite pada occupancy grid (port dstarLite_grid() MATLAB)
  - DStarGridNode     : ROS node (aktif saat dijalankan langsung sebagai script)
"""

import numpy as np
import heapq
from collections import defaultdict

try:
    import rospy
    from nav_msgs.msg import Path, OccupancyGrid, Odometry
    from geometry_msgs.msg import PoseStamped, PoseArray
    from tf.transformations import euler_from_quaternion
    _ROS_OK = True
except ImportError:
    _ROS_OK = False

_INF = 1e12


class DStarLite:
    """D* Lite algorithm for global path planning"""
    
    def __init__(self, grid_resolution=0.1):
        """
        Initialize D* Lite planner
        
        Args:
            grid_resolution: Grid cell resolution in meters
        """
        self.resolution = grid_resolution
        self.g_values = defaultdict(lambda: float('inf'))
        self.rhs_values = defaultdict(lambda: float('inf'))
        self.open_set = []
        self.k_m = 0
        self.start = None
        self.goal = None
        
    def heuristic(self, pos1, pos2):
        """
        Euclidean heuristic distance
        
        Args:
            pos1, pos2: (x, y) position tuples
            
        Returns:
            Euclidean distance
        """
        return np.linalg.norm(np.array(pos1) - np.array(pos2))
    
    def calculate_key(self, pos):
        """
        Calculate priority queue key for vertex
        
        Args:
            pos: Position (x, y) tuple
            
        Returns:
            Key tuple (priority, secondary_priority)
        """
        g = self.g_values[pos]
        rhs = self.rhs_values[pos]
        h = self.heuristic(pos, self.goal)
        return (min(g, rhs) + self.k_m + h, min(g, rhs))
    
    def get_neighbors(self, pos, obstacles):
        """
        Get 8-connected neighbors (excluding obstacles)
        
        Args:
            pos: Current position (x, y)
            obstacles: List of obstacle positions
            
        Returns:
            List of valid neighbor positions
        """
        neighbors = []
        x, y = pos
        
        # 8-connected grid
        for dx in [-self.resolution, 0, self.resolution]:
            for dy in [-self.resolution, 0, self.resolution]:
                if dx == 0 and dy == 0:
                    continue
                    
                neighbor = (round(x + dx, 2), round(y + dy, 2))
                
                # Check obstacle collision
                is_obstacle = any(
                    np.linalg.norm(np.array(neighbor) - np.array(obs)) < self.resolution * 1.5
                    for obs in obstacles
                )
                
                if not is_obstacle:
                    neighbors.append(neighbor)
        
        return neighbors
    
    def plan(self, start, goal, obstacles):
        """
        Execute D* Lite path planning
        
        Args:
            start: Start position (x, y) tuple
            goal: Goal position (x, y) tuple
            obstacles: List of obstacle positions [(x, y), ...]
            
        Returns:
            List of waypoints from start to goal
        """
        self.start = tuple([round(s, 2) for s in start])
        self.goal = tuple([round(g, 2) for g in goal])
        
        # Check if start == goal
        if self.heuristic(self.start, self.goal) < self.resolution:
            return [self.start, self.goal]
        
        # Reset state
        self.g_values.clear()
        self.rhs_values.clear()
        self.open_set = []
        self.k_m = 0
        
        # Initialize goal
        self.rhs_values[self.goal] = 0
        heapq.heappush(self.open_set, (self.calculate_key(self.goal), self.goal))
        
        path = []
        current = self.start
        visited = set()
        max_iterations = 2000
        iteration = 0
        
        while len(self.open_set) > 0 and iteration < max_iterations:
            iteration += 1
            
            if not self.open_set:
                break
            
            _, current = heapq.heappop(self.open_set)
            
            if current in visited:
                continue
            visited.add(current)
            
            # Update g-value
            if self.g_values[current] > self.rhs_values[current]:
                self.g_values[current] = self.rhs_values[current]
            else:
                self.g_values[current] = float('inf')
            
            # Check if goal reached
            if self.heuristic(current, self.goal) < self.resolution:
                path.append(self.goal)
                break
            
            path.append(current)
            
            # Expand neighbors
            neighbors = self.get_neighbors(current, obstacles)
            for neighbor in neighbors:
                if neighbor in visited:
                    continue
                
                # Cost = distance + epsilon
                cost = self.resolution + 0.01
                old_rhs = self.rhs_values[current]
                new_rhs = self.g_values[neighbor] + cost
                
                if new_rhs < self.rhs_values[current]:
                    self.rhs_values[current] = new_rhs
                    if current != self.goal:
                        heapq.heappush(self.open_set, (self.calculate_key(current), current))
        
        # Fallback to direct path if D* fails
        if len(path) < 2:
            path = [self.start, self.goal]

        return path


# ===================================================================
# OccupancyGridMap
# Port dari bagian "BANGUN MAP GRID UNTUK D* LITE" di MATLAB cobadin.m
# ===================================================================

class OccupancyGridMap:
    """
    Membangun occupancy grid dari daftar obstacle.
    Setiap obstacle diinflasikan sebesar (radius_fisik + inflate_m)
    sebelum di-mark sebagai occupied — identik dengan MATLAB.

    Koordinat: meter, origin di (ox, oy) sudut kiri-bawah.
    """

    def __init__(self, origin_m, size_m, cell_m=2.0):
        self.ox   = float(origin_m[0])
        self.oy   = float(origin_m[1])
        self.cell = float(cell_m)
        self.nC   = int(np.ceil(size_m[0] / cell_m))   # kolom (arah X)
        self.nR   = int(np.ceil(size_m[1] / cell_m))   # baris  (arah Y)
        self.grid = np.zeros((self.nR, self.nC), dtype=np.uint8)

        # Pusat setiap sel dalam meter [shape: nR x nC]
        xc = self.ox + (np.arange(self.nC) + 0.5) * self.cell
        yc = self.oy + (np.arange(self.nR) + 0.5) * self.cell
        self.xcGrid, self.ycGrid = np.meshgrid(xc, yc)

    def clear(self):
        self.grid[:] = 0

    def add_obstacles(self, obstacles, inflate_m, min_occ_m=None):
        """
        obstacles  : iterable of [cx_m, cy_m, radius_m]
        inflate_m  : = safePlan_m + safeDist_m dari MATLAB (default 0.9 m)
        min_occ_m  : radius minimum agar obstacle selalu masuk 1 sel
                     (default 0.5*sqrt(2)*cell_m, sama dengan MATLAB)
        """
        if min_occ_m is None:
            min_occ_m = 0.5 * np.sqrt(2) * self.cell
        for obs in obstacles:
            cx, cy, r_phys = float(obs[0]), float(obs[1]), float(obs[2])
            r_occ = max(r_phys + inflate_m, min_occ_m)
            mask  = (self.xcGrid - cx)**2 + (self.ycGrid - cy)**2 <= r_occ**2
            self.grid[mask] = 1

    def world_to_grid(self, x_m, y_m):
        """Meter → (col, row) 1-indexed float (sama dengan m2g MATLAB)."""
        col = (x_m - self.ox) / self.cell + 0.5
        row = (y_m - self.oy) / self.cell + 0.5
        return col, row

    def grid_to_world(self, col, row):
        """(col, row) 1-indexed → meter pusat sel (sama dengan g2m MATLAB)."""
        return (self.ox + (col - 0.5) * self.cell,
                self.oy + (row - 0.5) * self.cell)

    def to_ros_msg(self, frame_id='map'):
        """Konversi ke nav_msgs/OccupancyGrid untuk RViz (butuh ROS)."""
        if not _ROS_OK:
            raise RuntimeError("ROS tidak tersedia")
        msg = OccupancyGrid()
        msg.header.stamp    = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.info.resolution = self.cell
        msg.info.width      = self.nC
        msg.info.height     = self.nR
        msg.info.origin.position.x  = self.ox
        msg.info.origin.position.y  = self.oy
        msg.info.origin.orientation.w = 1.0
        msg.data = (self.grid * 100).astype(np.int8).flatten().tolist()
        return msg


# ===================================================================
# DStarLiteGrid
# Port langsung dari dstarLite_grid() MATLAB cobadin.m
# ===================================================================

class DStarLiteGrid:
    """
    D* Lite pada occupancy grid diskrit.
    Semua operasi dalam satuan sel integer (col, row) 1-indexed.
    Identik dengan fungsi dstarLite_grid() di MATLAB cobadin.m.

    Perbedaan utama dari DStarLite (lama):
    - Menggunakan OccupancyGridMap → tidak ada coordinate-based obstacle check
    - Grid diskrit → planning lebih deterministik dan efisien
    - Heuristic bisa di-weight (w >= 1)
    """

    # 8-connected neighbors: (dx, dy, biaya)
    MOVES = [
        ( 1,  0, 1.0),       (-1,  0, 1.0),
        ( 0,  1, 1.0),       ( 0, -1, 1.0),
        ( 1,  1, np.sqrt(2)), (-1, -1, np.sqrt(2)),
        ( 1, -1, np.sqrt(2)), (-1,  1, np.sqrt(2)),
    ]

    def plan(self, occ_map, start_grid, goal_grid, w=1.0):
        """
        Parameters
        ----------
        occ_map    : OccupancyGridMap
        start_grid : (col, row) float — posisi awal, akan di-snap ke integer
        goal_grid  : (col, row) float — posisi tujuan
        w          : bobot heuristic (MATLAB default = 1)

        Returns
        -------
        path_grid : list of (col, row) int, atau None jika tidak ada jalur
        expanded  : jumlah node yang di-ekspansi
        """
        nR, nC = occ_map.nR, occ_map.nC
        free   = (occ_map.grid == 0).copy()   # bisa dilalui

        # Snap ke integer + klem ke batas
        sx = int(np.clip(round(start_grid[0]), 1, nC))
        sy = int(np.clip(round(start_grid[1]), 1, nR))
        gx = int(np.clip(round(goal_grid[0]),  1, nC))
        gy = int(np.clip(round(goal_grid[1]),  1, nR))

        # Start selalu traversable (sama dengan MATLAB)
        free[sy-1, sx-1] = True

        # Goal di obstacle → geser ke sel bebas terdekat
        if not free[gy-1, gx-1]:
            gx, gy = self._nearest_free(free, gx, gy, nC, nR)

        # ----- Inisialisasi D* Lite -----
        N   = nR * nC
        g   = np.full(N, _INF, dtype=np.float64)
        rhs = np.full(N, _INF, dtype=np.float64)

        def sid(x, y):   return (y - 1) * nC + (x - 1)
        def sxy(i):      return (i % nC + 1, i // nC + 1)

        s0 = sid(sx, sy)   # start
        sg = sid(gx, gy)   # goal

        def h(a, b):
            xa, ya = sxy(a)
            xb, yb = sxy(b)
            return w * np.hypot(xa - xb, ya - yb)

        def calc_key(s):
            mg = min(g[s], rhs[s])
            return (mg + h(s0, s), mg)

        # Heap dengan lazy deletion
        heap    = []
        in_heap = {}   # s → key aktif

        def push(s):
            k = calc_key(s)
            heapq.heappush(heap, (k, s))
            in_heap[s] = k

        def pop_best():
            while heap:
                k, s = heapq.heappop(heap)
                if in_heap.get(s) == k:
                    del in_heap[s]
                    return k, s
            return (_INF, _INF), -1

        def top_key():
            while heap:
                k, s = heap[0]
                if in_heap.get(s) == k:
                    return k
                heapq.heappop(heap)
            return (_INF, _INF)

        def succs(s):
            x0, y0 = sxy(s)
            res = []
            for dx, dy, cost in self.MOVES:
                nx, ny = x0 + dx, y0 + dy
                if 1 <= nx <= nC and 1 <= ny <= nR and free[ny-1, nx-1]:
                    res.append((sid(nx, ny), cost))
            return res

        def update_vertex(u):
            if u != sg:
                ss = succs(u)
                rhs[u] = min((c + g[ns] for ns, c in ss), default=_INF)
            if u in in_heap:
                del in_heap[u]
            if g[u] != rhs[u]:
                push(u)

        def compute_path():
            expanded = 0
            while True:
                k_top = top_key()
                k_s   = calc_key(s0)
                if not (k_top < k_s or rhs[s0] != g[s0]):
                    break
                k_old, u = pop_best()
                if u == -1:
                    break
                expanded += 1
                if k_old < calc_key(u):
                    push(u)
                elif g[u] > rhs[u]:
                    g[u] = rhs[u]
                    for p, _ in succs(u):
                        update_vertex(p)
                else:
                    g[u] = _INF
                    update_vertex(u)
                    for p, _ in succs(u):
                        update_vertex(p)
            return expanded

        # ----- Jalankan dari goal (backward) -----
        rhs[sg] = 0
        push(sg)
        expanded = compute_path()

        if np.isinf(g[s0]):
            return None, expanded

        # ----- Ekstrak path (greedy descent) -----
        path    = [(sx, sy)]
        cur     = s0
        visited = {cur}

        for _ in range(nR * nC):
            if cur == sg:
                break
            ss = succs(cur)
            if not ss:
                return None, expanded
            best, _ = min(ss, key=lambda nc: nc[1] + g[nc[0]])
            if best in visited or np.isinf(g[best]):
                break
            visited.add(best)
            cur = best
            path.append(sxy(cur))

        return path, expanded

    @staticmethod
    def _nearest_free(free, gx, gy, nC, nR):
        for r in range(1, max(nC, nR) + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = gx + dx, gy + dy
                    if 1 <= nx <= nC and 1 <= ny <= nR and free[ny-1, nx-1]:
                        return nx, ny
        return gx, gy


# ===================================================================
# ROS Node — aktif hanya saat dstar_lite.py dijalankan langsung
# ===================================================================

if _ROS_OK and __name__ == '__main__':

    class DStarGridNode:
        """
        ROS node yang menjalankan DStarLiteGrid.
        Berjalan sejajar dengan path_planner_dstar_apf.py (topic berbeda).

        Subscribe : /odom, /waypoints/mission_1/in, /detected_obstacles
        Publish   : /planned_path_grid, /occupancy_grid_dstar
        """

        def __init__(self):
            rospy.init_node('dstar_grid_planner', anonymous=False)

            # Param peta
            self.cell_m  = rospy.get_param('~cell_m',    2.0)
            self.map_w   = rospy.get_param('~map_w',    50.0)
            self.map_h   = rospy.get_param('~map_h',    33.0)
            self.map_ox  = rospy.get_param('~map_ox',    0.0)
            self.map_oy  = rospy.get_param('~map_oy',    0.0)
            self.auto_ext = rospy.get_param('~auto_extent', True)
            self.pad_m   = rospy.get_param('~padding',   5.0)

            # Param obstacle
            self.safe_dist = rospy.get_param('~safe_dist_m', 0.6)
            self.safe_plan = rospy.get_param('~safe_plan_m', 0.3)
            self.inflate   = self.safe_plan + self.safe_dist   # = 0.9 m

            # Obstacle statis default = MATLAB obstacles_static_m
            default_obs = [
                [20.0, 20.0, 0.25], [40.0, 20.0, 0.25],
                [10.0, 10.0, 0.25], [30.0, 10.0, 0.25],
                [17.0, 16.5, 0.25], [41.0, 16.0, 0.25],
            ]
            self.static_obs = rospy.get_param('~static_obstacles', default_obs)
            self.extra_obs  = []

            self.w_h    = rospy.get_param('~w_heuristic', 1.0)
            self.pos_m  = None
            self.wps    = []
            self.active = False
            self.boundary_polygon = []  # batas perairan dari Web UI

            self.planner = DStarLiteGrid()

            rospy.Subscriber('/odom', Odometry, self._odom_cb, queue_size=1)
            rospy.Subscriber('/waypoints/mission_1/in', Path,
                             self._wp_cb, queue_size=1)
            rospy.Subscriber('/detected_obstacles', PoseArray,
                             self._obs_cb, queue_size=1)
            rospy.Subscriber('/water_boundary', Path,
                             self._boundary_cb, queue_size=1)

            self.pub_path   = rospy.Publisher('/planned_path_grid',   Path,
                                              queue_size=1, latch=True)
            self.pub_global = rospy.Publisher('/planned_path_global', Path,
                                              queue_size=1, latch=True)
            self.pub_grid   = rospy.Publisher('/occupancy_grid_dstar', OccupancyGrid,
                                              queue_size=1, latch=True)

            rospy.loginfo("[D*Grid] Ready | cell=%.1fm | inflate=%.2fm",
                          self.cell_m, self.inflate)

        def _odom_cb(self, msg):
            self.pos_m = (msg.pose.pose.position.x,
                          msg.pose.pose.position.y)

        def _wp_cb(self, msg):
            if not msg.poses:
                return
            self.wps    = [(p.pose.position.x, p.pose.position.y)
                           for p in msg.poses]
            self.active = True
            rospy.loginfo("[D*Grid] %d waypoints → plan seluruh rute", len(self.wps))
            self._plan()

        def _boundary_cb(self, msg):
            self.boundary_polygon = [(p.pose.position.x, p.pose.position.y)
                                     for p in msg.poses]
            rospy.loginfo("[D*Grid] %d titik batas perairan → replan",
                          len(self.boundary_polygon))
            if self.wps:
                self._plan()

        def _obs_cb(self, msg):
            new = [[p.position.x, p.position.y,
                    p.position.z if p.position.z > 0 else 0.25]
                   for p in msg.poses]
            if new != self.extra_obs:
                self.extra_obs = new
                rospy.loginfo("[D*Grid] %d obstacle baru → replan",
                              len(new))
                self._plan()

        def _build_map(self, start_m, goal_m):
            if self.auto_ext:
                all_obs = self.static_obs + self.extra_obs
                xs = ([start_m[0], goal_m[0]]
                      + [w[0] for w in self.wps]
                      + [o[0] for o in all_obs])
                ys = ([start_m[1], goal_m[1]]
                      + [w[1] for w in self.wps]
                      + [o[1] for o in all_obs])
                p   = self.pad_m
                ox  = min(xs) - p;  oy  = min(ys) - p
                w_m = max(xs) - ox + p;  h_m = max(ys) - oy + p
            else:
                ox, oy, w_m, h_m = (self.map_ox, self.map_oy,
                                     self.map_w,  self.map_h)

            occ = OccupancyGridMap((ox, oy), (w_m, h_m), self.cell_m)
            all_obs = list(self.static_obs) + list(self.extra_obs)
            if all_obs:
                occ.add_obstacles(all_obs, self.inflate)

            # Tandai sel di luar batas perairan sebagai obstacle
            if len(self.boundary_polygon) >= 3:
                poly = np.array(self.boundary_polygon)
                wx = occ.ox + (np.arange(occ.nC) + 0.5) * occ.cell
                wy = occ.oy + (np.arange(occ.nR) + 0.5) * occ.cell
                WX, WY = np.meshgrid(wx, wy)
                xs_flat, ys_flat = WX.ravel(), WY.ravel()
                inside = np.zeros(len(xs_flat), dtype=bool)
                px, py = poly[:, 0], poly[:, 1]
                j = len(poly) - 1
                for i in range(len(poly)):
                    dpy = py[j] - py[i]
                    cond = (py[i] > ys_flat) != (py[j] > ys_flat)
                    with np.errstate(divide='ignore', invalid='ignore'):
                        cross = np.where(np.abs(dpy) > 1e-10,
                                         (px[j]-px[i])*(ys_flat-py[i])/dpy + px[i],
                                         np.inf)
                    inside ^= cond & (xs_flat < cross)
                    j = i
                occ.grid |= (~inside).reshape(occ.nR, occ.nC).astype(np.uint8)

            return occ

        def _plan(self):
            if self.pos_m is None or not self.wps:
                return

            full_path = []
            seg_start = self.pos_m
            last_occ  = None

            for i, wp in enumerate(self.wps):
                occ     = self._build_map(seg_start, wp)
                last_occ = occ
                start_g = occ.world_to_grid(*seg_start)
                goal_g  = occ.world_to_grid(*wp)

                rospy.loginfo("[D*Grid] WP%d: (%.1f,%.1f)→(%.1f,%.1f) | grid %dx%d",
                              i + 1, seg_start[0], seg_start[1],
                              wp[0], wp[1], occ.nC, occ.nR)

                t0 = rospy.Time.now()
                path_g, expanded = self.planner.plan(occ, start_g, goal_g, self.w_h)
                dt = (rospy.Time.now() - t0).to_sec()

                if path_g is None:
                    rospy.logwarn("[D*Grid] WP%d: path tidak ditemukan, pakai garis lurus",
                                  i + 1)
                    seg_path = [seg_start, wp]
                else:
                    seg_path = [occ.grid_to_world(c, r) for c, r in path_g]
                    seg_path[0]  = seg_start
                    seg_path[-1] = wp
                    rospy.loginfo("[D*Grid] WP%d: %d titik | %d expanded | %.3fs",
                                  i + 1, len(seg_path), expanded, dt)

                # Gabung: hindari duplikat titik sambungan
                full_path.extend(seg_path if not full_path else seg_path[1:])
                seg_start = wp

            if not full_path:
                return

            # Publish seluruh path sekaligus
            pm = Path()
            pm.header.stamp    = rospy.Time.now()
            pm.header.frame_id = 'map'
            for (x, y) in full_path:
                ps = PoseStamped()
                ps.header = pm.header
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.orientation.w = 1.0
                pm.poses.append(ps)
            self.pub_path.publish(pm)
            self.pub_global.publish(pm)   # alias untuk web UI & mission_control

            if last_occ is not None:
                self.pub_grid.publish(last_occ.to_ros_msg())

            rospy.loginfo("[D*Grid] Seluruh rute: %d titik total", len(full_path))

    node = DStarGridNode()
    rospy.spin()
