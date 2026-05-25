%% usv_simulation.m
% Simulasi USV 4-DOF: D* Lite + G2-CBS C² + ILOS + PID
%
% Port dari implementasi ROS (lss_01_Hafshoh):
%   usv_simulator_4dof.py     → Dinamika kapal LSS-01
%   path_planner_dstar_apf.py → D* Lite + ILOS + PID
%   dstar_lite.py             → OccupancyGridMap + DStarLiteGrid
%   path_smoother.py          → G2-CBS C² + enforce_clearance
%   ilos_follower.py          → Integral Line-of-Sight guidance
%   pid_controller.py         → PID dengan derivative on measurement
%
% Untuk menambah obstacle statis yang muncul di tengah simulasi:
%   1. Edit bagian "OBSTACLE DINAMIS" di bawah
%   2. Sistem akan replan otomatis saat t >= waktu_muncul
%
% Diperlukan MATLAB R2019b+ (vecnorm, yline)

clear; clc; close all;

%% ===== SKALA GRID & PETA (METER) =====
cell_m = 2;                 % 1 grid cell = 2 meter

mapSize_m = [33 50];        % [Y X] tinggi & lebar peta dalam meter
nR = ceil(mapSize_m(1)/cell_m);   % 17 rows (arah Y)
nC = ceil(mapSize_m(2)/cell_m);   % 25 cols (arah X)
mapSize = [nR nC];

% Fungsi bantu konversi (identik dengan MATLAB cobadin.m)
m2g = @(p_m) [p_m(:,1)/cell_m + 0.5, p_m(:,2)/cell_m + 0.5]; % meter [x,y] -> grid [col,row]
g2m = @(pg)  [(pg(:,1)-0.5)*cell_m,   (pg(:,2)-0.5)*cell_m];  % grid [col,row] -> meter [x,y]

%% ===== OBSTACLES, START, GOAL (METER) =====
% obstacles_*_m = [cx cy radius_m]
obstacles_static_m = [20.0 20.0 0.25;   % kiri atas
                      40.0 20.0 0.25;   % kanan atas
                      10.0 10.0 0.25;   % kiri bawah
                      30.0 10.0 0.25;   % kanan bawah
                      17.0 16.5 0.25;   % tambahan 1
                      41.0 16.0 0.25];  % tambahan 2

% --- DESAIN DALAM METER ---
start_m    = [ 1.0  8.0];
waypoint_m = [25.0 20.0];
goal_m     = [48.0 13.0];

% --- METER -> GRID (boleh non-integer) ---
start    = m2g(start_m);
waypoint = m2g(waypoint_m);
goal_g   = m2g(goal_m);

%% ===== PARAMETER KAPAL LSS-01 =====
% (identik dengan usv_simulator_4dof.py)
% --- Surge ---
A1  =  1.5066;  A2  = -0.7405;  A3  =  0.4219;  A4  = -0.1397;
% --- Sway ---
A5  = -0.1464;  A6  = -3.1952;  A7  =  4.1189;  A8  = 0.0;  A9  = 0.0;
% --- Input gains ---
A18 =  0.0178;  A19 =  0.02;
% --- Yaw (derived) ---
A10 = (A1 / A18) * A19;
A11 = (1.0 / A18) * A19;
% --- Yaw fitted ---
A12 = -0.35;    A13 =  1.4038;  A14 = -2.0764;
A15 =  0.0010;  A16 =  0.9671;  A17 =  0.0021;
A20 =  0.0;     A21 =  0.0;     A22 =  0.0;
% --- Roll ---
KpLin  =  0.0;     KpAbs  =  0.0;  KpCub  =  0.0;
Kphi   =  13.5523; Kfy    = -0.0175;
Kv_phi = -3.3096;  Kr_phi = -2.7576; Kbias  = -0.3631;
g_accel = 9.81;

%% ===== BATAS AKTUATOR =====
lim_TX = 200.0;   % surge force limit
lim_TY =  60.0;   % yaw/sway force limit
lim_TK =  60.0;   % roll moment limit

%% ===== GAINS FORCE CONTROLLER =====
% (identik dengan usv_simulator_4dof.py)
Ku   = 80.0;   % surge P-gain
Kr   = 55.0;   % yaw P-gain
Kd_r =  4.0;   % yaw D-gain

%% ===== BATAS STATE =====
u_max_ms = 3.0;  v_max_ms = 3.0;
u_max_g  = u_max_ms / cell_m;   % 1.5 grid/s
v_max_g  = v_max_ms / cell_m;   % 1.5 grid/s
r_max    = 0.7;                  % rad/s
p_max    = 2.0;                  % rad/s

%% ===== BANKING CONTROL =====
% (identik dengan usv_simulator_4dof.py)
phi_max  = deg2rad(10.0);
tau_phi  = 0.25;
Kphi_p   = 6.0;
Kphi_i   = 0.5;
Kphi_d   = 3.0;

%% ===== ILOS GUIDANCE =====
% (dari path_planner_integrated.launch)
k_integral     = 0.3;
lookahead_m    = 2.5;    % Δ_min [m]
cte_thresh_m   = 1.5;   % reset integral threshold [m]
lookahead_k    = 1.5;   % adaptive lookahead multiplier
psi_filter_tau = 0.67;  % heading filter time constant [s]

%% ===== PID HEADING =====
% (dari path_planner_integrated.launch)
kp_psi     = 2.0;
ki_psi     = 0.05;
kd_psi     = 0.5;
RUDDER_MAX = deg2rad(40.0);  % rad

%% ===== MISI =====
u_des       = 1.5;   % kecepatan surge yang diinginkan [m/s]
arrive_dist = 2.0;   % jarak arrival [m]
goal_tol    = 4.0;   % jarak untuk speed ramp [m]
fade_in_sec = 2.0;   % waktu fade-in [s]

%% ===== SIMULASI =====
dt    = 0.02;    % time step [s] (50 Hz, sama dengan simulator ROS)
T_max = 300;     % durasi maksimum [s]
N_max = round(T_max / dt);

%% ===== PLANNING PARAMS =====
% (dari path_planner_integrated.launch)
safe_dist_m = 1.5;   % inflate obstacle untuk D* Lite
safe_plan_m = 0.3;   % inflate untuk clearance enforcement
inflate_m   = safe_plan_m + safe_dist_m;  % = 1.8 m total inflate
n_per_seg   = 25;    % sampel per segmen Bezier
eps_rdp     = 1.0;   % threshold RDP [m]

%% ===================================================================
%% 1. BANGUN OCCUPANCY GRID
%% ===================================================================
occ = false(nR, nC);  % false = bebas, true = obstacle

% min radius agar obstacle selalu masuk minimal 1 sel (sama dengan Python)
min_occ_m = 0.5 * sqrt(2) * cell_m;   % ≈ 1.414 m

for k_obs = 1:size(obstacles_static_m, 1)
    cx    = obstacles_static_m(k_obs, 1);
    cy    = obstacles_static_m(k_obs, 2);
    r_occ = max(obstacles_static_m(k_obs, 3) + inflate_m, min_occ_m);

    for row = 1:nR
        for col = 1:nC
            cx_cell = (col - 0.5) * cell_m;
            cy_cell = (row - 0.5) * cell_m;
            if (cx_cell - cx)^2 + (cy_cell - cy)^2 <= r_occ^2
                occ(row, col) = true;
            end
        end
    end
end

fprintf('Map: %d×%d grid (%d×%d m) | %d obstacle cells\n', ...
    nR, nC, mapSize_m(1), mapSize_m(2), sum(occ(:)));

%% ===================================================================
%% 2. D* LITE PATH PLANNING
%% ===================================================================
% Rencanakan tiap segmen: start→waypoint→goal
waypoints_list_m = {waypoint_m; goal_m};
path_global_m    = start_m;
seg_start        = start;
seg_start_m      = start_m;

for wi = 1:length(waypoints_list_m)
    seg_goal_m = waypoints_list_m{wi};
    seg_goal   = m2g(seg_goal_m);

    fprintf('Rencana WP%d: (%.1f,%.1f) → (%.1f,%.1f) m\n', ...
        wi, seg_start_m(1), seg_start_m(2), seg_goal_m(1), seg_goal_m(2));

    path_g = dstar_lite_plan(occ, nR, nC, seg_start, seg_goal);

    if isempty(path_g)
        warning('Path WP%d tidak ditemukan → fallback garis lurus', wi);
        path_g = [round(seg_start); round(seg_goal)];
    end

    seg_path_m = g2m(path_g);
    seg_path_m(1,:)   = seg_start_m;   % paksa endpoint tepat di meter
    seg_path_m(end,:) = seg_goal_m;

    if size(path_global_m, 1) == 1
        path_global_m = seg_path_m;
    else
        path_global_m = [path_global_m; seg_path_m(2:end,:)];
    end

    seg_start   = seg_goal;
    seg_start_m = seg_goal_m;
end

fprintf('D* Lite: %d titik raw\n', size(path_global_m, 1));

%% ===================================================================
%% 3. G2-CBS C² PATH SMOOTHING + CLEARANCE ENFORCEMENT
%% ===================================================================
path_smooth_m = smooth_path_g2cbs(path_global_m, n_per_seg, eps_rdp);

[path_smooth_m, clr_info] = enforce_clearance( ...
    path_smooth_m, obstacles_static_m, safe_plan_m);

% Paksa endpoint tetap tepat
path_smooth_m(1,:)   = start_m;
path_smooth_m(end,:) = goal_m;

fprintf('Smoothing: %d titik | min_clearance=%.3f m | iter=%d\n', ...
    size(path_smooth_m, 1), clr_info.min_clearance, clr_info.iterations);

%% ===================================================================
%% 4. OBSTACLE TAMBAHAN — muncul setelah kapal melewati waypoint pertama
%% ===================================================================
% Format: [cx_m, cy_m, radius_m]
extraObs_gt_m = [35.2 19.3 0.25];   % obstacle muncul setelah waypoint 1

% Radius waypoint-pass threshold: dianggap melewati WP saat jarak < ini [m]
wp_pass_dist    = arrive_dist + 1.0;   % 3 m
extra_obs_done  = false;               % flag agar hanya replan sekali
obstacles_all_m = obstacles_static_m;  % kumulatif (untuk visualisasi & clearance)

%% ===================================================================
%% 5. KONDISI AWAL
%% ===================================================================
x_g = start(1);   % posisi X dalam grid (col)
y_g = start(2);   % posisi Y dalam grid (row)
psi = 0.0;        % heading [rad]
phi = 0.0;        % roll [rad]
nu  = [0; 0; 0; 0];   % [u_g, v_g, r, p] (u_g/v_g dalam grid/s)

% Banking state
phi_des_prev = 0.0;
eInt_phi     = 0.0;

% ILOS state
sigma      = 0.0;   % cross-track integral σ
psi_d_filt = 0.0;   % filtered desired heading (init saat pertama dipanggil)
ilos_init  = false; % flag inisialisasi filter

% PID heading state
pid_integral  = 0.0;
pid_prev_err  = 0.0;
pid_filt_rate = 0.0;

% Logging — 17 kolom:
% [t, x, y, psi, phi, u, v, r, cte, psi_d, Fx, Fy, u_cmd, r_cmd, phi_des, x_ref, y_ref]
LOG        = zeros(N_max, 17);
LOG_replan = false(N_max, 1);
k_end      = N_max;

% Simpan occ awal dan smooth path awal (untuk Figure 1 — sebelum obstacle tambahan)
occ_init         = occ;
path_smooth_init = path_smooth_m;

%% ===================================================================
%% 6. MAIN SIMULATION LOOP
%% ===================================================================
for k = 1:N_max
    t = (k - 1) * dt;

    % Posisi dalam meter (sama dengan _grid_to_meter() Python)
    x_m = (x_g - 0.5) * cell_m;
    y_m = (y_g - 0.5) * cell_m;
    pos = [x_m, y_m];

    d_goal = norm(pos - goal_m);

    % Cek arrival
    if d_goal < arrive_dist
        fprintf('Goal reached at t=%.1f s | pos=(%.1f, %.1f) m\n', t, x_m, y_m);
        k_end = k;
        break;
    end

    % ---- Cek obstacle tambahan: muncul setelah waypoint pertama ----
    if ~extra_obs_done && norm(pos - waypoint_m) < wp_pass_dist
        fprintf('[Replan] t=%.1f s | Kapal melewati WP1 → obstacle baru (%.1f,%.1f) m muncul\n', ...
            t, extraObs_gt_m(1), extraObs_gt_m(2));

        % Update occupancy grid dengan obstacle baru
        r_occ_new = max(extraObs_gt_m(3) + inflate_m, min_occ_m);
        for row = 1:nR
            for col = 1:nC
                cx_c = (col-0.5)*cell_m; cy_c = (row-0.5)*cell_m;
                if (cx_c-extraObs_gt_m(1))^2 + (cy_c-extraObs_gt_m(2))^2 <= r_occ_new^2
                    occ(row,col) = true;
                end
            end
        end

        % Replan: dari posisi saat ini langsung ke goal (tanpa lewat waypoint)
        seg_start_re = m2g(pos);
        seg_goal_re  = m2g(goal_m);
        path_g_re    = dstar_lite_plan(occ, nR, nC, seg_start_re, seg_goal_re);

        if ~isempty(path_g_re)
            path_global_m_re     = g2m(path_g_re);
            path_global_m_re(1,:)   = pos;
            path_global_m_re(end,:) = goal_m;

            % Re-smooth
            obstacles_all_m  = [obstacles_static_m; extraObs_gt_m];
            path_smooth_re   = smooth_path_g2cbs(path_global_m_re, n_per_seg, eps_rdp);
            [path_smooth_re, ~] = enforce_clearance(path_smooth_re, obstacles_all_m, safe_plan_m);
            path_smooth_re(1,:)   = pos;
            path_smooth_re(end,:) = goal_m;
            path_smooth_m = path_smooth_re;

            % Reset ILOS setelah replan (sama dengan ilos.reset() Python)
            sigma      = 0.0;
            psi_d_filt = psi;
            ilos_init  = true;

            LOG_replan(k) = true;
            fprintf('[Replan] Selesai: %d titik smooth baru\n', size(path_smooth_m,1));
        else
            warning('[Replan] Path tidak ditemukan → tetap pakai path lama');
        end
        extra_obs_done = true;
    end

    % Fade-in: 2 detik pertama (sama dengan path_planner_dstar_apf.py)
    fade_in = min(1.0, t / fade_in_sec);

    % ---- ILOS Guidance ----
    if d_goal < lookahead_m
        % Terminal guidance: heading langsung ke goal
        psi_des   = wrap_angle(atan2(goal_m(2)-pos(2), goal_m(1)-pos(1)));
        cte_val   = 0.0;
        if ~ilos_init
            psi_d_filt = psi_des;
            ilos_init  = true;
        end
    else
        if ~ilos_init
            psi_d_filt = psi;
            ilos_init  = true;
        end
        [psi_des, sigma, psi_d_filt, cte_val] = ilos_compute( ...
            pos, path_smooth_m, sigma, psi_d_filt, dt, ...
            k_integral, lookahead_m, cte_thresh_m, lookahead_k, psi_filter_tau);
    end

    % ---- PID Heading (derivative on measurement) ----
    e_psi   = wrap_angle(psi_des - psi);
    e_psi_c = max(-deg2rad(20), min(deg2rad(20), e_psi));  % clamp ±20°

    r_now = nu(3);   % yaw rate dari state (pengganti sensor odom)
    [r_pid, pid_integral, pid_prev_err, pid_filt_rate] = pid_update_with_rate( ...
        e_psi_c, r_now, pid_integral, pid_prev_err, pid_filt_rate, ...
        kp_psi, ki_psi, kd_psi, dt);

    r_cmd = max(-1.0, min(1.0, r_pid)) * fade_in * RUDDER_MAX;

    % ---- Speed Command (heading-first, min 0.15 untuk efektivitas rudder) ----
    cos_factor = max(0.15, cos(e_psi));
    u_cmd      = u_des * min(1.0, d_goal / goal_tol) * fade_in * cos_factor;  % m/s
    u_cmd_g    = u_cmd / cell_m;   % konversi ke grid/s

    % ---- Forces (port dari _compute_forces() Python) ----
    Fx = Ku * (u_cmd_g - nu(1));
    Fx = max(-lim_TX, min(lim_TX, Fx));

    Fy = Kr * (r_cmd - r_now) - Kd_r * r_now;
    Fy = max(-lim_TY, min(lim_TY, Fy));

    % ---- Banking Control (port dari _compute_banking() Python) ----
    U_phys  = max(0.3, norm(nu(1:2)) * cell_m);   % kecepatan fisik [m/s]
    a_y_cmd = U_phys * r_cmd;
    phi_cmd = 5.0 * atan(a_y_cmd / g_accel) + 0.3 * phi;
    phi_cmd = max(-phi_max, min(phi_max, phi_cmd));

    alpha_phi    = dt / (tau_phi + dt);
    phi_des      = phi_des_prev + alpha_phi * (phi_cmd - phi_des_prev);
    phi_des_prev = phi_des;

    e_phi    = phi_des - phi;
    eInt_phi = eInt_phi + e_phi * dt;

    Tk = Kphi_p * e_phi + Kphi_i * eInt_phi - Kphi_d * nu(4);
    Tk = max(-lim_TK, min(lim_TK, Tk));  %#ok<NASGU>

    % ---- Dynamics (port dari _dynamics() Python) ----
    u = nu(1); v = nu(2); r = nu(3); p = nu(4);

    udot = A1*v*r + A2*u + A3*abs(u)*u + A4*(abs(u)^2)*u + A18*Fx;

    vdot = -(1/A1)*u*r + A5*v + A6*abs(v)*v + A7*(abs(v)^2)*v ...
           + A8*abs(r)*v + A9*abs(v)*r;

    rdot = -A10*v*u + A11*u*v + A12*r + A13*abs(r)*r + A14*(abs(r)^2)*r ...
           + A15*abs(r)*u + A16*abs(u)*r + A17*abs(u)*u ...
           + A20*abs(r)*u + A21*abs(u)*r + A22*abs(u)*u + A19*Fy;

    pdot = -KpLin*p - KpAbs*abs(p)*p - KpCub*(abs(p)^2)*p ...
           - Kphi*sin(phi) + Kfy*Fy + Kv_phi*v + Kr_phi*r + Kbias;

    % Kinematika: xdot_g, ydot_g dalam grid/s
    xdot_g = u*cos(psi) - v*sin(psi);
    ydot_g = u*sin(psi) + v*cos(psi);

    % ---- Euler Integration ----
    nu  = nu + dt * [udot; vdot; rdot; pdot];
    x_g = x_g + dt * xdot_g;
    y_g = y_g + dt * ydot_g;
    psi = psi + dt * nu(3);
    phi = phi + dt * nu(4);

    % Clamp state (sama dengan Python)
    nu(1) = max(-u_max_g, min(u_max_g, nu(1)));
    nu(2) = max(-v_max_g, min(v_max_g, nu(2)));
    nu(3) = max(-r_max,   min(r_max,   nu(3)));
    nu(4) = max(-p_max,   min(p_max,   nu(4)));
    psi   = wrap_angle(psi);

    % Cek NaN/Inf
    if any(~isfinite([x_g, y_g, psi, phi, nu']))
        warning('State non-finite di t=%.2f s! Reset velocity.', t);
        nu(:) = 0;
        k_end = k;
        break;
    end

    % ---- Nearest point on smooth path (untuk referensi x, y) ----
    dist_to_path = vecnorm([path_smooth_m(:,1)-x_m, path_smooth_m(:,2)-y_m], 2, 2);
    [~, idx_ref] = min(dist_to_path);
    x_ref = path_smooth_m(idx_ref, 1);
    y_ref = path_smooth_m(idx_ref, 2);

    % ---- Logging ----
    LOG(k,:) = [t, x_m, y_m, psi, phi, ...
                nu(1)*cell_m, nu(2)*cell_m, nu(3), ...   % u, v, r
                cte_val, psi_des, Fx, Fy, u_cmd, r_cmd, ...
                phi_des, x_ref, y_ref];

    if mod(k, 500) == 0
        fprintf('t=%5.1f s | (%.1f, %.1f) m | u=%.2f m/s | ψ=%5.1f° | d_goal=%.1f m\n', ...
            t, x_m, y_m, nu(1)*cell_m, rad2deg(psi), d_goal);
    end
end

% Potong log sesuai step terakhir
LOG        = LOG(1:k_end, :);
LOG_replan = LOG_replan(1:k_end);
t_log    = LOG(:,1);
x_log    = LOG(:,2);  y_log    = LOG(:,3);
psi_log  = LOG(:,4);  phi_log  = LOG(:,5);
u_log    = LOG(:,6);  v_log    = LOG(:,7);   r_log    = LOG(:,8);
cte_log  = LOG(:,9);  psid_log = LOG(:,10);
Fx_log   = LOG(:,11); Fy_log   = LOG(:,12);
ucmd_log = LOG(:,13); rcmd_log = LOG(:,14);
phid_log = LOG(:,15); xref_log = LOG(:,16);  yref_log = LOG(:,17);

% Indeks replan pertama
replan_idx = find(LOG_replan, 1);

% Jarak ke setiap obstacle statis sepanjang waktu
dist_to_obs = zeros(length(t_log), size(obstacles_static_m,1));
for k_obs = 1:size(obstacles_static_m,1)
    dist_to_obs(:,k_obs) = sqrt((x_log - obstacles_static_m(k_obs,1)).^2 + ...
                                 (y_log - obstacles_static_m(k_obs,2)).^2);
end
safe_radius = obstacles_static_m(:,3)' + safe_dist_m;   % [1 x 6] batas aman tiap obs

%% ===================================================================
%% 7. VISUALISASI — 5 Figure Terpisah
%% ===================================================================
th_circ = linspace(0, 2*pi, 64)';

%% --- Figure 1: Global Path D* Lite + G2-CBS Smoothing (peta awal) ---
figure('Name','Fig 1: Global Path & Smoothing','Position',[50 50 900 550]);
hold on; box on;
imagesc([0 nC*cell_m], [0 nR*cell_m], double(occ_init));
colormap(gca, [1 1 1; 0.75 0.75 0.75]);
caxis([0 1]);
set(gca, 'YDir', 'normal');

for k_obs = 1:size(obstacles_static_m,1)
    cx_o = obstacles_static_m(k_obs,1);
    cy_o = obstacles_static_m(k_obs,2);
    r_p  = obstacles_static_m(k_obs,3);
    r_inf = r_p + inflate_m;
    fill(cx_o + r_p*cos(th_circ), cy_o + r_p*sin(th_circ), ...
        [0.9 0.2 0.2], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    plot(cx_o + r_inf*cos(th_circ), cy_o + r_inf*sin(th_circ), ...
        'r--', 'LineWidth', 0.8);
    text(cx_o, cy_o, sprintf('O%d',k_obs), 'HorizontalAlignment','center', ...
        'FontSize', 7, 'Color', [0.7 0 0], 'FontWeight', 'bold');
end

plot(path_global_m(:,1), path_global_m(:,2), 'c.-', ...
    'LineWidth', 1.0, 'MarkerSize', 6, 'DisplayName', 'D* Lite (raw)');
plot(path_smooth_init(:,1), path_smooth_init(:,2), 'g-', ...
    'LineWidth', 2.0, 'DisplayName', 'G2-CBS C² smooth');
plot(start_m(1),    start_m(2),    'go', 'MarkerSize', 12, ...
    'MarkerFaceColor','g', 'DisplayName','Start');
plot(waypoint_m(1), waypoint_m(2), 'bs', 'MarkerSize', 10, ...
    'MarkerFaceColor','b', 'DisplayName','Waypoint 1');
plot(goal_m(1),     goal_m(2),     'r^', 'MarkerSize', 12, ...
    'MarkerFaceColor','r', 'DisplayName','Goal');

xlabel('X [m]'); ylabel('Y [m]');
title('Figure 1: Global Path D* Lite + G2-CBS C^2 Smoothing (Peta Awal)');
legend('Location','northwest','FontSize',8);
axis([0 mapSize_m(2) 0 mapSize_m(1)]);
grid on;

%% --- Figure 2: Trajektori USV Actual ---
figure('Name','Fig 2: Trajektori USV','Position',[100 80 900 550]);
hold on; box on;
imagesc([0 nC*cell_m], [0 nR*cell_m], double(occ));
colormap(gca, [1 1 1; 0.75 0.75 0.75]);
caxis([0 1]);
set(gca, 'YDir', 'normal');

for k_obs = 1:size(obstacles_static_m,1)
    cx_o = obstacles_static_m(k_obs,1);
    cy_o = obstacles_static_m(k_obs,2);
    r_p  = obstacles_static_m(k_obs,3);
    fill(cx_o + r_p*cos(th_circ), cy_o + r_p*sin(th_circ), ...
        [0.9 0.2 0.2], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    text(cx_o, cy_o, sprintf('O%d',k_obs), 'HorizontalAlignment','center', ...
        'FontSize', 7, 'Color', [0.7 0 0], 'FontWeight', 'bold');
end

% Obstacle tambahan
cx_e = extraObs_gt_m(1); cy_e = extraObs_gt_m(2); r_e = extraObs_gt_m(3);
fill(cx_e + r_e*cos(th_circ), cy_e + r_e*sin(th_circ), ...
    [0.9 0.5 0], 'FaceAlpha', 0.7, 'EdgeColor', 'none');
text(cx_e, cy_e+0.9, 'Extra', 'HorizontalAlignment','center', ...
    'FontSize', 7, 'Color', [0.7 0.3 0], 'FontWeight', 'bold');

% Trajektori: biru sebelum replan, magenta setelah
if ~isempty(replan_idx) && replan_idx > 1
    plot(x_log(1:replan_idx), y_log(1:replan_idx), 'b-', ...
        'LineWidth', 2.0, 'DisplayName', 'Sebelum replan');
    plot(x_log(replan_idx:end), y_log(replan_idx:end), 'm-', ...
        'LineWidth', 2.0, 'DisplayName', 'Setelah replan');
    plot(x_log(replan_idx), y_log(replan_idx), 'kp', ...
        'MarkerSize', 14, 'MarkerFaceColor','y', 'DisplayName','Titik Replan');
else
    plot(x_log, y_log, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Trajektori USV');
end

% Heading arrows setiap 5 detik
step_hdg = max(1, round(5.0/dt));
idx_hdg  = 1:step_hdg:length(t_log);
for ii = idx_hdg
    quiver(x_log(ii), y_log(ii), ...
           1.5*cos(psi_log(ii)), 1.5*sin(psi_log(ii)), ...
           0, 'Color',[0.3 0.3 0.8], 'MaxHeadSize', 2, 'LineWidth', 0.6);
end

plot(start_m(1),    start_m(2),    'go', 'MarkerSize', 12, ...
    'MarkerFaceColor','g', 'DisplayName','Start');
plot(waypoint_m(1), waypoint_m(2), 'bs', 'MarkerSize', 10, ...
    'MarkerFaceColor','b', 'DisplayName','Waypoint 1');
plot(goal_m(1),     goal_m(2),     'r^', 'MarkerSize', 12, ...
    'MarkerFaceColor','r', 'DisplayName','Goal');

xlabel('X [m]'); ylabel('Y [m]');
title('Figure 2: Trajektori USV (Biru = Sebelum Replan, Magenta = Setelah Replan)');
legend('Location','northwest','FontSize',8);
axis([0 mapSize_m(2) 0 mapSize_m(1)]);
grid on;

%% --- Figure 3: Jarak USV ke Setiap Obstacle Statis ---
figure('Name','Fig 3: Jarak ke Obstacle','Position',[150 110 900 480]);
hold on; box on;
colors_obs = lines(size(obstacles_static_m,1));
for k_obs = 1:size(obstacles_static_m,1)
    plot(t_log, dist_to_obs(:,k_obs), ...
        'Color', colors_obs(k_obs,:), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('O%d (r_{safe}=%.2f m)', k_obs, safe_radius(k_obs)));
end
yline(min(safe_radius), 'k--', 'LineWidth', 1.2, ...
    'Label', 'Batas Aman Min', 'LabelHorizontalAlignment', 'right');
if ~isempty(replan_idx)
    xline(t_log(replan_idx), 'r--', 'LineWidth', 1.2, ...
        'Label', sprintf('Replan t=%.1fs', t_log(replan_idx)), ...
        'LabelHorizontalAlignment', 'right');
end
xlabel('t [s]'); ylabel('Jarak [m]');
title('Figure 3: Jarak USV ke Setiap Obstacle Statis');
legend('Location','northeast','FontSize',7);
grid on;

%% --- Figure 4: Speed dan Heading USV ---
figure('Name','Fig 4: Speed & Heading','Position',[200 140 900 500]);

subplot(2,1,1);
hold on; box on;
plot(t_log, u_log,    'b-', 'LineWidth', 1.5, 'DisplayName', 'u aktual');
plot(t_log, ucmd_log, 'k:', 'LineWidth', 1.0, 'DisplayName', 'u_{cmd}');
yline(u_des, 'r--', 'LineWidth', 1.2, ...
    'Label', sprintf('u_{des}=%.1f m/s',u_des), 'LabelHorizontalAlignment','right');
if ~isempty(replan_idx)
    xline(t_log(replan_idx), 'r-', 'LineWidth', 0.8, 'HandleVisibility','off');
end
xlabel('t [s]'); ylabel('u [m/s]');
title('Kecepatan Surge');
legend('Location','southeast','FontSize',8);
grid on;

subplot(2,1,2);
hold on; box on;
plot(t_log, rad2deg(psi_log),  'b-',  'LineWidth', 1.5, 'DisplayName', '\psi aktual');
plot(t_log, rad2deg(psid_log), 'r--', 'LineWidth', 1.5, 'DisplayName', '\psi_d desired');
if ~isempty(replan_idx)
    xline(t_log(replan_idx), 'r-', 'LineWidth', 0.8, 'HandleVisibility','off');
end
xlabel('t [s]'); ylabel('\psi [°]');
title('Heading (Yaw)');
legend('Location','northwest','FontSize',8);
grid on;

sgtitle('Figure 4: Kecepatan dan Heading USV', 'FontSize', 12);

%% --- Figure 5: Actual vs Desired (x, y, yaw, roll) ---
figure('Name','Fig 5: Actual vs Desired','Position',[250 170 1000 700]);

subplot(4,1,1);
hold on; box on;
plot(t_log, x_log,    'b-',  'LineWidth', 1.5, 'DisplayName', 'x aktual');
plot(t_log, xref_log, 'r--', 'LineWidth', 1.5, 'DisplayName', 'x referensi');
if ~isempty(replan_idx)
    xline(t_log(replan_idx), 'k--', 'LineWidth', 0.8, 'HandleVisibility','off');
end
ylabel('x [m]'); title('Posisi X');
legend('Location','northwest','FontSize',8); grid on;

subplot(4,1,2);
hold on; box on;
plot(t_log, y_log,    'b-',  'LineWidth', 1.5, 'DisplayName', 'y aktual');
plot(t_log, yref_log, 'r--', 'LineWidth', 1.5, 'DisplayName', 'y referensi');
if ~isempty(replan_idx)
    xline(t_log(replan_idx), 'k--', 'LineWidth', 0.8, 'HandleVisibility','off');
end
ylabel('y [m]'); title('Posisi Y');
legend('Location','northwest','FontSize',8); grid on;

subplot(4,1,3);
hold on; box on;
plot(t_log, rad2deg(psi_log),  'b-',  'LineWidth', 1.5, 'DisplayName', '\psi aktual');
plot(t_log, rad2deg(psid_log), 'r--', 'LineWidth', 1.5, 'DisplayName', '\psi_d desired');
if ~isempty(replan_idx)
    xline(t_log(replan_idx), 'k--', 'LineWidth', 0.8, 'HandleVisibility','off');
end
ylabel('\psi [°]'); title('Heading (Yaw)');
legend('Location','northwest','FontSize',8); grid on;

subplot(4,1,4);
hold on; box on;
plot(t_log, rad2deg(phi_log),  'b-',  'LineWidth', 1.5, 'DisplayName', '\phi aktual');
plot(t_log, rad2deg(phid_log), 'r--', 'LineWidth', 1.5, 'DisplayName', '\phi_d desired');
if ~isempty(replan_idx)
    xline(t_log(replan_idx), 'k--', 'LineWidth', 0.8, 'HandleVisibility','off');
end
xlabel('t [s]'); ylabel('\phi [°]'); title('Roll Angle');
legend('Location','northwest','FontSize',8); grid on;

sgtitle('Figure 5: Actual vs Desired — Posisi X, Y, Yaw, Roll', 'FontSize', 12);

%% ===================================================================
%% 8. RINGKASAN
%% ===================================================================
fprintf('\n========== Ringkasan Simulasi ==========\n');
fprintf('Durasi simulasi   : %.1f s\n', t_log(end));
fprintf('Mean surge        : %.3f m/s (target: %.1f m/s)\n', mean(u_log), u_des);
fprintf('Max surge         : %.3f m/s\n', max(u_log));
fprintf('Max |CTE|         : %.4f m\n', max(abs(cte_log)));
fprintf('Mean |CTE|        : %.4f m\n', mean(abs(cte_log)));
fprintf('RMS CTE           : %.4f m\n', sqrt(mean(cte_log.^2)));
fprintf('Max |roll|        : %.3f°\n', max(abs(rad2deg(phi_log))));
fprintf('Max |sway|        : %.4f m/s\n', max(abs(v_log)));
fprintf('Jarak ke goal     : %.3f m\n', norm([x_log(end) y_log(end)] - goal_m));
fprintf('=========================================\n');


%% ===================================================================
%%                      HELPER FUNCTIONS
%% ===================================================================

% ----------------------------------------------------------------
% D* Lite — sesuai Algorithm 3 (Koenig & Likhachev 2002)
%   Key(s)         : [min(g,rhs)+h(s_start,s)+km ; min(g,rhs)]
%   UpdateVertex   : update rhs, remove/re-insert ke OPEN
%   ComputePath    : loop sampai s_start konsisten
%   Path extraction: greedy descent via argmin cost+g
% ----------------------------------------------------------------
function path_g = dstar_lite_plan(occ, nR, nC, start_g, goal_g, w)
    if nargin < 6, w = 1.0; end   % bobot heuristik ε (w=1 → optimal)

    sx = max(1, min(nC, round(start_g(1))));
    sy = max(1, min(nR, round(start_g(2))));
    gx = max(1, min(nC, round(goal_g(1))));
    gy = max(1, min(nR, round(goal_g(2))));

    trav = ~occ;
    trav(sy, sx) = true;   % start selalu traversable
    if ~trav(gy, gx)
        [gx, gy] = nearest_free_cell(occ, gx, gy, nC, nR);
    end

    sid = sub2ind([nR nC], sy, sx);
    gid = sub2ind([nR nC], gy, gx);

    DX = [ 1 -1  0  0  1 -1  1 -1];
    DY = [ 0  0  1 -1  1 -1 -1  1];
    DC = [ 1  1  1  1  sqrt(2) sqrt(2) sqrt(2) sqrt(2)];

    N   = nR * nC;
    g   = 1e12 * ones(N, 1);
    rhs = 1e12 * ones(N, 1);
    km  = 0;
    Uk  = zeros(0, 2);   % priority queue: [K1 K2] per baris
    Ui  = zeros(0, 1);   % priority queue: node id

    % --- Initialize (Main, line 35-38) ---
    rhs(gid) = 0;
    [Uk, Ui] = dsl_push(Uk, Ui, gid, dsl_key(g, rhs, km, sid, gid, nR, nC, w));

    % --- ComputeShortestPath (line 13-29) ---
    while ~isempty(Ui)
        k_top   = dsl_topkey(Uk);
        k_start = dsl_key(g, rhs, km, sid, sid, nR, nC, w);
        if ~dsl_less(k_top, k_start) && rhs(sid) == g(sid), break; end

        [k_old, u, Uk, Ui] = dsl_pop(Uk, Ui);
        k_now = dsl_key(g, rhs, km, sid, u, nR, nC, w);

        if dsl_less(k_old, k_now)
            % Lazy deletion: key kadaluarsa → re-insert (line 17-18)
            [Uk, Ui] = dsl_push(Uk, Ui, u, k_now);

        elseif g(u) > rhs(u)
            % Overconsistent → g = rhs, update Pred(u) (line 19-22)
            g(u) = rhs(u);
            nb = dsl_succ(u, trav, nR, nC, DX, DY, DC);
            for ii = 1:size(nb, 1)
                [g, rhs, Uk, Ui] = dsl_upv( ...
                    nb(ii,1), gid, sid, g, rhs, km, Uk, Ui, trav, nR, nC, DX, DY, DC, w);
            end

        else
            % Underconsistent → g = ∞, update Pred(u)∪{u} (line 24-28)
            g(u) = 1e12;
            nb = dsl_succ(u, trav, nR, nC, DX, DY, DC);
            [g, rhs, Uk, Ui] = dsl_upv( ...
                u, gid, sid, g, rhs, km, Uk, Ui, trav, nR, nC, DX, DY, DC, w);
            for ii = 1:size(nb, 1)
                [g, rhs, Uk, Ui] = dsl_upv( ...
                    nb(ii,1), gid, sid, g, rhs, km, Uk, Ui, trav, nR, nC, DX, DY, DC, w);
            end
        end
    end

    % --- Path extraction: greedy descent (Main, line 40-41) ---
    if isinf(g(sid)), path_g = []; return; end
    cur    = sid;
    path_g = [sx, sy];
    for step = 1:N
        if cur == gid, break; end
        nb = dsl_succ(cur, trav, nR, nC, DX, DY, DC);
        if isempty(nb), path_g = []; return; end
        [~, idx] = min(nb(:,2) + g(nb(:,1)));
        cur = nb(idx, 1);
        [yy, xx] = ind2sub([nR nC], cur);
        path_g(end+1,:) = [xx, yy]; %#ok<AGROW>
    end
    if cur ~= gid, path_g = []; end
end

% UpdateVertex(u) — Algorithm 3, line 3-12
function [g, rhs, Uk, Ui] = dsl_upv(u, gid, sid, g, rhs, km, Uk, Ui, trav, nR, nC, DX, DY, DC, w)
    if u ~= gid
        nb = dsl_succ(u, trav, nR, nC, DX, DY, DC);
        if isempty(nb)
            rhs(u) = 1e12;
        else
            rhs(u) = min(nb(:,2) + g(nb(:,1)));
        end
    end
    keep = (Ui ~= u);          % OPEN.remove(u)
    Uk = Uk(keep, :);
    Ui = Ui(keep);
    if g(u) ~= rhs(u)          % OPEN.insert(u, Key(u))
        [Uk, Ui] = dsl_push(Uk, Ui, u, dsl_key(g, rhs, km, sid, u, nR, nC, w));
    end
end

% Key(s) — Algorithm 3, line 1-2  |  h skalakan dengan bobot ε=w
function K = dsl_key(g, rhs, km, sid, s, nR, nC, w)
    m = min(g(s), rhs(s));
    [sy, sx] = ind2sub([nR nC], sid);
    [vy, vx] = ind2sub([nR nC], s);
    K = [m + w * hypot(double(sx-vx), double(sy-vy)) + km,  m];
end

% Lexicographic key comparison: k1 < k2
function res = dsl_less(k1, k2)
    res = k1(1) < k2(1) - 1e-10 || ...
          (abs(k1(1) - k2(1)) < 1e-10 && k1(2) < k2(2) - 1e-10);
end

% Top key (min lexicographic) — OPEN.TopKey()
function k = dsl_topkey(Uk)
    [~, ord] = sortrows(Uk, [1 2]);
    k = Uk(ord(1), :);
end

% Pop min key — OPEN.Pop()
function [k, u, Uk, Ui] = dsl_pop(Uk, Ui)
    [~, ord] = sortrows(Uk, [1 2]);
    k        = Uk(ord(1), :);
    u        = Ui(ord(1));
    Uk(ord(1), :) = [];
    Ui(ord(1))    = [];
end

% Insert — OPEN.insert(s, key)
function [Uk, Ui] = dsl_push(Uk, Ui, s, k)
    Uk(end+1, :) = k;
    Ui(end+1)    = s;
end

% Succ(s) — 8-connected neighbors: [node_id, edge_cost]
function nb = dsl_succ(s, trav, nR, nC, DX, DY, DC)
    [y, x] = ind2sub([nR nC], s);
    nb = zeros(0, 2);
    for i = 1:8
        nx = x + DX(i);  ny = y + DY(i);
        if nx>=1 && nx<=nC && ny>=1 && ny<=nR && trav(ny, nx)
            nb(end+1,:) = [sub2ind([nR nC], ny, nx), DC(i)]; %#ok<AGROW>
        end
    end
end

function [gx, gy] = nearest_free_cell(occ, gx0, gy0, nC, nR)
    for rr = 1:max(nC, nR)
        for dx = -rr:rr
            for dy = -rr:rr
                nx = gx0+dx; ny = gy0+dy;
                if nx>=1 && nx<=nC && ny>=1 && ny<=nR && ~occ(ny,nx)
                    gx = nx; gy = ny; return;
                end
            end
        end
    end
    gx = gx0; gy = gy0;
end

% ----------------------------------------------------------------
% G2-CBS C² Path Smoothing — port dari PathSmoother.smooth()
% ----------------------------------------------------------------
function Ps = smooth_path_g2cbs(path_xy, n_per_seg, eps_rdp)
    P = rm_dups(path_xy);
    if size(P,1) <= 2; Ps = P; return; end

    if eps_rdp > 0 && size(P,1) > 3
        P = rdp_simplify(P, eps_rdp);
        P = rm_dups(P);
        if size(P,1) <= 2; Ps = P; return; end
    end

    N = size(P,1);

    % Arc-length parameterization
    t_arc = [0; cumsum(vecnorm(diff(P), 2, 2))];
    if t_arc(end) < 1e-9; Ps = P(1,:); return; end

    h = diff(t_arc);

    % Natural spline second derivatives (Thomas algorithm)
    Mx = spline_second_deriv(t_arc, P(:,1));
    My = spline_second_deriv(t_arc, P(:,2));

    % First derivatives at knots (G2-CBS matching — identik dengan Python)
    sx = diff(P(:,1)) ./ h;
    sy = diff(P(:,2)) ./ h;
    mx = zeros(N,1); my = zeros(N,1);

    mx(1) = sx(1) - h(1)*(2*Mx(1) + Mx(2))/6;
    my(1) = sy(1) - h(1)*(2*My(1) + My(2))/6;
    for ii = 2:N-1
        mx(ii) = 0.5*((sx(ii-1) + h(ii-1)*(Mx(ii-1)+2*Mx(ii))/6) + ...
                      (sx(ii)   - h(ii)  *(2*Mx(ii)+Mx(ii+1))/6));
        my(ii) = 0.5*((sy(ii-1) + h(ii-1)*(My(ii-1)+2*My(ii))/6) + ...
                      (sy(ii)   - h(ii)  *(2*My(ii)+My(ii+1))/6));
    end
    mx(N) = sx(N-1) + h(N-1)*(Mx(N-1)+2*Mx(N))/6;
    my(N) = sy(N-1) + h(N-1)*(My(N-1)+2*My(N))/6;

    % Cubic Bezier segments
    Ps = P(1,:);
    for ii = 1:N-1
        hi = h(ii);
        b0 = P(ii,:);   b3 = P(ii+1,:);
        b1 = b0 + (hi/3)*[mx(ii),   my(ii)];
        b2 = b3 - (hi/3)*[mx(ii+1), my(ii+1)];

        tau = linspace(0,1,n_per_seg)';
        B = (1-tau).^3.*b0 + 3*(1-tau).^2.*tau.*b1 + ...
            3*(1-tau).*tau.^2.*b2 + tau.^3.*b3;

        Ps = [Ps; B(2:end,:)]; %#ok<AGROW>
    end
    Ps = rm_dups(Ps);
end

% Thomas algorithm untuk natural cubic spline second derivatives
% Port dari spline_M() — path_smoother.py
function M = spline_second_deriv(t, y)
    N = length(y);
    if N <= 2; M = zeros(N,1); return; end
    h = diff(t);

    a = zeros(N,1); b = ones(N,1);
    c = zeros(N,1); d = zeros(N,1);
    for ii = 2:N-1
        a(ii) = h(ii-1);
        b(ii) = 2*(h(ii-1)+h(ii));
        c(ii) = h(ii);
        d(ii) = 6*((y(ii+1)-y(ii))/h(ii) - (y(ii)-y(ii-1))/h(ii-1));
    end
    for ii = 2:N
        if abs(b(ii-1)) < 1e-14, continue; end
        mv = a(ii)/b(ii-1);
        b(ii) = b(ii) - mv*c(ii-1);
        d(ii) = d(ii) - mv*d(ii-1);
    end
    M = zeros(N,1);
    if abs(b(N)) > 1e-14; M(N) = d(N)/b(N); end
    for ii = N-1:-1:1
        if abs(b(ii)) > 1e-14
            M(ii) = (d(ii) - c(ii)*M(ii+1)) / b(ii);
        end
    end
end

% ----------------------------------------------------------------
% Clearance Enforcement — port dari PathSmoother.enforce_clearance()
% ----------------------------------------------------------------
function [P, info] = enforce_clearance(path_xy, obstacles_m, safe_dist_m, ...
                                        max_iter, gain, max_step, lam, ds)
    if nargin < 4; max_iter  = 80;   end
    if nargin < 5; gain      = 0.6;  end
    if nargin < 6; max_step  = 0.2;  end
    if nargin < 7; lam       = 0.15; end
    if nargin < 8; ds        = 0.5;  end

    P   = resample_path(path_xy, ds);
    N   = size(P,1);
    obs = obstacles_m;   % [cx cy r]

    if N <= 2 || isempty(obs)
        info.min_clearance = inf;
        info.iterations    = 0;
        return;
    end

    it = 0;
    for it = 1:max_iter
        dP  = zeros(N,2);
        vio = false;

        for ii = 2:N-1
            push = [0 0];
            for ko = 1:size(obs,1)
                v_ob  = P(ii,:) - obs(ko,1:2);
                d_ob  = norm(v_ob);
                R     = obs(ko,3) + safe_dist_m;
                if d_ob < R && d_ob > 1e-9
                    push = push + gain*(R-d_ob)*(v_ob/d_ob);
                    vio  = true;
                end
            end
            sm    = lam*((P(ii-1,:)+P(ii+1,:))/2 - P(ii,:));
            delta = push + sm;
            nm    = norm(delta);
            if nm > max_step; delta = delta*(max_step/nm); end
            dP(ii,:) = delta;
        end
        P(2:end-1,:) = P(2:end-1,:) + dP(2:end-1,:);
        if ~vio; break; end
    end

    P  = rm_dups(P);
    mc = inf;
    for ko = 1:size(obs,1)
        d  = vecnorm(P - obs(ko,1:2), 2, 2) - (obs(ko,3) + safe_dist_m);
        mc = min(mc, min(d));
    end
    info.min_clearance = double(mc);
    info.iterations    = it;
end

% ----------------------------------------------------------------
% ILOS Guidance — port dari ILOSFollower.compute_desired_heading()
% ----------------------------------------------------------------
function [psi_d, sigma, psi_d_filt, e_y] = ilos_compute( ...
    pos, path, sigma, psi_d_filt, dt, ...
    k_int, lookahead, cte_thresh, la_k, psi_tau)

    e_y = 0;
    if size(path,1) < 2
        psi_d = psi_d_filt;
        return;
    end

    seg_idx = ilos_find_segment(pos, path);

    p1 = path(seg_idx,:);
    p2 = path(min(seg_idx+1, size(path,1)),:);
    seg_vec = p2 - p1;
    seg_len = norm(seg_vec);

    if seg_len < 1e-6
        psi_d = psi_d_filt;
        return;
    end

    % Path tangent heading α_k
    alpha_k = atan2(seg_vec(2), seg_vec(1));

    % Cross-track error e_y (positive = kiri dari path)
    seg_norm = [-seg_vec(2), seg_vec(1)] / seg_len;
    e_y = dot(pos - p1, seg_norm);

    % Adaptive lookahead: Δ_eff
    delta_eff = max(lookahead, abs(e_y) * la_k);

    % Reset integral jika CTE > threshold
    if abs(e_y) > cte_thresh
        sigma = 0.0;
    end

    % Integral update — rigorous ILOS form (Fossen & Pettersen 2014)
    nu_val = e_y + k_int * sigma;
    denom  = delta_eff^2 + nu_val^2;
    sigma  = sigma + dt * delta_eff * e_y / denom;

    % ILOS heading command (raw)
    nu_val    = e_y + k_int * sigma;
    psi_d_raw = alpha_k + atan2(-nu_val, delta_eff);

    % Low-pass filter (α = dt/τ — perilaku filter sama di semua dt)
    alpha_f    = min(1.0, dt / psi_tau);
    psi_d_filt = psi_d_filt + alpha_f * wrap_angle(psi_d_raw - psi_d_filt);

    psi_d = wrap_angle(psi_d_filt);
end

function idx = ilos_find_segment(pos, path)
    % Cari segmen path terdekat ke posisi saat ini
    min_d = inf; idx = 1;
    for ii = 1:size(path,1)-1
        p1  = path(ii,:);
        p2  = path(ii+1,:);
        seg = p2 - p1;
        L2  = max(1e-12, dot(seg,seg));
        tt  = max(0, min(1, dot(pos-p1, seg)/L2));
        d   = norm(pos - (p1 + tt*seg));
        if d < min_d; min_d = d; idx = ii; end
    end
end

% ----------------------------------------------------------------
% PID dengan derivative on measurement — port dari pid_controller.py
% ----------------------------------------------------------------
function [out, integral, prev_err, filt_rate] = pid_update_with_rate( ...
    error, measured_rate, integral, prev_err, filt_rate, kp, ki, kd, dt)

    p_term = kp * error;

    % Anti-windup zero-crossing: kurangi 50% saat error berganti tanda
    if error * prev_err < 0
        integral = integral * 0.5;
    end

    integral = integral + error * dt;
    integral = max(-1.0, min(1.0, integral));
    i_term   = ki * integral;

    % Derivative on measurement (α=0.8, filter ringan)
    filt_rate = filt_rate + 0.8*(measured_rate - filt_rate);
    d_term    = -kd * filt_rate;

    prev_err = error;
    out = p_term + i_term + d_term;
end

% ----------------------------------------------------------------
% Utilitas
% ----------------------------------------------------------------
function P = rm_dups(P, tol)
    if nargin < 2; tol = 1e-8; end
    if isempty(P); return; end
    keep = [true; vecnorm(diff(P), 2, 2) > tol];
    P    = P(keep,:);
end

function P = resample_path(P, ds)
    if size(P,1) < 2 || ds <= 0; return; end
    segs = vecnorm(diff(P), 2, 2);
    s    = [0; cumsum(segs)];
    ss   = (0:ds:s(end))';
    if isempty(ss) || ss(end) < s(end); ss(end+1) = s(end); end
    P = [interp1(s, P(:,1), ss), interp1(s, P(:,2), ss)];
end

function P = rdp_simplify(P, eps)
    if size(P,1) <= 2; return; end
    A = P(1,:); B = P(end,:);
    AB = B - A;
    L2 = max(1e-12, dot(AB,AB));
    dists = zeros(size(P,1),1);
    for ii = 2:size(P,1)-1
        tt = max(0, min(1, dot(P(ii,:)-A, AB)/L2));
        dists(ii) = norm(P(ii,:) - (A + tt*AB));
    end
    [dm, ix] = max(dists);
    if dm > eps
        L = rdp_simplify(P(1:ix,:),   eps);
        R = rdp_simplify(P(ix:end,:), eps);
        P = [L(1:end-1,:); R];
    else
        P = [A; B];
    end
end

function a = wrap_angle(a)
    a = mod(a + pi, 2*pi) - pi;
end
