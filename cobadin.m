%% USV 4-DOF: D* Lite (weighted) + Kamera untuk Obstacle Statis Tambahan
% Global planner : D* Lite -> G2-CBS C^2
% Local control  : Clearance Guard + ILOS (tracking)
% Semua path & dinamika dihitung di "grid units".
% 1 grid cell = cell_m meter.

clear; clc; close all; rng(1);   % rng untuk hasil kamera yang reprodusibel

%% ===== SKALA GRID & PETA (METER) =====
cell_m = 2;                 % 1 grid cell = 2 meter

mapSize_m = [33 50];        % [Y X] tinggi & lebar peta dalam meter
nR = ceil(mapSize_m(1)/cell_m);
nC = ceil(mapSize_m(2)/cell_m);
mapSize = [nR nC];          % ukuran map dalam grid

% fungsi bantu konversi
m2g = @(p_m) [p_m(:,1)/cell_m + 0.5, p_m(:,2)/cell_m + 0.5];   % meter -> grid (bisa non-Integer)
g2m = @(pg) [(pg(:,1)-0.5)*cell_m, (pg(:,2)-0.5)*cell_m];      % grid -> meter (pusat sel)

%% ===== OBSTACLES, START, GOAL (METER, DESAIN) =====
% obstacles_*_m = [cx cy radius_m]

obstacles_static_m = [20.0 20.0 0.25;   % kiri atas
                      40.0 20.0 0.25;   % kanan atas
                      10.0 10.0 0.25;   % kiri bawah
                      30.0 10.0 0.25;   % kanan bawah
                      17.0 16.5 0.25;   % tambahan 1
                      41.0 16.0 0.25];  % tambahan 2


% obstacle statis tambahan (UNKNOWN pada planner, tapi ada di dunia nyata)
extraObs_gt_m = [35.2 19.3 0.25];      % ground truth (mis: di antara waypoint dan goal)

% --- DESAIN DALAM METER ---
start_m    = [ 1.0  8.0];
waypoint_m = [25.0 20.0];
goal_m     = [48.0 13.0];

% --- METER -> GRID (boleh non-integer) ---
start    = m2g(start_m);
waypoint = m2g(waypoint_m);
goal     = m2g(goal_m);

%% ===== PARAMETER USV 4 DOF =====
params = struct();

% Fisik kapal LSS-01
params.m  = 11.8;
params.mx = 1.5;
params.my = 6.0;
params.B = 0.82;

params.Iz = 2.20;
params.Jz = 0.0;

params.Ix = 1.00;
params.Jx = 0.0;

params.alphay = 0.20;
params.lx = 0.15;
params.ly = 0.10;

params.GMT = 0.04;
params.g   = 9.81;
params.W   = params.m*params.g;
params.hCG = 0.20;

params.Xu=-15; params.Xuu=-5;
params.Yv=-30; params.Yvv=-25;
params.Nr=-6;  params.Nrr=-12;
params.Kp=-4;  params.Kpp=-6;


%% limit aktuator
lims.TX=200;
lims.TY=60;
lims.TN=80;
lims.TK=60;

%% ===== BANKING (ROLL COUPLING) =====
bank.k_bank      = 4;
bank.phi_max_deg = 10;
bank.phi_max     = deg2rad(bank.phi_max_deg);

%% ===== SAFE DISTANCE (METER) =====
safeDist_m   = 0.6;                 % clearance runtime
safePlan_m   = 0.5*safeDist_m;      % inflasi saat planning
occInflate_m = safePlan_m + safeDist_m;  % radius occupancy (fisik + margin)
minOcc_m     = 0.5*sqrt(2)*cell_m;      % jamin obstacle selalu kena minimal 1 sel

%% ===== GRID NODE KOORDINAT DALAM METER (PUSAT SEL) =====
[xcGrid, ycGrid] = meshgrid(((1:nC)-0.5)*cell_m, ...
                            ((1:nR)-0.5)*cell_m);

%% ===== BANGUN MAP GRID UNTUK D* LITE (OCCUPANCY) =====
% PETA AWAL: HANYA 4 OBSTACLE STATIS YANG DIKETAHUI
obstacles_planning_m = obstacles_static_m;   % hanya yang diketahui

map = zeros(nR,nC);
for k = 1:size(obstacles_planning_m,1)
    cx   = obstacles_planning_m(k,1);
    cy   = obstacles_planning_m(k,2);
    rOcc = obstacles_planning_m(k,3) + occInflate_m;
    rOcc = max(rOcc, minOcc_m);
    map( (xcGrid-cx).^2 + (ycGrid-cy).^2 <= rOcc^2 ) = 1;
end

%% ===== METER -> GRID CONVERSION (untuk planner/guard) =====
obstacles_static = [ ...
    m2g(obstacles_static_m(:,1:2)), ...
    obstacles_static_m(:,3)/cell_m ];

obstacles_planning = [ ...
    m2g(obstacles_planning_m(:,1:2)), ...
    obstacles_planning_m(:,3)/cell_m ];

safeDist     = safeDist_m /cell_m;   % guard runtime (grid)
safeDistPlan = safePlan_m /cell_m;   % D* Lite (grid)
goal_tol     = 0.35;                  % toleransi goal di grid (~1.1 m)

%% ===== EXTRA OBSTACLE (GROUND TRUTH, GRID) =====

%% ===== DYNAMIC OBSTACLE (CAMERA DETECTED) =====
dynObs(1).pos = m2g([15 7]);
dynObs(1).vel = [0 0.35]/cell_m;   % naik
dynObs(1).rad = 0.25/cell_m;
dynObs(1).active = true;

dynObs(2).pos = m2g([37 32]);
%dynObs(2).pos = m2g([37 35]); %tanpa replan
dynObs(2).vel = [0 -0.35]/cell_m;  % turun
dynObs(2).rad = 0.25/cell_m;
dynObs(2).active = true;

extraGT.pos0    = m2g(extraObs_gt_m(1:2));
extraGT.pos     = extraGT.pos0;
extraGT.rad_obs = extraObs_gt_m(3)/cell_m;
extraGT.rad     = extraGT.rad_obs;   % bisa ditambah margin kalau mau
extraGT.active  = true;

extra_known     = false;    % apakah sudah "terdeteksi kamera"
replan_done     = false;    % apakah replan sudah dilakukan
newObs_m        = [];       % akan diisi saat terdeteksi
newObs          = [];       % versi grid


%% ===== D* Lite (weighted heuristic) =====
w = 1;

[pathSW, expandedSW] = dstarLite_grid(map, start,   waypoint, w);
[pathWG, expandedWG] = dstarLite_grid(map, waypoint, goal,    w);

if isempty(pathSW) || isempty(pathWG)
    error('Path S->W atau W->G tidak ditemukan!');
end

fprintf('Expanded nodes S->W (awal) = %d\n', expandedSW);
fprintf('Expanded nodes W->G (awal) = %d\n', expandedWG);

% pastikan endpoint mengikuti titik desain
pathSW(1,:)    = start;
pathSW(end,:)  = waypoint;
pathWG(1,:)    = waypoint;
pathWG(end,:)  = goal;

%% ===== Smooth global path awal (S->W->G) =====
samplesPerSeg = 25;
epsRDP        = 0.3;

% gabungkan path diskrit (hapus duplikasi di waypoint)
pathAll = [pathSW; pathWG(2:end,:)];

% smoothing sekali untuk path global
pAll = smooth_path_g2cbs_c2(pathAll, samplesPerSeg, epsRDP);

% clearance guard sekali juga, global
[pAll, infoAll] = enforce_safe_clearance(pAll, obstacles_planning, safeDistPlan, ...
    'maxIter',80,'gain',0.6,'maxStep',0.1,'lambda',0.05,'ds',0.25);

fprintf('Min clearance global (grid)  = %.3f\n',  infoAll.min_clearance);
fprintf('Min clearance global (meter) = %.3f m\n',infoAll.min_clearance*cell_m);

% arc-length path global awal
cumAll = cumulativeArc(pAll);

% simpan path awal untuk perbandingan setelah replan
pAll_init   = pAll;
cumAll_init = cumAll;

% indeks waypoint di path awal
[~, idx_wp_init] = min(vecnorm(pAll_init - waypoint, 2, 2));

%% ===== FIGURE 1: GLOBAL PLAN AWAL (DALAM METER) =====
figure(1); clf; hold on; axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]');
axis([0 mapSize_m(2) 0 mapSize_m(1)]);
title('Global Plan awal (meter): D* Lite + Smoothing');

theta = linspace(0,2*pi,60);

% obstacles awal dalam meter
for k=1:size(obstacles_static_m,1)
    cx_m = obstacles_static_m(k,1);
    cy_m = obstacles_static_m(k,2);
    r_phys  = obstacles_static_m(k,3);          % radius fisik
    r_guard = r_phys + safeDist_m;              % jarak aman

    fill(cx_m + r_phys*cos(theta), ...
         cy_m + r_phys*sin(theta), ...
         'r','FaceAlpha',0.3,'EdgeColor','none');
    plot(cx_m + r_guard*cos(theta), ...
         cy_m + r_guard*sin(theta), ...
         'r--','LineWidth',0.8);
end

% obstacle tambahan (ground truth) - digambar beda warna agar kelihatan,
% tetapi belum dipakai planner pada awalnya
cxE = extraObs_gt_m(1); cyE = extraObs_gt_m(2); rE = extraObs_gt_m(3);
fill(cxE + rE*cos(theta), cyE + rE*sin(theta), ...
     'g','FaceAlpha',0.25,'EdgeColor','none');
plot(cxE + (rE+safeDist_m)*cos(theta), ...
     cyE + (rE+safeDist_m)*sin(theta), ...
     'g--','LineWidth',0.8);

% Titik Start / Waypoint / Goal
plot(start_m(1),   start_m(2),   'yo','MarkerFaceColor','y');
plot(waypoint_m(1),waypoint_m(2),'mo','MarkerFaceColor','m');
plot(goal_m(1),    goal_m(2),    'ro','MarkerFaceColor','r');

% Path diskrit dan smooth (meter)
pSWm   = g2m(pathSW);
pWGm   = g2m(pathWG);
pAll_m = g2m(pAll);

pSWg    = plot(pSWm(:,1),pSWm(:,2),'c--','LineWidth',1.2);
pWGg    = plot(pWGm(:,1),pWGm(:,2),'b--','LineWidth',1.2);
pSmooth = plot(pAll_m(:,1),pAll_m(:,2),'k-','LineWidth',2.0);

legend([pSWg pWGg pSmooth], ...
    {'D*Lite S\rightarrowW','D*Lite W\rightarrowG','D* Lite + Smoothing'}, ...
    'Location','bestoutside');

%% ===== Tracking & Dynamics (grid units) =====
dt       = 0.05;              % s
v_ship_m = 1.2;               % m/s
v_ref    = v_ship_m / cell_m; % grid/s
Ld       = 6;                 % lookahead (grid)
R_ship_m = 0.9;               % radius aman kapal

%% ===== SAPF PARAM =====
sapf.zeta     = 1.2;
sapf.eta      = 1.8;
sapf.qstar    = 8.0/cell_m;
sapf.dvort    = 0.6;
sapf.alpha_th = deg2rad(35);
sapf.v_nom    = v_ref;

% Controller gains
Ku = 80;   % surge P
Kr = 80;   % yaw-rate P

% Sway PD (dipakai guard)
Kv   = 60;
Kdv  = 10;
Ymax = 60;
v_ref_sway = 0;

% Actuators
Tmax = 120;
Nmax = 80;
r_max_cmd = 0.55;

% Clearance guard params
avoid.buffer       = 1.4;
avoid.yaw_gain     = 1.2;
avoid.target_shift = 0.0;
avoid.minLdScale   = 0.45;
avoid.slowdownMin  = 0.10;

% ILOS gains
ilosParams.kp = 0.28;
ilosParams.ki = 0.000;
ilosParams.kd = 0.04;

%% ===== CAMERA PARAMS (grid) =====
camera.fov       = deg2rad(120);
camera.maxRange  = 5.0;      
camera.minRange  = 0.3;
camera.sigma_r   = 0.0;      % set ke 0 dulu biar mudah debug
camera.sigma_b   = 0.0;

%% ===== Waypoint handling (grid) =====
[~, idx_wp] = min(vecnorm(pAll - waypoint, 2, 2));
cumAll      = cumulativeArc(pAll);
s_wp        = cumAll(idx_wp);

wp_tol      = 0.35;          % radius waypoint (grid) ~ 1.4 m
reached_wp  = false;

%% ===== obstacles_rt untuk guard runtime =====
% hanya obstacle statis yang diketahui di awal
obstacles_rt = obstacles_static;

%% ===== FIGURE 2: ANIMASI (DALAM METER) =====
figure(2); clf; hold on; axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]');
axis([0 mapSize_m(2) 0 mapSize_m(1)]);
title('USV 4-DOF (ILOS + D*Lite + Kamera obstacle statis tambahan)');

theta = linspace(0,2*pi,60);

% obstacle statis (yang diketahui sejak awal)
for k=1:size(obstacles_static_m,1)
    cx_m = obstacles_static_m(k,1);
    cy_m = obstacles_static_m(k,2);
    r_phys  = obstacles_static_m(k,3);
    r_guard = r_phys + safeDist_m;

    fill(cx_m + r_phys*cos(theta), ...
         cy_m + r_phys*sin(theta), ...
         'r','FaceAlpha',0.3,'EdgeColor','none','HandleVisibility','off');
    plot(cx_m + r_guard*cos(theta), ...
         cy_m + r_guard*sin(theta), ...
         'r--','LineWidth',0.8,'HandleVisibility','off');
end

% obstacle ekstra (ground truth) - hanya digambar garis putus2 sampai terdeteksi
hExtraFill = fill(NaN,NaN,'m','FaceAlpha',0.3,'EdgeColor','none','HandleVisibility','off');
hExtraGuard= plot(NaN,NaN,'m--','LineWidth',0.8,'HandleVisibility','off');

hStart2 = plot(start_m(1),   start_m(2),   'yo','MarkerFaceColor','y','DisplayName','Start');
hWay2   = plot(waypoint_m(1),waypoint_m(2),'mo','MarkerFaceColor','m','DisplayName','Waypoint');
hGoal2  = plot(goal_m(1),    goal_m(2),    'ro','MarkerFaceColor','r','DisplayName','Goal');

% global path (meter) - awal (disembunyikan)
pAll_m = g2m(pAll);
hSmooth = plot(pAll_m(:,1),pAll_m(:,2),'k-','LineWidth',2.0, ...
               'Visible','off','HandleVisibility','off');

% traj (meter)
hTrace  = plot(NaN,NaN,'b-','LineWidth',1.6,'DisplayName','Traj');

for i=1:2

    hDyn(i) = fill(NaN,NaN,'c', ...
        'FaceAlpha',0.5,'EdgeColor','b');

    hDynGuard(i) = plot(NaN,NaN,'c--','LineWidth',1.2);

end

% kapal (body frame)
L_real    = 1.6;        % meter
L_model   = 1.8;
shipScale = L_real / L_model;
baseShip = shipScale * [ 1.00  0.00;
                        -0.80  0.45;
                        -0.40  0.00;
                        -0.80 -0.45 ];
tKapal = hgtransform('Parent', gca);
patch('XData', baseShip(:,1), 'YData', baseShip(:,2), ...
      'FaceColor',[0 0.7 0],'EdgeColor','k','LineWidth',0.8, ...
      'Parent', tKapal, 'HandleVisibility','off');

% Kamera FOV (di body-frame kapal) untuk visual
fovRange_m = camera.maxRange * cell_m;
angF     = linspace(-camera.fov/2, camera.fov/2, 30);
xF_body = [0, fovRange_m*cos(angF), 0];
yF_body = [0, fovRange_m*sin(angF), 0];
patch('XData', xF_body, 'YData', yF_body, ...
      'Parent', tKapal, ...
      'FaceColor',[1 1 0], ...
      'FaceAlpha',0.15, ...
      'EdgeColor',[1 0.8 0], ...
      'LineWidth',0.8, ...
      'HandleVisibility','off');

legend([hStart2 hWay2 hGoal2 hTrace],'Location','northeastoutside');

%% ===== State & logs =====
x   = start(1);
y   = start(2);
psi0 = atan2(pAll(min(2,size(pAll,1)),2)-start(2), ...
             pAll(min(2,size(pAll,1)),1)-start(1));
psi = psi0;
nu  = [0;0;0;0];  % [u v r p]
phi = 0;          % roll angle


pos0_m = g2m([x y]);
set(tKapal,'Matrix',makehgtform('translate',[pos0_m(1) pos0_m(2) 0],'zrotate',psi));

traj      = [];
dist_log  = [];
time_log  = [];
t         = 0;
cte_log   = [];
x_log     = [];
y_log     = [];
psi_log   = [];
phi_log   = [];
x_des_log = [];
y_des_log = [];
psi_des_log = [];
phi_des_log = [];
phi_des_prev = 0;     % initial roll desired
Ye_int  = 0;
Ye_prev = 0;

% simpan path setelah replan (untuk Figure 6)
pAll_replan = [];
didReplan   = false;
force_replan_track = false;
replan_idx_start   = 1;

eInt_phi = 0;
%% ===== MAIN LOOP =====
disp('=== Tracking Global Path (ILOS + D* Lite + Kamera obstacle statis), grid units ===');
while true
    %% UPDATE DYNAMIC OBSTACLE
for i = 1:2
    if dynObs(i).active
        dynObs(i).pos = dynObs(i).pos + dynObs(i).vel*dt;
    end
end
   
for i=1:2

    px = dynObs(i).pos(1)*cell_m - cell_m/2;
    py = dynObs(i).pos(2)*cell_m - cell_m/2;

    r  = dynObs(i).rad*cell_m;
    rg = r + safeDist_m;

    % body obstacle
    set(hDyn(i), ...
        'XData', px + r*cos(theta), ...
        'YData', py + r*sin(theta));

    % guard circle
    set(hDynGuard(i), ...
        'XData', px + rg*cos(theta), ...
        'YData', py + rg*sin(theta));

end
    %% 1) Cek waypoint (jarak euklidean)
    if ~reached_wp
        dist_wp = hypot(waypoint(1)-x, waypoint(2)-y);
        if dist_wp < wp_tol
            reached_wp = true;
            fprintf('Waypoint reached at t = %.2f s (dist = %.3f grid)\n', t, dist_wp);
        end
    end

    %% 2) Proyeksi posisi ke path global
    if force_replan_track

    idx = replan_idx_start;

    p_ref = pAll(idx,:);

    idx2 = min(idx+1,size(pAll,1));

    psi_path = atan2( ...
        pAll(idx2,2)-pAll(idx,2), ...
        pAll(idx2,1)-pAll(idx,1));

    s_on = cumAll(idx);
    remaining = cumAll(end)-s_on;

    % kalau sudah dekat titik ini, lanjut normal lagi
    if norm([x y]-p_ref) < 0.6
        force_replan_track = false;
    end

else

    [s_on, psi_path, p_ref, ~] = projectOnPath(pAll, cumAll, [x y]);
    remaining = cumAll(end) - s_on;

end

    % Paksa referensi ke waypoint saat mendekati (supaya benar2 "nyentuh")
    if ~reached_wp && (s_on > s_wp - 0.5)   % 0.5 grid ~ 2 m sebelum waypoint
        p_ref = waypoint;
        idx2 = min(idx_wp+1, size(pAll,1));
        psi_path = atan2( ...
            pAll(idx2,2) - pAll(idx_wp,2), ...
            pAll(idx2,1) - pAll(idx_wp,1));
    end

    %% 3) Cek goal hanya SETELAH waypoint tercapai
    dist_goal = hypot(goal(1)-x, goal(2)-y);
    if reached_wp && (dist_goal < goal_tol)
        disp('Goal reached!');
        x = goal(1); y = goal(2);
        traj=[traj; x y];
        x_log=[x_log; x]; y_log=[y_log; y]; psi_log=[psi_log; psi]; phi_log=[phi_log; phi];


        [~, psi_path_g, p_ref_g, ~] = projectOnPath(pAll, cumAll, [x y]);
        psi_des_g = psi_path_g;
        x_des_log=[x_des_log; p_ref_g(1)];
        y_des_log=[y_des_log; p_ref_g(2)];
        psi_des_log=[psi_des_log; psi_des_g];
        phi_des_log=[phi_des_log; phi_des_prev];
        break;
    end

    %% 4) DETEKSI OBSTACLE TAMBAHAN DENGAN KAMERA
 enable_extra_obstacle = true;
 %% ===== CAMERA DETECT DYNAMIC OBSTACLE =====
obs_cam = [];

for i=1:2

    [seen, meas] = camera_detect([x y], psi, dynObs(i), camera);

    if seen
        obs_cam = [obs_cam;
                   meas.pos_est dynObs(i).rad];
    end

end
    need_replan = false;
 if enable_extra_obstacle
    if ~extra_known
        [seenExtra, measExtra] = camera_detect([x y], psi, extraGT, camera);
        if seenExtra
            extra_known = true;
            fprintf('Obstacle tambahan terdeteksi kamera pada t = %.2f s\n', t);

            % Untuk demonstrasi: gunakan posisi GROUND TRUTH dari obstacle
            % (bisa diganti measExtra.pos_est untuk lebih "realistis")
            newObs_m = extraObs_gt_m;
            newObs   = [m2g(newObs_m(1:2)) newObs_m(3)/cell_m];

            % masukkan ke planner & guard
            obstacles_planning_m = [obstacles_planning_m; newObs_m];
            obstacles_planning   = [obstacles_planning;   newObs];
            obstacles_rt         = [obstacles_rt;         newObs];

            % gambar obstacle ini di Figure 2
            cxN = newObs_m(1); cyN = newObs_m(2); rN = newObs_m(3);
            rN_guard = rN + safeDist_m;
            set(hExtraFill,'XData',cxN + rN*cos(theta), ...
                           'YData',cyN + rN*sin(theta));
            set(hExtraGuard,'XData',cxN + rN_guard*cos(theta), ...
                            'YData',cyN + rN_guard*sin(theta));

            need_replan = true;
        end
    end
 end
    %% 5) REPLAN D* LITE SETELAH OBSTACLE BARU DIKETAHUI
    if need_replan && ~replan_done
        disp('>> Replanning global path dengan D* Lite (setelah obstacle tambahan diketahui)...');

        % rebuild map dinamis (menggunakan obstacles_planning_m, sekarang termasuk obstacle tambahan)
        map_dyn = zeros(nR,nC);
        for kk=1:size(obstacles_planning_m,1)
            cx   = obstacles_planning_m(kk,1);
            cy   = obstacles_planning_m(kk,2);
            rOcc = obstacles_planning_m(kk,3) + occInflate_m;
            rOcc = max(rOcc, minOcc_m);
            map_dyn( (xcGrid-cx).^2 + (ycGrid-cy).^2 <= rOcc^2 ) = 1;
        end

        % LOGIKA:
        %  - Jika USV masih di antara Start–Waypoint (belum lewat waypoint):
        %       -> HANYA replan segmen Waypoint → Goal (prefix Start→Waypoint dipertahankan).
        %  - Jika USV sudah lewat waypoint:
        %       -> replan langsung dari posisi sekarang ke Goal.
    % LOGIKA BARU:
    %  - START REPLAN SELALU POSISI SEKARANG [x y]
    %  - Kalau BELUM lewat waypoint  -> paksa replan lewat waypoint
    %  - Kalau SUDAH lewat waypoint  -> replan langsung ke goal

    if ~reached_wp
        % ===== KASUS: waypoint belum tercapai =====
        fprintf('Replan dari POSISI SEKARANG -> Waypoint -> Goal\n');

        % 1) path dari POSISI SEKARANG ke WAYPOINT
        [pathSW_new, expandedReplan_SW] = dstarLite_grid(map_dyn, [x y], waypoint, w);

        % 2) path dari WAYPOINT ke GOAL
        [pathWG_new, expandedReplan_WG] = dstarLite_grid(map_dyn, waypoint, goal, w);

        fprintf('Expanded nodes REPLAN pos->W = %d, W->G = %d\n', ...
                expandedReplan_SW, expandedReplan_WG);

        if ~isempty(pathSW_new) && ~isempty(pathWG_new)
            % paksa endpoint sesuai definisi
            pathSW_new(1,:)   = [x y];
            pathSW_new(end,:) = waypoint;
            pathWG_new(1,:)   = waypoint;
            pathWG_new(end,:) = goal;

            % smoothing & clearance untuk masing-masing segmen
% Gabungkan path DISKRIT dulu
pathAll_new = [pathSW_new; pathWG_new(2:end,:)];

% Smooth SEKALI saja untuk path global
pAll_smooth = smooth_path_g2cbs_c2(pathAll_new, samplesPerSeg, epsRDP);

% Clearance enforcement satu kali
[pAll, infoAll_re] = enforce_safe_clearance(pAll_smooth, obstacles_planning, safeDistPlan, ...
    'maxIter',80,'gain',0.6,'maxStep',0.2,'lambda',0.15,'ds',0.25);



            pAll(end,:) = goal;
pAll_replan = pAll;
didReplan   = true;

            % update data path global untuk tracking
            cumAll = cumulativeArc(pAll);
            [~, idx_wp] = min(vecnorm(pAll - waypoint, 2, 2));
            s_wp        = cumAll(idx_wp);

            % reset integral ILOS biar nggak "kaget"
            Ye_int  = 0;
            Ye_prev = 0;
        else
            warning('Replan pos->W atau W->G gagal, path lama dipakai.');
        end

    else
        % ===== KASUS: waypoint sudah tercapai, replan langsung ke GOAL =====
        fprintf('Replan dari POSISI SEKARANG -> GOAL\n');

        [pathNew, expandedReplan] = dstarLite_grid(map_dyn, [x y], goal, w);
        fprintf('Expanded nodes REPLAN (pos sekarang -> goal) = %d\n', expandedReplan);

        if ~isempty(pathNew)
            pathNew(1,:)   = [x y];
            pathNew(end,:) = goal;

pAll_smooth = smooth_path_g2cbs_c2(pathNew, samplesPerSeg, epsRDP);

[pAll, infoAll2] = enforce_safe_clearance(pAll_smooth, obstacles_planning, safeDistPlan, ...
    'maxIter',80,'gain',0.6,'maxStep',0.2,'lambda',0.15,'ds',0.25);

            fprintf('Min clearance global setelah replan (grid)  = %.3f\n',  infoAll2.min_clearance);
            fprintf('Min clearance global setelah replan (meter) = %.3f m\n',infoAll2.min_clearance*cell_m);

            pAll(end,:) = goal;

            pAll_replan = pAll;
            didReplan   = true;

            cumAll = cumulativeArc(pAll);

            Ye_int  = 0;
            Ye_prev = 0;
        else
            warning('Replan (pos->goal) gagal, path lama dipakai.');
        end
    end

    % update path global (meski tidak ditampilkan besar-besaran)
    pAll_m = g2m(pAll);
    set(hSmooth,'XData',pAll_m(:,1),'YData',pAll_m(:,2));

replan_done = true;
% mulai tracking dari jalur baru
[~,~,~,seg_now] = projectOnPath(pAll,cumAll,[x y]);

replan_idx_start = min(seg_now+1,size(pAll,1));
force_replan_track = true;

% ==================================================
% PAKSA TRACKER PINDAH KE PATH BARU
% ==================================================
[s_on, psi_path, p_ref, seg_now] = projectOnPath(pAll, cumAll, [x y]);

% ambil titik 2 langkah di depan segmen sekarang
idx_ref = min(seg_now + 2, size(pAll,1));

% referensi baru
p_ref = pAll(idx_ref,:);

% heading mengikuti segmen baru
idx_prev = max(idx_ref-1,1);

psi_path = atan2( ...
    pAll(idx_ref,2) - pAll(idx_prev,2), ...
    pAll(idx_ref,1) - pAll(idx_prev,1));

remaining = cumAll(end) - s_on;

% reset integral guidance
Ye_int  = 0;
Ye_prev = 0;
    end

    %% 6) ILOS Guidance
    Ld_local    = max(0.4, min(Ld, 0.6*remaining));
    v_ref_seg   = v_ref * max(0.3, min(1.0, remaining/3.0));

    [psi_des, Ye, Ye_int, Ye_prev] = ilos_guidance( ...
        p_ref, psi_path, [x y], Ye_int, Ye_prev, ilosParams, dt);
%% ===== SAPF FROM CAMERA =====
if ~isempty(obs_cam)

   [psi_des, v_ref_seg, ~] = SAPF_camera( ...
        [x y], psi, psi_des, obs_cam, sapf);

end
    % log desired
    x_des_log  = [x_des_log; p_ref(1)];
    y_des_log  = [y_des_log; p_ref(2)];
    psi_des_log=[psi_des_log; psi_des];

    obstacles_rt = obstacles_static;
if ~isempty(obs_cam)
    obstacles_rt = [obstacles_rt; obs_cam];
end

    %% 7) Controller + guard + sway PD
    Tcmd = controller_guard_4dof_ilos([x y psi], nu, psi_des, v_ref_seg, Ld_local, ...
            obstacles_rt, safeDist, avoid, ...
            Ku, Kr, Tmax, Nmax, r_max_cmd, ...
            Kv, Kdv, Ymax, v_ref_sway);

Fx = Tcmd(1);
Fy = Tcmd(2);
Mz = Tcmd(3);

%% ===== BANKING CONTROL =====
u = nu(1);
v = nu(2);
r = nu(3);
p = nu(4);
Ld_bank = max(0.5, Ld);;      % lookahead efektif
psi_err = atan2(sin(psi_des - psi), cos(psi_des - psi));
kappa = 2*sin(psi_err)/Ld_bank;     % estimasi kelengkungan belok

U_eff = max(0.3, hypot(u,v));       % speed kapal

a_y_cmd = U_eff^2 * kappa;          % lateral acceleration

%phi_cmd = bank.k_bank * atan(a_y_cmd/params.g);
phi_cmd = 5*atan(a_y_cmd/params.g) + 0.3*phi;
phi_cmd = max(-bank.phi_max,min(bank.phi_max,phi_cmd));

tau_phi = 0.25;
alpha_phi = dt/(tau_phi+dt);

phi_des = phi_des_prev + alpha_phi*(phi_cmd - phi_des_prev);
phi_des_log = [phi_des_log; phi_des];

phi_des_prev = phi_des;

Kphi = 40;
e_phi = phi_des - phi;
eInt_phi = eInt_phi + e_phi*dt;
Kphi_p = 6;
Kphi_i = 0.5;
Kphi_d = 3;

Tk = Kphi_p*e_phi ...
   + Kphi_i*eInt_phi ...
   - Kphi_d*p;


%% actuator limits
Tx = max(-lims.TX, min(lims.TX, Fx));
Ty = max(-lims.TY, min(lims.TY, Fy));
Tn = max(-lims.TN, min(lims.TN, Mz));
Tk = max(-lims.TK, min(lims.TK, Tk));

T = [Tx; Ty; Tn; Tk];

[Vdot, eta_dot] = usv4dof(nu, T, psi, phi, params);

%% UPDATE STATE
nu = nu + dt*Vdot;

u_max_state = 2.0*v_ref;
v_max_state = 2.0*v_ref;
r_max_state = 0.7;

nu(1)=max(-u_max_state,min(u_max_state,nu(1)));
nu(2)=max(-v_max_state,min(v_max_state,nu(2)));
nu(3)=max(-r_max_state,min(r_max_state,nu(3)));
nu(4)=max(-2,min(2,nu(4)));

x   = x   + eta_dot(1)*dt;
y   = y   + eta_dot(2)*dt;
psi = psi + eta_dot(3)*dt;
phi = phi + eta_dot(4)*dt;

    if any(~isfinite(nu)) || ~isfinite(psi) || ~isfinite(x) || ~isfinite(y)
        error('State menjadi non-finite pada tracking global');
    end

    traj=[traj; x y];
    x_log=[x_log; x]; y_log=[y_log; y]; psi_log=[psi_log; psi]; phi_log=[phi_log; phi];


    % CTE log
    [~, segIdx] = min(vecnorm(pAll - [x y], 2, 2));
    segIdx = max(2, min(segIdx, size(pAll,1)));
    cte_log=[cte_log; crossTrackError([x y], pAll(segIdx-1,:), pAll(segIdx,:))];

    % jarak ke obstacle terdekat (dari obstacles_rt)
    dObs = inf;
    for k=1:size(obstacles_rt,1)
        cx=obstacles_rt(k,1); cy=obstacles_rt(k,2); r0=obstacles_rt(k,3);
        dObs=min(dObs,hypot(x-cx,y-cy)-r0);
    end
    dist_log=[dist_log; dObs];
    time_log=[time_log; t];
    t=t+dt;

    % update plot kapal & traj (meter)
    pos_m = g2m([x y]);
    set(tKapal,'Matrix',makehgtform('translate',[pos_m(1) pos_m(2) 0],'zrotate',psi));

    traj_m = g2m(traj);
    set(hTrace,'XData',traj_m(:,1),'YData',traj_m(:,2));

    drawnow;
end

%% ===== PLOTS LOGGING =====
figure(3); clf;
plot(time_log,dist_log*cell_m,'b-','LineWidth',1.5); hold on;
yline(safeDist*cell_m,'r--','Minimum Safe Distance');
xlabel('Time (s)'); ylabel('Min Distance to Obstacle [m]');
title('USV Distance to Obstacle'); grid on;

figure(4); clf;
plot(time_log,abs(cte_log*cell_m),'m-','LineWidth',1.5); grid on;
xlabel('Time (s)'); ylabel('|CTE| (m)'); title('Cross-Track Error vs Time');

figure(5); clf;
set(gcf,'Color','w','Position',[150 80 950 750]);

N = numel(x_log);
tvec = (0:N-1)' * dt;

Ndes = min([numel(tvec), numel(x_des_log), numel(y_des_log), ...
            numel(psi_des_log), numel(phi_des_log), ...
            numel(x_log), numel(y_log), ...
            numel(psi_log), numel(phi_log)]);

if Ndes > 0

    tvec = tvec(1:Ndes);

    % actual
    x_act   = x_log(1:Ndes)*cell_m;
    y_act   = y_log(1:Ndes)*cell_m;
    psi_act = unwrap(psi_log(1:Ndes))*180/pi;
    phi_act = phi_log(1:Ndes)*180/pi;

    % desired
    x_des   = x_des_log(1:Ndes)*cell_m;
    y_des   = y_des_log(1:Ndes)*cell_m;
    psi_des = unwrap(psi_des_log(1:Ndes))*180/pi;

    phi_des = phi_des_log(1:Ndes)*180/pi;

    %% ===== X =====
    subplot(4,1,1)
    plot(tvec,x_act,'b','LineWidth',1.8); hold on
    plot(tvec,x_des,'r--','LineWidth',1.5)
    grid on
    ylabel('X (m)')
    title('USV States vs Time')

    %% ===== Y =====
    subplot(4,1,2)
    plot(tvec,y_act,'b','LineWidth',1.8); hold on
    plot(tvec,y_des,'r--','LineWidth',1.5)
    grid on
    ylabel('Y (m)')

    %% ===== YAW =====
    subplot(4,1,3)
    plot(tvec,psi_act,'b','LineWidth',1.8); hold on
    plot(tvec,psi_des,'r--','LineWidth',1.5)
    grid on
    ylabel('\psi (deg)')

    %% ===== ROLL =====
    subplot(4,1,4)
    plot(tvec,phi_act,'b','LineWidth',1.8); hold on
    plot(tvec,phi_des,'r--','LineWidth',1.5)

    yline(5,'k:');
    yline(-5,'k:');

    grid on
    ylabel('\phi (deg)')
    xlabel('Time (s)')

    linkaxes(findall(gcf,'Type','axes'),'x')

else
    warning('Tidak ada sampel untuk Figure 5');
end
%% ===== FIGURE 6: Perbandingan Global Path Sebelum & Sesudah Replan =====
if didReplan && ~isempty(pAll_replan)
    figure(6); clf; hold on; axis equal; grid on;
    xlabel('X [m]'); ylabel('Y [m]');
    axis([0 mapSize_m(2) 0 mapSize_m(1)]);
    title('Global Path: Sebelum vs Sesudah Replan (Obstacle Tambahan)');

    th = linspace(0,2*pi,60);

    % Obstacles awal
    for k=1:size(obstacles_static_m,1)
        cx = obstacles_static_m(k,1); cy = obstacles_static_m(k,2); r_phys = obstacles_static_m(k,3);
        r_guard = r_phys + safeDist_m;
        fill(cx + r_phys*cos(th), cy + r_phys*sin(th), ...
             'r','FaceAlpha',0.3,'EdgeColor','none');
        plot(cx + r_guard*cos(th), cy + r_guard*sin(th), ...
             'r--','LineWidth',0.8);
    end

    % obstacle tambahan
    cxN = extraObs_gt_m(1); cyN = extraObs_gt_m(2); rN = extraObs_gt_m(3);
    rN_guard = rN + safeDist_m;
    fill(cxN + rN*cos(th), cyN + rN*sin(th), ...
         'm','FaceAlpha',0.3,'EdgeColor','none');
    plot(cxN + rN_guard*cos(th), cyN + rN_guard*sin(th), ...
         'm--','LineWidth',0.8);

    % Titik penting
    plot(start_m(1),   start_m(2),   'yo','MarkerFaceColor','y','DisplayName','Start');
    plot(waypoint_m(1),waypoint_m(2),'mo','MarkerFaceColor','m','DisplayName','Waypoint');
    plot(goal_m(1),    goal_m(2),    'ro','MarkerFaceColor','r','DisplayName','Goal');

    % Path awal & setelah replan (meter)
    p_init_m   = g2m(pAll_init);
    p_replan_m = g2m(pAll_replan);

    h1 = plot(p_init_m(:,1),  p_init_m(:,2),  'k--','LineWidth',1.8,'DisplayName','Path awal');
    h2 = plot(p_replan_m(:,1),p_replan_m(:,2),'b-','LineWidth',2.0,'DisplayName','Path setelah replan');

    legend([h1 h2],'Location','northeastoutside');
end

%% ==================== FUNCTIONS ====================

function [path, expanded] = dstarLite_grid(map, start_xy, goal_xy, w)
    if nargin < 4, w = 1; end
    [nR, nC] = size(map);
    traversable = (map == 0);

    % --- start & goal (grid index) ---
    sx = min(max(round(start_xy(1)),1),nC);
    sy = min(max(round(start_xy(2)),1),nR);
    gx = min(max(round(goal_xy(1)),1), nC);
    gy = min(max(round(goal_xy(2)),1), nR);

    traversable(sy,sx) = true;
    if ~traversable(gy,gx), error('Goal berada di dalam obstacle.'); end

    S   = @(x,y) sub2ind([nR nC], y, x);
    sid = S(sx,sy);
    gid = S(gx,gy);

    INF = 1e12;
    g   = INF*ones(nR*nC,1);
    rhs = INF*ones(nR*nC,1);
    km  = 0;

    U_key = [];
    U_id  = [];

    % --- counter node yang diekspansi ---
    expanded = 0;

    function normalizeOpen()
        if isempty(U_id) || isempty(U_key)
            U_key = []; U_id = [];
            return;
        end
        n = min(size(U_key,1), size(U_id,1));
        U_key = U_key(1:n,:);
        U_id  = U_id(1:n,1);
    end

    function h = H(a,b)
        [ay,ax] = ind2sub([nR nC], a);
        [by,bx] = ind2sub([nR nC], b);
        h = w * hypot(double(ax-bx), double(ay-by));
    end
    function K = CalcKey(s)
        K = [min(g(s), rhs(s)) + H(sid, s) + km, min(g(s), rhs(s))];
    end
    function [val, idx] = minrows(A)
        [~, idx] = min(A(:,1) + 1e-12*A(:,2));
        val = A(idx,:);
    end
    function push(s)
        normalizeOpen();
        U_key(end+1,:) = CalcKey(s);
        U_id (end+1,1) = s;
    end
    function [k,u,idx] = top()
        normalizeOpen();
        if isempty(U_id), k=[INF INF]; u=-1; idx=[]; return; end
        [k, idx] = minrows(U_key);
        u = U_id(idx);
    end
    function [k,u] = pop()
        [k,u,idx] = top();
        if u~=-1 && ~isempty(idx)
            U_key(idx,:) = [];
            U_id(idx)    = [];
        end
        normalizeOpen();
    end

    moves = [ 1 0; -1 0; 0 1; 0 -1; 1 1; -1 -1; 1 -1; -1 1 ];
    cost  = [ 1    1     1     1    sqrt(2) sqrt(2) sqrt(2) sqrt(2) ];

    function N = Succ(s)
        [y,x] = ind2sub([nR nC], s);
        N = [];
        for i=1:8
            nx = x + moves(i,1); ny = y + moves(i,2);
            if nx>=1 && ny>=1 && nx<=nC && ny<=nR && traversable(ny,nx)
                N(end+1,:) = [ sub2ind([nR nC], ny, nx), cost(i) ]; %#ok<AGROW>
            end
        end
    end
    function P = Pred(s)
        P = Succ(s);
    end

    rhs(gid) = 0;
    push(gid);

    function UpdateVertex(u)
        normalizeOpen();
        if u ~= gid
            su = Succ(u);
            if isempty(su)
                rhs(u) = INF;
            else
                rhs(u) = min( su(:,2) + g(su(:,1)) );
            end
        end
        if ~isempty(U_id)
            idx_keep = find(U_id ~= u);
            U_id  = U_id(idx_keep,:);
            U_key = U_key(idx_keep,:);
        end
        if g(u) ~= rhs(u)
            U_key(end+1,:) = CalcKey(u);
            U_id (end+1,1) = u;
        end
        normalizeOpen();
    end

    function ComputeShortestPath()
        [k_old,~,~] = top();
        k_start = CalcKey(sid);
        while any(k_old < k_start) || (rhs(sid) ~= g(sid))
            [~,u] = pop();
            if u == -1, break; end

            % node u dipop dari OPEN -> dianggap "expanded"
            expanded = expanded + 1;

            if g(u) > rhs(u)
                g(u) = rhs(u);
                P = Pred(u);
                for ii=1:size(P,1), UpdateVertex(P(ii,1)); end
            else
                g(u)  = INF;
                P = Pred(u);
                UpdateVertex(u);
                for ii=1:size(P,1), UpdateVertex(P(ii,1)); end
            end
            [k_old,~,~] = top();
            k_start     = CalcKey(sid);
        end
    end

    ComputeShortestPath();

    if isinf(g(sid)), path = []; return; end
    cur = sid; path = [sx sy]; maxStep = nR*nC; step = 0;
    while cur ~= gid && step < maxStep
        su = Succ(cur);
        if isempty(su), path = []; return; end
        [~,idx] = min( su(:,2) + g(su(:,1)) );
        cur = su(idx,1);
        [yy,xx] = ind2sub([nR nC], cur);
        path(end+1,:) = [xx yy]; %#ok<AGROW>
        step = step + 1;
    end
end

function [psi_des, Ye, Ye_int, Ye_prev] = ilos_guidance( ...
        p_ref, psi_path, pos, Ye_int, Ye_prev, params, dt)
    x = pos(1); y = pos(2);
    xd = p_ref(1); yd = p_ref(2);
    a  = psi_path;
    Ye = -(x - xd)*sin(a) + (y - yd)*cos(a);
    Ye_int = Ye_int + Ye*dt;
    dYe    = (Ye - Ye_prev)/max(dt,1e-6);
    Ye_prev = Ye;

    k_ip = params.kp;
    k_i  = params.ki;
    k_id = params.kd;
    arg = k_ip*Ye + k_i*Ye_int + k_id*dYe;
    psi_des = a - atan(arg);
end

function T = controller_guard_4dof_ilos(eta,Vb,psi_des,u_ref,Ld, ...
                                   obstacles,safeDist,avoid, ...
                                   Ku,Kr,Tmax,Nmax,r_max_cmd, ...
                                   Kv,Kdv,Ymax,v_ref_sway)
    x=eta(1); y=eta(2); psi=eta(3); u=Vb(1); v=Vb(2); r=Vb(3);

    d_min=inf; avoid_dir=[0 0]; r0=0;
    for kk=1:size(obstacles,1)
        cx=obstacles(kk,1); cy=obstacles(kk,2); rr=obstacles(kk,3);
        vv=[x-cx,y-cy]; d=norm(vv);
        if d<d_min, d_min=d; avoid_dir=vv/max(d,1e-9); r0=rr; end
    end
    clr   = d_min - r0;
    clr_s = clr - safeDist;

    Ld_scale = 1.0;
    if clr < avoid.buffer*safeDist
        Ld_scale = max(avoid.minLdScale, min(1.0, clr/(avoid.buffer*safeDist)));
    end
    Ld_eff = max(0.5, Ld*Ld_scale); %#ok<NASGU>

    yaw_avoid = 0;
    u_ref_eff = u_ref;
    if clr < avoid.buffer*safeDist
        psi_away = atan2(avoid_dir(2),avoid_dir(1));
        dpsiAway = atan2(sin(psi_away-psi), cos(psi_away-psi));
        yaw_avoid = avoid.yaw_gain * dpsiAway;

        if clr_s < 0
            scale = max(avoid.slowdownMin, clr/safeDist);
            u_ref_eff = u_ref * max(0.05, min(1.0, scale));
        end
    end

e_psi = atan2(sin(psi_des-psi), cos(psi_des-psi));

Kpsi = 1.8;
Kr   = 22.0;
Kd_r = 10.0;
    r_cmd = yaw_avoid + Kpsi*e_psi;
    r_cmd = max(-r_max_cmd, min(r_max_cmd, r_cmd));

% speed scheduling
u_scale = max(0.2, cos(e_psi));
u_cmd   = u_ref_eff * u_scale;

Fx = Ku*(u_cmd-u);
Fx = max(-Tmax,min(Tmax,Fx));

Mz = Kr*(r_cmd-r) - Kd_r*r;
Mz = max(-Nmax,min(Nmax,Mz));

Fy = Kv*(v_ref_sway - v) - Kdv*v;
Fy = max(-Ymax, min(Ymax, Fy));

T = [Fx Fy Mz];

end

function cum = cumulativeArc(P)
    if size(P,1)<2, cum=0;
    else, cum=[0; cumsum(sqrt(sum(diff(P).^2,2)))];
    end
end

function [s_on, psi_path, p_ref, seg] = projectOnPath(P, cum, p)
    N=size(P,1); best=inf; s_on=0; seg=1; p_ref=P(1,:);
    for i=1:N-1
        A=P(i,:); B=P(i+1,:); AB=B-A; L=dot(AB,AB); if L<1e-9, continue; end
        t=dot(p-A,AB)/L; t=max(0,min(1,t));
        proj=A+t*AB; d=norm(p-proj);
        if d<best
            best=d; seg=i; s_on=cum(i)+t*norm(AB);
            p_ref = proj;
        end
    end
    psi_path = atan2(P(seg+1,2)-P(seg,2), P(seg+1,1)-P(seg,1));
end

function e_ct = crossTrackError(p, p1, p2)
    A = [p2(1)-p1(1); p2(2)-p1(2)];
    B = [p(1)-p1(1);  p(2)-p1(2)];
    e_ct = (A(1)*B(2) - A(2)*B(1)) / max(1e-6, norm(A));
end

function Ps = smooth_path_g2cbs_c2(P, nPerSeg, epsRDP)
    if nargin<2, nPerSeg=40; end
    if nargin<3, epsRDP=0; end
    P = remove_dups(P);
    if size(P,1) <= 2, Ps = P; return; end
    if epsRDP > 0 && size(P,1) > 3
        P = rdp(P, epsRDP); P = remove_dups(P);
        if size(P,1) <= 2, Ps = P; return; end
    end
    N = size(P,1);
    t = zeros(N,1);
    for i=2:N, t(i) = t(i-1) + norm(P(i,:) - P(i-1,:)); end
    if t(end) < 1e-9, Ps = P(1,:); return; end
    h = diff(t);

    Mx = natural_spline_second_derivs(t, P(:,1));
    My = natural_spline_second_derivs(t, P(:,2));

    mx = zeros(N,1); my = zeros(N,1);
    sx = diff(P(:,1))./h; sy = diff(P(:,2))./h;

    mx(1) = sx(1) - h(1)*(2*Mx(1)+Mx(2))/6;
    my(1) = sy(1) - h(1)*(2*My(1)+My(2))/6;
    for i=2:N-1
        mLx = sx(i-1) + h(i-1)*(Mx(i-1)+2*Mx(i))/6;
        mRx = sx(i)   - h(i)  *(2*Mx(i)+Mx(i+1))/6;
        mx(i) = 0.5*(mLx + mRx);
        mLy = sy(i-1) + h(i-1)*(My(i-1)+2*My(i))/6;
        mRy = sy(i)   - h(i)  *(2*My(i)+My(i+1))/6;
        my(i) = 0.5*(mLy + mRy);
    end
    mx(N) = sx(end) + h(end)*(Mx(end-1)+2*Mx(end))/6;
    my(N) = sy(end) + h(end)*(My(end-1)+2*My(end))/6;

    Ps = P(1,:);
    for i = 1:N-1
        hi = h(i);
        b0 = P(i,:);      b3 = P(i+1,:);
        b1 = b0 + (hi/3) * [mx(i),   my(i)];
        b2 = b3 - (hi/3) * [mx(i+1), my(i+1)];
        tau = linspace(0,1,nPerSeg).';
        B = (1-tau).^3 .* b0 ...
          + 3*(1-tau).^2 .* tau .* b1 ...
          + 3*(1-tau)    .* tau.^2 .* b2 ...
          + tau.^3 .* b3;
        if i > 1, B = B(2:end,:); end
        Ps = [Ps; B];
    end
    Ps = remove_dups(Ps);
end

function M = natural_spline_second_derivs(t, y)
    N = numel(y);
    h = diff(t);
    if N <= 2, M = zeros(N,1); return; end
    A = zeros(N,N); d = zeros(N,1);
    A(1,1) = 1; d(1) = 0; A(N,N) = 1; d(N) = 0;
    for i = 2:N-1
        A(i,i-1) = h(i-1);
        A(i,i)   = 2*(h(i-1)+h(i));
        A(i,i+1) = h(i);
        d(i) = 6 * ( (y(i+1)-y(i))/h(i) - (y(i)-y(i-1))/h(i-1) );
    end
    M = A \ d;
end

function [Pout, info] = enforce_safe_clearance(Pin, obs, safeDist, varargin)
    ip = inputParser;
    ip.addParameter('maxIter',80);
    ip.addParameter('gain',0.6);
    ip.addParameter('maxStep',0.8);
    ip.addParameter('lambda',0.15);
    ip.addParameter('ds',1.0);
    ip.parse(varargin{:}); prm = ip.Results;
    P = resample_by_arclength(Pin, prm.ds);
    N=size(P,1);
    if N<=2, Pout=P; info.min_clearance=Inf; info.iterations=0; return; end
    for it=1:prm.maxIter
        dP = zeros(N,2); vio=false(N,1);
        for i=2:N-1
            pi = P(i,:); push=[0 0];
            for k=1:size(obs,1)
                c=obs(k,1:2); R=obs(k,3)+safeDist;
                v=pi-c; d=norm(v)+1e-9; clr=d-R;
                if clr<0, dir=v/d; push=push+prm.gain*(-clr)*dir; vio(i)=true; end
            end
            smooth = prm.lambda * ((P(i-1,:)+P(i+1,:))/2 - P(i,:));
            dP(i,:) = push + smooth;
        end
        for i=2:N-1
            nrm=norm(dP(i,:)); if nrm>prm.maxStep, dP(i,:)=dP(i,:)/nrm*prm.maxStep; end
        end
        P(2:N-1,:) = P(2:N-1,:) + dP(2:N-1,:);
        if ~any(vio), break; end
    end
    Pout=remove_dups(P); info.iterations=it;
    info.min_clearance=min_clearance_poly(Pout, obs, safeDist);
end

function m = min_clearance_poly(P, obs, safeDist)
    m=inf;
    for k=1:size(obs,1)
        c=obs(k,1:2); R=obs(k,3)+safeDist;
        d = sqrt((P(:,1)-c(1)).^2 + (P(:,2)-c(2)).^2) - R;
        m = min(m, min(d));
    end
end

function Q = resample_by_arclength(P, ds)
    if size(P,1)<=2 || ds<=0, Q=P; return; end
    s = cumulativeArc(P); ss = 0:ds:s(end); if ss(end)<s(end), ss=[ss s(end)]; end
    Q = interp1(s, P, ss, 'linear');
end

function Q = rdp(P, eps)
    if size(P,1)<=2, Q=P; return; end
    [d, idx] = maxPointDist(P);
    if d > eps
        Q1 = rdp(P(1:idx,:), eps); Q2 = rdp(P(idx:end,:), eps);
        Q  = [Q1(1:end-1,:); Q2];
    else
        Q = [P(1,:); P(end,:)];
    end
end

function [dmax, idx] = maxPointDist(P)
    A=P(1,:); B=P(end,:); AB=B-A; L2=max(1e-12,sum(AB.^2));
    dmax=-1; idx=1;
    for i=2:size(P,1)-1
        AP=P(i,:)-A; t=max(0,min(1,(AP*AB')/L2)); proj=A+t*AB;
        d=norm(P(i,:)-proj); if d>dmax, dmax=d; idx=i; end
    end
    if dmax<0, dmax=0; idx=1; end
end

function Q = remove_dups(Q)
    if isempty(Q), return; end
    keep = [true; vecnorm(diff(Q,1,1),2,2) > 1e-8];
    Q = Q(keep,:);
end

function [Vdot, eta_dot] = usv4dof(V,T,psi,phi,P)

u=V(1); v=V(2); r=V(3); p=V(4);

m=P.m; mx=P.mx; my=P.my;
ay=P.alphay; lx=P.lx; ly=P.ly;
Iz=P.Iz; Jz=P.Jz;
Ix=P.Ix; Jx=P.Jx;

M = [m+mx 0 0 0;
     0 m+my my*ay-my*ly 0;
     0 my*ay Iz+Jz 0;
     0 -my*ly 0 Ix+Jx];

C = [0 -(m+my)*r 0 0;
     (m+mx)*r 0 0 0;
     0 0 0 0;
     0 0 -mx*lx*u 0];

Xd = P.Xu*u + P.Xuu*abs(u)*u;
Yd = P.Yv*v + P.Yvv*abs(v)*v;
Nd = P.Nr*r + P.Nrr*abs(r)*r;
Kd = P.Kp*p + P.Kpp*abs(p)*p;

tau_d = [Xd;Yd;Nd;Kd];

tau_rest = [0;0;0;-P.W*P.GMT*phi];
tau_cen  = [0;0;0;P.m*P.hCG*u*r];

rhs = T + tau_d + tau_rest + tau_cen - C*V;

Vdot = M\rhs;

xdot = u*cos(psi)-v*sin(psi);
ydot = u*sin(psi)+v*cos(psi);

eta_dot = [xdot; ydot; r; p];
end

function [seen, meas] = camera_detect(pos, psi, obsDyn, camera)
    meas = struct('dist',[], 'bearing',[], 'pos_est',[]);
    seen = false;

    if ~obsDyn.active
        return;
    end

    r_vo = obsDyn.pos - pos;
    d    = norm(r_vo);
    if d < 1e-6
        return;
    end

    if d < camera.minRange || d > camera.maxRange
        return;
    end

    bearing_world = atan2(r_vo(2), r_vo(1));
    alpha         = atan2( sin(bearing_world - psi), cos(bearing_world - psi) );

    if abs(alpha) > camera.fov/2
        return;
    end

    d_meas     = d     + camera.sigma_r * randn();
    alpha_meas = alpha + camera.sigma_b * randn();

    bearing_est = psi + alpha_meas;
    pos_est = pos + d_meas * [cos(bearing_est), sin(bearing_est)];

    meas.dist    = d_meas;
    meas.bearing = alpha_meas;
    meas.pos_est = pos_est;
    seen = true;
end

function dmin = minDistToPolyline(P, c)
% MINDISTTOPOLYLINE  Jarak minimum dari titik c ke polyline P (grid units)
% P : [N x 2], urutan titik polyline
% c : [1 x 2], titik (cx, cy)
    if size(P,1) < 2
        dmin = inf;
        return;
    end
    dmin = inf;
    cx = c(1); cy = c(2);
    for i = 1:size(P,1)-1
        p1 = P(i,:);   p2 = P(i+1,:);
        A = p2 - p1;
        L2 = max(1e-12, dot(A,A));
        t = max(0, min(1, ([cx cy]-p1)*A'/L2));
        proj = p1 + t*A;
        d = hypot(cx - proj(1), cy - proj(2));
        if d < dmin
            dmin = d;
        end
    end
end

function [psi_ref, v_ref, Ftot] = SAPF_camera(usvPos, psi_now, psi_path, obs_cam, param)

% =====================================================
% Safe Artificial Potential Field (Camera Based)
% Input:
% usvPos   = posisi kapal [x y]
% psi_now  = heading sekarang
% psi_path = heading dari global path (ILOS)
% obs_cam  = obstacle hasil kamera [x y r]
% param    = parameter SAPF
%
% Output:
% psi_ref  = heading referensi baru
% v_ref    = kecepatan referensi
% Ftot     = gaya total
% =====================================================

%% ===== Attractive Force ke Jalur =====
Fatt = param.zeta * [cos(psi_path), sin(psi_path)];

%% ===== Repulsive Force =====
Frep = [0 0];

for k = 1:size(obs_cam,1)

    obs = obs_cam(k,1:2);
    r   = obs_cam(k,3);

    vec = usvPos - obs;
    d   = norm(vec) - r;

    if d < param.qstar && d > 0.01

        dir = vec / norm(vec);

        mag = param.eta * (1/d - 1/param.qstar)/(d^2);

        Frep = Frep + mag*dir;

        %% VORTEX EFFECT
        vort = [-dir(2), dir(1)];

        Frep = Frep + param.dvort*mag*vort;

    end
end

%% ===== Total Force =====
Ftot = Fatt + Frep;

%% ===== Heading Reference =====
psi_ref = atan2(Ftot(2),Ftot(1));

%% ===== Adaptive Speed =====
err = atan2(sin(psi_ref-psi_now), cos(psi_ref-psi_now));

if abs(err) > param.alpha_th
    v_ref = param.v_nom * 0.5;
else
    v_ref = param.v_nom;
end

end