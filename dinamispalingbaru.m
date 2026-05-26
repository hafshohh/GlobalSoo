%% USV 4-DOF: D* Lite (Global) + SAPF Paper (Local) + Kamera Obstacle Kotak
% SAPF ref: Szczepanski, IEEE RA-L — "Safe Artificial Potential Field"
%   - Attractive force : menuju referensi ILOS (global path)
%   - Obstacle force   : superposisi repulsive + vortex via R(γ) (eq.12–17)
%   - Tuning analitik  : ζ & η dari eq.23 & eq.29
% Obstacle dinamis: 2 kotak 0.45×0.45 m, terdeteksi kamera FOV 120°
% Toggle SAPF      : ENABLE_SAPF = true / false

clear; clc; close all; rng(1);

%% ===== TOGGLE =====
ENABLE_SAPF = true;   % false → USV hanya ikut global path, obstacle menabrak

%% ===== GRID & PETA =====
cell_m    = 2;
mapSize_m = [33 50];
nR = ceil(mapSize_m(1)/cell_m);
nC = ceil(mapSize_m(2)/cell_m);
m2g = @(p) [p(:,1)/cell_m+0.5,  p(:,2)/cell_m+0.5];
g2m = @(p) [(p(:,1)-0.5)*cell_m, (p(:,2)-0.5)*cell_m];
[xcGrid, ycGrid] = meshgrid(((1:nC)-0.5)*cell_m, ((1:nR)-0.5)*cell_m);

%% ===== OBSTACLE STATIS =====
obstacles_static_m = [20 20 .25; 40 20 .25; 10 10 .25;
                      30 10 .25; 17 16.5 .25; 41 16 .25];
extraObs_gt_m = [35.2 19.3 0.25];   % unknown saat planning awal

start_m    = [ 1.0  8.0];
waypoint_m = [25.0 20.0];
goal_m     = [48.0 13.0];
start    = m2g(start_m);
waypoint = m2g(waypoint_m);
goal_g   = m2g(goal_m);

%% ===== OBSTACLE DINAMIS — KOTAK 0.45×0.45 m =====
% Posisi dipilih agar jalur USV tertabrak bila SAPF non-aktif
boxObs(1).pos_m = [15.0,  7.0];  % x=15, mulai bawah → bergerak NAIK
boxObs(1).vel_m = [0.0,   0.40]; % m/s  (melewati jalur ~t=12–15 s)
boxObs(1).hw    = 0.225;          % half-width [m]
boxObs(1).hh    = 0.225;          % half-height [m]

boxObs(2).pos_m = [37.0, 32.0];  % x=37, mulai atas  → bergerak TURUN
boxObs(2).vel_m = [0.0,  -0.40]; % m/s  (melewati jalur ~t=37–40 s)
boxObs(2).hw    = 0.225;
boxObs(2).hh    = 0.225;

%% ===== PARAMETER USV 4-DOF (LSS-01) =====
P = struct('m',11.8,'mx',1.5,'my',6.0,'B',0.82,'Iz',2.20,'Jz',0,...
    'Ix',1.00,'Jx',0,'alphay',0.20,'lx',0.15,'ly',0.10,...
    'GMT',0.04,'g',9.81,'hCG',0.20,...
    'Xu',-15,'Xuu',-5,'Yv',-30,'Yvv',-25,'Nr',-6,'Nrr',-12,'Kp',-4,'Kpp',-6);
P.W = P.m*P.g;
lims = struct('TX',200,'TY',60,'TN',80,'TK',60);

%% ===== PLANNING PARAMS =====
safeDist_m     = 1.5;
safePlan_m     = 0.3;
occInflate_m   = safePlan_m + safeDist_m;
minOcc_m       = 0.5*sqrt(2)*cell_m;
safeDist_g     = safeDist_m   / cell_m;
safeDistPlan_g = safePlan_m   / cell_m;
goal_tol       = 0.35;   % grid

obstacles_planning_m = obstacles_static_m;
obs_plan_g = [m2g(obstacles_planning_m(:,1:2)), obstacles_planning_m(:,3)/cell_m];
obs_stat_g = [m2g(obstacles_static_m(:,1:2)),   obstacles_static_m(:,3)/cell_m];

map = zeros(nR,nC);
for k = 1:size(obstacles_planning_m,1)
    cx=obstacles_planning_m(k,1); cy=obstacles_planning_m(k,2);
    r=max(obstacles_planning_m(k,3)+occInflate_m, minOcc_m);
    map((xcGrid-cx).^2+(ycGrid-cy).^2 <= r^2) = 1;
end

%% ===== D* LITE GLOBAL PLANNER =====
w = 1;
[pathSW, eSW] = dstarLite_grid(map, start, waypoint, w);
[pathWG, eWG] = dstarLite_grid(map, waypoint, goal_g, w);
if isempty(pathSW)||isempty(pathWG), error('Path tidak ditemukan!'); end
fprintf('D*Lite expanded: S->W=%d | W->G=%d\n', eSW, eWG);

pathSW(1,:)=start; pathSW(end,:)=waypoint;
pathWG(1,:)=waypoint; pathWG(end,:)=goal_g;

%% ===== SMOOTHING PER-SEGMEN (waypoint dijamin dilewati) =====
nSeg=25; epsRDP=0.3;
pSW=smooth_path_g2cbs_c2(pathSW,nSeg,epsRDP);
pWG=smooth_path_g2cbs_c2(pathWG,nSeg,epsRDP);
[pSW,~]=enforce_safe_clearance(pSW,obs_plan_g,safeDistPlan_g,...
    'maxIter',80,'gain',0.6,'maxStep',0.1,'lambda',0.05,'ds',0.25);
[pWG,~]=enforce_safe_clearance(pWG,obs_plan_g,safeDistPlan_g,...
    'maxIter',80,'gain',0.6,'maxStep',0.1,'lambda',0.05,'ds',0.25);
pSW(1,:)=start; pSW(end,:)=waypoint;
pWG(1,:)=waypoint; pWG(end,:)=goal_g;

pAll     = [pSW; pWG(2:end,:)];
pAll_init = pAll;
cumAll   = cumulativeArc(pAll);
[~,idx_wp] = min(vecnorm(pAll-waypoint,2,2));
s_wp = cumAll(idx_wp);

%% ===== SAPF PARAMS (tuning analitik — eq.23 & eq.29) =====
sapf.v_max        = 1.2;           % [m/s] kecepatan maks
sapf.a_max        = 0.30;          % [m/s²] deselerasi maks
sapf.d_g_star     = 5.0;           % margin jarak ke titik referensi [m]
sapf.d_safe       = safeDist_m;    % jarak aman minimum = 1.5 m
sapf.d_vort       = 3.5;           % jarak saat follow-along obstacle [m]
sapf.Q_star       = 7.0;           % batas reaksi obstacle [m]
sapf.alpha_th     = deg2rad(30);   % threshold sudut D(α) [rad]
sapf.theta_max_err = deg2rad(40);  % max orientation error [rad]
% ζ (eq.23)
sapf.zeta = sqrt(2*sapf.a_max*sapf.d_g_star) / sapf.d_g_star;
% η (eq.29)
sapf.eta = sapf.d_safe^2 * sapf.Q_star * ...
           (sapf.v_max - sapf.d_g_star*sapf.zeta) / ...
           (sapf.d_safe - sapf.Q_star);
fprintf('SAPF: zeta=%.4f | eta=%.4f\n', sapf.zeta, sapf.eta);

%% ===== KAMERA (meter) =====
cam.fov      = deg2rad(120);
cam.maxRange = 10.0;   % [m]
cam.minRange = 0.30;   % [m]

%% ===== CONTROLLER & TRACKING PARAMS =====
dt=0.05; v_ref=1.2/cell_m; Ld=6;
Ku=80; Kr_ctrl=80; Kv=60; Kdv=10; Ymax=60;
Tmax=120; Nmax=80; r_max=0.55; v_sway=0;
avoid.buffer=1.4; avoid.yaw_gain=1.2; avoid.target_shift=0;
avoid.minLdScale=0.45; avoid.slowdownMin=0.10;
ilos_p.kp=0.28; ilos_p.ki=0; ilos_p.kd=0.04;
wp_tol=0.35; reached_wp=false; phi_max=deg2rad(10);

%% ===== EXTRA STATIC OBS (unknown pada planning) =====
extraGT.pos    = m2g(extraObs_gt_m(1:2));
extraGT.rad    = extraObs_gt_m(3)/cell_m;
extraGT.active = true;
extra_known=false; replan_done=false;
newObs=[]; newObs_m=[];
obs_rt_g = obs_stat_g;

safeDist_show = safeDist_m;
th = linspace(0,2*pi,60);

%% ===== FIGURE 1: GLOBAL PLAN AWAL =====
figure(1); clf; hold on; axis equal; grid on; box on;
xlabel('X [m]'); ylabel('Y [m]'); axis([0 50 0 33]);
title('Figure 1: Global Plan Awal + Posisi Awal Obstacle Dinamis (Kotak)');

for k=1:size(obstacles_static_m,1)
    cx=obstacles_static_m(k,1); cy=obstacles_static_m(k,2);
    r=obstacles_static_m(k,3); rg=r+safeDist_show;
    fill(cx+r*cos(th),cy+r*sin(th),'r','FaceAlpha',0.3,'EdgeColor','none','HandleVisibility','off');
    plot(cx+rg*cos(th),cy+rg*sin(th),'r--','LineWidth',0.8,'HandleVisibility','off');
end
cxE=extraObs_gt_m(1); cyE=extraObs_gt_m(2); rE=extraObs_gt_m(3);
fill(cxE+rE*cos(th),cyE+rE*sin(th),'m','FaceAlpha',0.25,'EdgeColor','none','HandleVisibility','off');
plot(cxE+(rE+safeDist_show)*cos(th),cyE+(rE+safeDist_show)*sin(th),'m--','LineWidth',0.8,'HandleVisibility','off');

for i=1:2
    cx_b=boxObs(i).pos_m(1); cy_b=boxObs(i).pos_m(2);
    hw_b=boxObs(i).hw; hh_b=boxObs(i).hh;
    xb=[cx_b-hw_b cx_b+hw_b cx_b+hw_b cx_b-hw_b cx_b-hw_b];
    yb=[cy_b-hh_b cy_b-hh_b cy_b+hh_b cy_b+hh_b cy_b-hh_b];
    fill(xb,yb,[0 0.7 0.9],'FaceAlpha',0.6,'EdgeColor','b','LineWidth',1.5,'HandleVisibility','off');
    rg_b=sqrt(hw_b^2+hh_b^2)+safeDist_show;
    plot(cx_b+rg_b*cos(th),cy_b+rg_b*sin(th),'b--','LineWidth',0.8,'HandleVisibility','off');
    text(cx_b,cy_b+hh_b+0.8,sprintf('Box%d (t=0)',i),'FontSize',8,'Color','b','HorizontalAlignment','center','FontWeight','bold');
end

pSWm=g2m(pathSW); pWGm=g2m(pathWG); pAllm=g2m(pAll);
h1f=plot(pSWm(:,1),pSWm(:,2),'c--','LineWidth',1.2,'DisplayName','D*Lite S→W');
h2f=plot(pWGm(:,1),pWGm(:,2),'b--','LineWidth',1.2,'DisplayName','D*Lite W→G');
h3f=plot(pAllm(:,1),pAllm(:,2),'k-','LineWidth',2.2,'DisplayName','D*Lite + Smooth');
plot(start_m(1),start_m(2),'yo','MarkerFaceColor','y','MarkerSize',9,'HandleVisibility','off');
plot(waypoint_m(1),waypoint_m(2),'mo','MarkerFaceColor','m','MarkerSize',9,'HandleVisibility','off');
plot(goal_m(1),goal_m(2),'ro','MarkerFaceColor','r','MarkerSize',9,'HandleVisibility','off');
legend([h1f h2f h3f],'Location','bestoutside');

%% ===== FIGURE 2: ANIMASI SETUP =====
figure(2); clf; hold on; axis equal; grid on; box on;
xlabel('X [m]'); ylabel('Y [m]'); axis([0 50 0 33]);
if ENABLE_SAPF
    title('Figure 2: Trajektori USV  (SAPF: AKTIF)','Color',[0 0.5 0]);
else
    title('Figure 2: Trajektori USV  (SAPF: NON-AKTIF — USV akan kena obstacle)','Color',[0.8 0 0]);
end

for k=1:size(obstacles_static_m,1)
    cx=obstacles_static_m(k,1); cy=obstacles_static_m(k,2);
    r=obstacles_static_m(k,3); rg=r+safeDist_show;
    fill(cx+r*cos(th),cy+r*sin(th),'r','FaceAlpha',0.3,'EdgeColor','none','HandleVisibility','off');
    plot(cx+rg*cos(th),cy+rg*sin(th),'r--','LineWidth',0.8,'HandleVisibility','off');
end
hExF  = fill(NaN,NaN,'m','FaceAlpha',0.3,'EdgeColor','none','HandleVisibility','off');
hExG  = plot(NaN,NaN,'m--','LineWidth',0.8,'HandleVisibility','off');

for i=1:2
    hBF(i)=fill(NaN,NaN,[0 0.7 0.9],'FaceAlpha',0.55,'EdgeColor','b','LineWidth',1.5,'HandleVisibility','off');
    hBG(i)=plot(NaN,NaN,'b--','LineWidth',0.8,'HandleVisibility','off');
end

plot(start_m(1),start_m(2),'yo','MarkerFaceColor','y','MarkerSize',9,'DisplayName','Start');
plot(waypoint_m(1),waypoint_m(2),'mo','MarkerFaceColor','m','MarkerSize',9,'DisplayName','Waypoint');
plot(goal_m(1),goal_m(2),'ro','MarkerFaceColor','r','MarkerSize',9,'DisplayName','Goal');
hTraj = plot(NaN,NaN,'b-','LineWidth',1.8,'DisplayName','Traj');

% Kapal (diamond)
bShip = (0.9/1.8)*[1 0;-0.8 0.45;-0.4 0;-0.8 -0.45];
tK = hgtransform('Parent',gca);
patch('XData',bShip(:,1),'YData',bShip(:,2),...
    'FaceColor',[0 0.7 0],'EdgeColor','k','LineWidth',0.8,...
    'Parent',tK,'HandleVisibility','off');
% FOV kamera
angF=linspace(-cam.fov/2,cam.fov/2,30);
xFov=[0 cam.maxRange*cos(angF) 0];
yFov=[0 cam.maxRange*sin(angF) 0];
patch('XData',xFov,'YData',yFov,'Parent',tK,...
    'FaceColor',[1 1 0],'FaceAlpha',0.12,'EdgeColor',[1 0.8 0],...
    'LineWidth',0.8,'HandleVisibility','off');

hInfo = text(0.5,-1.8,'','FontSize',10,'FontWeight','bold',...
    'BackgroundColor','w','EdgeColor','k','Margin',4);
legend([hTraj],'Location','northwest','FontSize',8);

%% ===== STATE AWAL =====
x=start(1); y=start(2);
psi=atan2(pAll(min(2,size(pAll,1)),2)-start(2), pAll(min(2,size(pAll,1)),1)-start(1));
nu=[0;0;0;0]; phi=0;
pm0=g2m([x y]);
set(tK,'Matrix',makehgtform('translate',[pm0(1) pm0(2) 0],'zrotate',psi));

t=0;
xl=[]; yl=[]; psil=[]; phil=[];
xdl=[]; ydl=[]; psidl=[]; phidl=[];
tl=[]; ctel=[]; dist_dyn_l=[];
phi_des_prev=0; eInt_phi=0; Ye_int=0; Ye_prev=0;
force_track=false; rp_idx=1;
pAll_replan=[]; didReplan=false;

%% ===== MAIN SIMULATION LOOP =====
fprintf('\n=== Simulasi: SAPF=%s ===\n', mat2str(ENABLE_SAPF));
while true
    %% 1. Update posisi obstacle dinamis
    for i=1:2
        boxObs(i).pos_m = boxObs(i).pos_m + boxObs(i).vel_m*dt;
    end

    %% 2. Visualisasi obstacle kotak (meter)
    for i=1:2
        cx_b=boxObs(i).pos_m(1); cy_b=boxObs(i).pos_m(2);
        hw_b=boxObs(i).hw; hh_b=boxObs(i).hh;
        xb=[cx_b-hw_b cx_b+hw_b cx_b+hw_b cx_b-hw_b cx_b-hw_b];
        yb=[cy_b-hh_b cy_b-hh_b cy_b+hh_b cy_b+hh_b cy_b-hh_b];
        set(hBF(i),'XData',xb,'YData',yb);
        rg_b=sqrt(hw_b^2+hh_b^2)+safeDist_show;
        set(hBG(i),'XData',cx_b+rg_b*cos(th),'YData',cy_b+rg_b*sin(th));
    end

    %% 3. Waypoint check
    if ~reached_wp && hypot(waypoint(1)-x,waypoint(2)-y)<wp_tol
        reached_wp=true;
        fprintf('  Waypoint reached  t=%.2f s\n',t);
    end

    %% 4. Proyeksi ke global path
    if force_track
        ir=rp_idx; p_ref=pAll(ir,:);
        ir2=min(ir+1,size(pAll,1));
        psi_path=atan2(pAll(ir2,2)-pAll(ir,2),pAll(ir2,1)-pAll(ir,1));
        s_on=cumAll(ir); remaining=cumAll(end)-s_on;
        if norm([x y]-p_ref)<0.6, force_track=false; end
    else
        [s_on,psi_path,p_ref,~]=projectOnPath(pAll,cumAll,[x y]);
        remaining=cumAll(end)-s_on;
    end
    if ~reached_wp && s_on>s_wp-0.5
        p_ref=waypoint;
        ir2=min(idx_wp+1,size(pAll,1));
        psi_path=atan2(pAll(ir2,2)-pAll(idx_wp,2),pAll(ir2,1)-pAll(idx_wp,1));
    end

    %% 5. Goal check
    dg = hypot(goal_g(1)-x, goal_g(2)-y);
    if reached_wp && dg<goal_tol
        fprintf('  Goal reached  t=%.1f s\n',t);
        pm=g2m([x y]); xl=[xl;pm(1)]; yl=[yl;pm(2)];
        psil=[psil;psi]; phil=[phil;phi]; tl=[tl;t];
        [~,pp,ppr,~]=projectOnPath(pAll,cumAll,[x y]);
        pprm=g2m(ppr);
        xdl=[xdl;pprm(1)]; ydl=[ydl;pprm(2)];
        psidl=[psidl;pp]; phidl=[phidl;phi_des_prev];
        break;
    end

    %% 6. Kamera: deteksi obstacle kotak dinamis → SAPF input
    pos_m_now = g2m([x y]);
    obs_sapf_m = [];
    for i=1:2
        [seen_b, npt_m] = camera_detect_box(pos_m_now, psi, boxObs(i), cam);
        if seen_b
            obs_sapf_m = [obs_sapf_m; npt_m]; %#ok<AGROW>
        end
    end

    %% 7. Kamera: deteksi extra static obstacle
    need_rp = false;
    if ~extra_known
        cam_g=struct('fov',cam.fov,'maxRange',cam.maxRange/cell_m,...
                     'minRange',cam.minRange/cell_m,'sigma_r',0,'sigma_b',0);
        [seenE,~]=camera_detect(m2g(pos_m_now),psi,extraGT,cam_g);
        if seenE
            extra_known=true;
            fprintf('  Extra obs terdeteksi  t=%.2f s\n',t);
            newObs_m=extraObs_gt_m;
            newObs=[m2g(newObs_m(1:2)) newObs_m(3)/cell_m];
            obstacles_planning_m=[obstacles_planning_m;newObs_m];
            obs_plan_g=[obs_plan_g;newObs];
            obs_rt_g=[obs_rt_g;newObs];
            cxN=newObs_m(1); cyN=newObs_m(2);
            rN=newObs_m(3); rNg=rN+safeDist_show;
            set(hExF,'XData',cxN+rN*cos(th),'YData',cyN+rN*sin(th));
            set(hExG,'XData',cxN+rNg*cos(th),'YData',cyN+rNg*sin(th));
            need_rp=true;
        end
    end

    %% 8. Replan D* Lite (extra static obstacle)
    if need_rp && ~replan_done
        map_d=zeros(nR,nC);
        for kk=1:size(obstacles_planning_m,1)
            cx=obstacles_planning_m(kk,1); cy=obstacles_planning_m(kk,2);
            r=max(obstacles_planning_m(kk,3)+occInflate_m,minOcc_m);
            map_d((xcGrid-cx).^2+(ycGrid-cy).^2<=r^2)=1;
        end
        if ~reached_wp
            [pSWr,~]=dstarLite_grid(map_d,[x y],waypoint,w);
            [pWGr,~]=dstarLite_grid(map_d,waypoint,goal_g,w);
            if ~isempty(pSWr)&&~isempty(pWGr)
                pSWr(1,:)=[x y]; pSWr(end,:)=waypoint;
                pWGr(1,:)=waypoint; pWGr(end,:)=goal_g;
                pr1=smooth_path_g2cbs_c2(pSWr,nSeg,epsRDP);
                pr2=smooth_path_g2cbs_c2(pWGr,nSeg,epsRDP);
                [pr1,~]=enforce_safe_clearance(pr1,obs_plan_g,safeDistPlan_g,...
                    'maxIter',80,'gain',0.6,'maxStep',0.2,'lambda',0.15,'ds',0.25);
                [pr2,~]=enforce_safe_clearance(pr2,obs_plan_g,safeDistPlan_g,...
                    'maxIter',80,'gain',0.6,'maxStep',0.2,'lambda',0.15,'ds',0.25);
                pr1(1,:)=[x y]; pr1(end,:)=waypoint;
                pr2(1,:)=waypoint; pr2(end,:)=goal_g;
                pAll=[pr1;pr2(2:end,:)]; pAll_replan=pAll; didReplan=true;
                cumAll=cumulativeArc(pAll);
                [~,idx_wp]=min(vecnorm(pAll-waypoint,2,2));
                s_wp=cumAll(idx_wp);
                Ye_int=0; Ye_prev=0;
            end
        else
            [pNr,~]=dstarLite_grid(map_d,[x y],goal_g,w);
            if ~isempty(pNr)
                pNr(1,:)=[x y]; pNr(end,:)=goal_g;
                pSm=smooth_path_g2cbs_c2(pNr,nSeg,epsRDP);
                [pAll,~]=enforce_safe_clearance(pSm,obs_plan_g,safeDistPlan_g,...
                    'maxIter',80,'gain',0.6,'maxStep',0.2,'lambda',0.15,'ds',0.25);
                pAll(end,:)=goal_g; pAll_replan=pAll; didReplan=true;
                cumAll=cumulativeArc(pAll); Ye_int=0; Ye_prev=0;
            end
        end
        replan_done=true;
        [s_on,psi_path,p_ref,sn]=projectOnPath(pAll,cumAll,[x y]);
        rp_idx=min(sn+1,size(pAll,1)); force_track=true;
        remaining=cumAll(end)-s_on; Ye_int=0; Ye_prev=0;
    end

    %% 9. ILOS Guidance
    Ld_loc  = max(0.4, min(Ld, 0.6*remaining));
    v_ref_s = v_ref * max(0.3, min(1.0, remaining/3.0));
    [psi_des,~,Ye_int,Ye_prev]=ilos_guidance(p_ref,psi_path,[x y],Ye_int,Ye_prev,ilos_p,dt);
    if dg<4
        pg=atan2(goal_g(2)-y,goal_g(1)-x); af=min(1,(4-dg)/4);
        psi_des=atan2((1-af)*sin(psi_des)+af*sin(pg),(1-af)*cos(psi_des)+af*cos(pg));
        v_ref_s=v_ref_s*max(0.25,dg/4);
    end

    %% 10. SAPF local planner (jika aktif & kamera mendeteksi obstacle)
    sapf_active_now = false;
    if ENABLE_SAPF && ~isempty(obs_sapf_m)
        p_ref_m_s = g2m(p_ref);
        [psi_des, v_ms] = SAPF_compute(pos_m_now, psi, obs_sapf_m, p_ref_m_s, psi_des, sapf);
        v_ref_s = v_ms / cell_m;
        sapf_active_now = true;
    end

    prm=g2m(p_ref);
    xdl=[xdl;prm(1)]; ydl=[ydl;prm(2)]; psidl=[psidl;psi_des];

    obs_rt_g=obs_stat_g;
    if ~isempty(newObs), obs_rt_g=[obs_rt_g;newObs]; end
    if dg<3, obs_rt_g=[]; end

    %% 11. Controller
    Tcmd=controller_guard_4dof_ilos([x y psi],nu,psi_des,v_ref_s,Ld_loc,...
        obs_rt_g,safeDist_g,avoid,Ku,Kr_ctrl,Tmax,Nmax,r_max,Kv,Kdv,Ymax,v_sway);
    Fx=Tcmd(1); Fy=Tcmd(2); Mz=Tcmd(3);

    %% 12. Banking
    u=nu(1); vs=nu(2); r=nu(3); p=nu(4);
    pe=atan2(sin(psi_des-psi),cos(psi_des-psi));
    kap=2*sin(pe)/max(0.5,Ld); Ueff=max(0.3,hypot(u,vs));
    phi_cmd=5*atan(Ueff^2*kap/P.g)+0.3*phi;
    phi_cmd=max(-phi_max,min(phi_max,phi_cmd));
    alp=dt/(0.25+dt);
    phi_des=phi_des_prev+alp*(phi_cmd-phi_des_prev);
    phidl=[phidl;phi_des]; phi_des_prev=phi_des;
    ep=phi_des-phi; eInt_phi=eInt_phi+ep*dt;
    Tk=6*ep+0.5*eInt_phi-3*p;
    Tx=max(-lims.TX,min(lims.TX,Fx)); Ty=max(-lims.TY,min(lims.TY,Fy));
    Tn=max(-lims.TN,min(lims.TN,Mz)); Tk=max(-lims.TK,min(lims.TK,Tk));

    %% 13. Dinamika USV 4-DOF
    [Vd,etad]=usv4dof(nu,[Tx;Ty;Tn;Tk],psi,phi,P);
    nu=nu+dt*Vd;
    nu(1)=max(-2*v_ref,min(2*v_ref,nu(1)));
    nu(2)=max(-2*v_ref,min(2*v_ref,nu(2)));
    nu(3)=max(-0.7,min(0.7,nu(3)));
    nu(4)=max(-2,min(2,nu(4)));
    x=x+etad(1)*dt; y=y+etad(2)*dt;
    psi=psi+etad(3)*dt; phi=phi+etad(4)*dt;
    if any(~isfinite(nu))||~isfinite(x), error('Non-finite t=%.2fs',t); end

    %% 14. Logging
    pm=g2m([x y]);
    xl=[xl;pm(1)]; yl=[yl;pm(2)]; psil=[psil;psi]; phil=[phil;phi]; tl=[tl;t];

    % Jarak ke obstacle kotak (nearest point on box surface)
    d2=zeros(1,2);
    for i=1:2
        np=nearest_on_box(boxObs(i).pos_m,boxObs(i).hw,boxObs(i).hh,pm);
        d2(i)=norm(pm-np);
    end
    dist_dyn_l=[dist_dyn_l;d2]; %#ok<AGROW>

    [~,si]=min(vecnorm(pAll-[x y],2,2));
    si=max(2,min(si,size(pAll,1)));
    ctel=[ctel; crossTrackError([x y],pAll(si-1,:),pAll(si,:))]; %#ok<AGROW>

    t=t+dt;

    %% 15. Update figure 2
    pm_v=g2m([x y]);
    set(tK,'Matrix',makehgtform('translate',[pm_v(1) pm_v(2) 0],'zrotate',psi));
    set(hTraj,'XData',xl,'YData',yl);
    set(hInfo,'String',sprintf(' t=%5.1f s | spd=%.2f m/s | ψ=%6.1f° | SAPF_local=%s ',...
        t, hypot(nu(1),nu(2))*cell_m, rad2deg(psi), mat2str(sapf_active_now)));
    drawnow;
end

%% ===== FIGURE 3: JARAK USV KE OBSTACLE DINAMIS =====
figure(3); clf; hold on; box on; grid on;
clr_dyn = {[0 0.45 0.74], [0.85 0.33 0.10]};
for i=1:2
    imin=min(dist_dyn_l(:,i));
    plot(tl, dist_dyn_l(:,i), 'Color',clr_dyn{i}, 'LineWidth',1.8,...
        'DisplayName', sprintf('Box%d  (min=%.2f m)', i, imin));
end
yline(safeDist_m,'k--','LineWidth',1.5,...
    'Label',sprintf('Safe dist = %.1f m', safeDist_m),...
    'LabelHorizontalAlignment','right','FontWeight','bold');

% Warnai area pelanggaran
for i=1:2
    idx_v = dist_dyn_l(:,i) < safeDist_m;
    if any(idx_v)
        tv = tl(idx_v); dv = dist_dyn_l(idx_v,i);
        fill([tv;flipud(tv)],[dv;safeDist_m*ones(sum(idx_v),1)],...
            clr_dyn{i},'FaceAlpha',0.18,'EdgeColor','none','HandleVisibility','off');
        fprintf('Box%d VIOLATION: min dist = %.3f m  (aman jika > %.1f m)\n',...
            i, min(dist_dyn_l(:,i)), safeDist_m);
    end
end

xlabel('t [s]'); ylabel('Jarak [m]');
title(sprintf('Figure 3: Jarak USV ke Obstacle Dinamis (SAPF: %s)', mat2str(ENABLE_SAPF)));
legend('Location','northeast','FontSize',9);

%% ===== FIGURE 4: CTE =====
figure(4); clf;
plot(tl, abs(ctel)*cell_m, 'm-', 'LineWidth', 1.5); grid on;
xlabel('t [s]'); ylabel('|CTE| [m]');
title('Figure 4: Cross-Track Error vs Time');

%% ===== FIGURE 5: ACTUAL vs DESIRED =====
figure(5); clf; set(gcf,'Position',[150 80 950 720],'Color','w');
N=min([numel(tl),numel(xl),numel(xdl),numel(psil),numel(psidl),numel(phil),numel(phidl)]);
if N>0
    tv=tl(1:N);
    subD={xl(1:N),xdl(1:N),'X [m]';
          yl(1:N),ydl(1:N),'Y [m]';
          unwrap(psil(1:N))*180/pi, unwrap(psidl(1:N))*180/pi, '\psi [deg]';
          phil(1:N)*180/pi, phidl(1:N)*180/pi, '\phi [deg]'};
    for sp=1:4
        subplot(4,1,sp); hold on; grid on;
        plot(tv,subD{sp,1},'b','LineWidth',1.8,'DisplayName','Aktual');
        plot(tv,subD{sp,2},'r--','LineWidth',1.5,'DisplayName','Desired');
        ylabel(subD{sp,3});
        if sp==1, legend('Location','southeast','FontSize',8);
            title('Figure 5: Actual vs Desired States'); end
        if sp==4, xlabel('t [s]'); end
    end
    linkaxes(findall(gcf,'Type','axes'),'x');
end

%% ===== FIGURE 6: PATH AWAL vs SETELAH REPLAN =====
if didReplan && ~isempty(pAll_replan)
    figure(6); clf; hold on; axis equal; grid on; box on;
    xlabel('X [m]'); ylabel('Y [m]'); axis([0 50 0 33]);
    title('Figure 6: Global Path Awal vs Setelah Replan (Extra Obstacle)');
    for k=1:size(obstacles_static_m,1)
        cx=obstacles_static_m(k,1); cy=obstacles_static_m(k,2);
        r=obstacles_static_m(k,3); rg=r+safeDist_show;
        fill(cx+r*cos(th),cy+r*sin(th),'r','FaceAlpha',0.3,'EdgeColor','none','HandleVisibility','off');
        plot(cx+rg*cos(th),cy+rg*sin(th),'r--','LineWidth',0.8,'HandleVisibility','off');
    end
    cxN=extraObs_gt_m(1); cyN=extraObs_gt_m(2); rN=extraObs_gt_m(3); rNg=rN+safeDist_show;
    fill(cxN+rN*cos(th),cyN+rN*sin(th),'m','FaceAlpha',0.3,'EdgeColor','none','HandleVisibility','off');
    plot(cxN+rNg*cos(th),cyN+rNg*sin(th),'m--','LineWidth',0.8,'HandleVisibility','off');
    plot(start_m(1),start_m(2),'yo','MarkerFaceColor','y','MarkerSize',9,'HandleVisibility','off');
    plot(waypoint_m(1),waypoint_m(2),'mo','MarkerFaceColor','m','MarkerSize',9,'HandleVisibility','off');
    plot(goal_m(1),goal_m(2),'ro','MarkerFaceColor','r','MarkerSize',9,'HandleVisibility','off');
    hi=plot(g2m(pAll_init(:,1:2))*0+g2m(pAll_init),...  % workaround
        NaN,NaN,'k--','LineWidth',0,'DisplayName','dummy');
    pim=g2m(pAll_init); prm6=g2m(pAll_replan);
    hi=plot(pim(:,1),pim(:,2),'k--','LineWidth',2.0,'DisplayName','Path awal');
    hr=plot(prm6(:,1),prm6(:,2),'b-','LineWidth',2.2,'DisplayName','Path setelah replan');
    legend([hi hr],'Location','bestoutside');
end

fprintf('\n=== Simulasi selesai. Durasi=%.1f s ===\n', tl(end));

%% =====================================================================
%%                         FUNCTIONS
%% =====================================================================

%% ----- SAPF: Safe Artificial Potential Field (Szczepanski eq.12-17) -----
function [psi_des, v_ref] = SAPF_compute(pos_m, psi, nearest_pts_m, p_ref_m, psi_ilos, sapf)
% pos_m         : posisi USV [x y] meter
% psi           : heading USV [rad]
% nearest_pts_m : Nx2, titik terdekat obstacle yang terdeteksi [m]
% p_ref_m       : titik referensi ILOS dalam meter
% psi_ilos      : heading dari ILOS (fallback)
% sapf          : struct parameter

    %% Attractive force → menuju referensi ILOS (eq.3–4)
    dg = norm(p_ref_m - pos_m);
    if dg < 1e-9
        F_att = [0 0];
    elseif dg <= sapf.d_g_star
        F_att = sapf.zeta * (p_ref_m - pos_m);           % quadratic
    else
        F_att = sapf.d_g_star * sapf.zeta * (p_ref_m - pos_m) / dg;  % conic
    end

    %% Obstacle force: SAPF = R(γ) × F_rep (eq.12–17)
    F_obst = [0 0];
    for k = 1:size(nearest_pts_m,1)
        pt   = nearest_pts_m(k,:);
        v_ob = pos_m - pt;          % vektor dari obstacle ke USV
        d_Oi = norm(v_ob);

        if d_Oi < 1e-6 || d_Oi > sapf.Q_star, continue; end

        v_dir = v_ob / d_Oi;        % arah repulsive (menjauh)

        % Repulsive force magnitude — F = -∇U_rep (eq.6)
        F_rep_mag = sapf.eta * (1/d_Oi - 1/sapf.Q_star) / d_Oi^2;
        F_rep     = F_rep_mag * v_dir;

        % α: sudut dari heading USV ke arah obstacle
        psi_to_obs = atan2(pt(2)-pos_m(2), pt(1)-pos_m(1));
        alpha      = abs(wrap_angle(psi_to_obs - psi));

        % D(α): pilih CW (+1) atau CCW (-1) (eq.17)
        if alpha <= sapf.alpha_th
            D_alpha = +1;
        else
            D_alpha = -1;
        end

        % d_rel: jarak ternormalisasi (eq.16)
        if d_Oi <= sapf.d_safe
            d_rel = 0;
        elseif d_Oi >= 2*sapf.d_vort - sapf.d_safe
            d_rel = 1;
        else
            d_rel = (d_Oi - sapf.d_safe) / (2*(sapf.d_vort - sapf.d_safe));
        end

        % γ: sudut rotasi — 0 = repulsive, ±π/2 = vortex (eq.15)
        if d_rel <= 0.5
            gamma = pi * D_alpha * d_rel;
        else
            gamma = pi * D_alpha * (1 - d_rel);
        end

        % Rotation matrix R(γ) (eq.14)
        cg=cos(gamma); sg=sin(gamma);
        R_g = [cg -sg; sg cg];

        F_obst = F_obst + (R_g * F_rep')';
    end

    %% Total force & output
    F_tot  = F_att + F_obst;
    F_norm = norm(F_tot);

    if F_norm < 1e-9
        psi_des = psi_ilos;
        v_ref   = sapf.v_max * 0.20;
        return;
    end

    % Reference heading (eq.10)
    psi_des = atan2(F_tot(2), F_tot(1));

    % Reference speed — linear penalty on heading error (eq.11)
    theta_err = abs(wrap_angle(psi_des - psi));
    if theta_err >= sapf.theta_max_err
        alpha_s = 0;
    else
        alpha_s = (sapf.theta_max_err - theta_err) / sapf.theta_max_err;
    end
    v_ref = max(0.15*sapf.v_max, alpha_s*sapf.v_max);
end

%% ----- Camera: deteksi obstacle KOTAK -----
function [seen, nearest_m] = camera_detect_box(pos_m, psi, boxOb, cam)
% Kembalikan nearest visible point pada box obstacle
    seen=false; nearest_m=[];
    npt = nearest_on_box(boxOb.pos_m, boxOb.hw, boxOb.hh, pos_m);
    v   = npt - pos_m;
    d   = norm(v);
    if d < cam.minRange || d > cam.maxRange, return; end
    bearing = atan2(v(2),v(1));
    alpha   = atan2(sin(bearing-psi), cos(bearing-psi));
    if abs(alpha) > cam.fov/2, return; end
    seen=true; nearest_m=npt;
end

%% ----- Nearest point on box -----
function pt = nearest_on_box(center_m, hw, hh, query_m)
    nx = max(center_m(1)-hw, min(center_m(1)+hw, query_m(1)));
    ny = max(center_m(2)-hh, min(center_m(2)+hh, query_m(2)));
    pt = [nx, ny];
end

%% ----- wrap angle -----
function a = wrap_angle(a)
    a = mod(a+pi, 2*pi) - pi;
end

%% ----- D* Lite -----
function [path, expanded] = dstarLite_grid(map, start_xy, goal_xy, w)
    if nargin<4, w=1; end
    [nR,nC]=size(map); trav=(map==0);
    sx=min(max(round(start_xy(1)),1),nC); sy=min(max(round(start_xy(2)),1),nR);
    gx=min(max(round(goal_xy(1)),1),nC);  gy=min(max(round(goal_xy(2)),1),nR);
    trav(sy,sx)=true;
    if ~trav(gy,gx), error('Goal di obstacle.'); end
    sid=sub2ind([nR nC],sy,sx); gid=sub2ind([nR nC],gy,gx);
    INF=1e12; g=INF*ones(nR*nC,1); rhs=INF*ones(nR*nC,1);
    km=0; Uk=[]; Ui=[]; expanded=0;
    function normalizeOpen()
        if isempty(Ui)||isempty(Uk), Uk=[];Ui=[];return;end
        n=min(size(Uk,1),size(Ui,1)); Uk=Uk(1:n,:); Ui=Ui(1:n,1);
    end
    function h=H(a,b)
        [ay,ax]=ind2sub([nR nC],a); [by,bx]=ind2sub([nR nC],b);
        h=w*hypot(double(ax-bx),double(ay-by));
    end
    function K=CalcKey(s)
        K=[min(g(s),rhs(s))+H(sid,s)+km, min(g(s),rhs(s))];
    end
    function [val,idx]=minrows(A)
        [~,idx]=min(A(:,1)+1e-12*A(:,2)); val=A(idx,:);
    end
    function push(s), normalizeOpen(); Uk(end+1,:)=CalcKey(s); Ui(end+1,1)=s; end
    function [k,u,idx]=top()
        normalizeOpen();
        if isempty(Ui), k=[INF INF];u=-1;idx=[];return;end
        [k,idx]=minrows(Uk); u=Ui(idx);
    end
    function [k,u]=pop()
        [k,u,idx]=top();
        if u~=-1&&~isempty(idx), Uk(idx,:)=[]; Ui(idx)=[]; end
        normalizeOpen();
    end
    moves=[1 0;-1 0;0 1;0 -1;1 1;-1 -1;1 -1;-1 1];
    cost=[1 1 1 1 sqrt(2) sqrt(2) sqrt(2) sqrt(2)];
    function N=Succ(s)
        [y2,x2]=ind2sub([nR nC],s); N=[];
        for ii=1:8
            nx2=x2+moves(ii,1); ny2=y2+moves(ii,2);
            if nx2>=1&&ny2>=1&&nx2<=nC&&ny2<=nR&&trav(ny2,nx2)
                N(end+1,:)=[sub2ind([nR nC],ny2,nx2),cost(ii)]; end
        end
    end
    function UpdateVertex(u2)
        normalizeOpen();
        if u2~=gid
            su=Succ(u2);
            if isempty(su), rhs(u2)=INF;
            else, rhs(u2)=min(su(:,2)+g(su(:,1)));
            end
        end
        if ~isempty(Ui)
            kp=find(Ui~=u2); Ui=Ui(kp,1); Uk=Uk(kp,:);
        end
        if g(u2)~=rhs(u2), Uk(end+1,:)=CalcKey(u2); Ui(end+1,1)=u2; end
        normalizeOpen();
    end
    rhs(gid)=0; push(gid);
    function ComputeShortestPath()
        [k_old,~,~]=top(); k_s=CalcKey(sid);
        while any(k_old<k_s)||(rhs(sid)~=g(sid))
            [~,u2]=pop(); if u2==-1,break;end
            expanded=expanded+1;
            if g(u2)>rhs(u2)
                g(u2)=rhs(u2); P2=Succ(u2);
                for ii=1:size(P2,1), UpdateVertex(P2(ii,1)); end
            else
                g(u2)=INF; P2=Succ(u2); UpdateVertex(u2);
                for ii=1:size(P2,1), UpdateVertex(P2(ii,1)); end
            end
            [k_old,~,~]=top(); k_s=CalcKey(sid);
        end
    end
    ComputeShortestPath();
    if isinf(g(sid)), path=[]; return; end
    cur=sid; path=[sx sy]; maxSt=nR*nC; st=0;
    while cur~=gid&&st<maxSt
        su=Succ(cur); if isempty(su),path=[];return;end
        [~,idx2]=min(su(:,2)+g(su(:,1))); cur=su(idx2,1);
        [yy,xx]=ind2sub([nR nC],cur); path(end+1,:)=[xx yy]; st=st+1;
    end
end

%% ----- ILOS guidance -----
function [psi_des,Ye,Ye_int,Ye_prev]=ilos_guidance(p_ref,psi_path,pos,Ye_int,Ye_prev,par,dt)
    a=psi_path; xd=p_ref(1); yd=p_ref(2);
    Ye=-(pos(1)-xd)*sin(a)+(pos(2)-yd)*cos(a);
    Ye_int=Ye_int+Ye*dt; dYe=(Ye-Ye_prev)/max(dt,1e-6); Ye_prev=Ye;
    psi_des=a-atan(par.kp*Ye+par.ki*Ye_int+par.kd*dYe);
end

%% ----- Controller + clearance guard -----
function T=controller_guard_4dof_ilos(eta,Vb,psi_des,u_ref,Ld,...
        obstacles,safeDist,avoid,Ku,Kr,Tmax,Nmax,r_max,Kv,Kdv,Ymax,v_rs)
    x2=eta(1);y2=eta(2);psi2=eta(3); u2=Vb(1);v2=Vb(2);r2=Vb(3);
    dmin=inf; adir=[0 0]; r0=0;
    for kk=1:size(obstacles,1)
        cx2=obstacles(kk,1);cy2=obstacles(kk,2);rr=obstacles(kk,3);
        vv=[x2-cx2,y2-cy2]; d2=norm(vv);
        if d2<dmin, dmin=d2; adir=vv/max(d2,1e-9); r0=rr; end
    end
    clr=dmin-r0; clrs=clr-safeDist;
    yaw_av=0; u_eff=u_ref;
    if clr<avoid.buffer*safeDist
        pa=atan2(adir(2),adir(1));
        dp=atan2(sin(pa-psi2),cos(pa-psi2));
        yaw_av=avoid.yaw_gain*dp;
        if clrs<0
            sc=max(avoid.slowdownMin,clr/safeDist);
            u_eff=u_ref*max(0.05,min(1.0,sc));
        end
    end
    e_psi=atan2(sin(psi_des-psi2),cos(psi_des-psi2));
    Kpsi=1.8; KrY=22.0; Kdr=10.0;
    r_cmd=yaw_av+Kpsi*e_psi; r_cmd=max(-r_max,min(r_max,r_cmd));
    u_cmd=u_eff*max(0.2,cos(e_psi));
    Fx=Ku*(u_cmd-u2); Fx=max(-Tmax,min(Tmax,Fx));
    Mz=KrY*(r_cmd-r2)-Kdr*r2; Mz=max(-Nmax,min(Nmax,Mz));
    Fy=Kv*(v_rs-v2)-Kdv*v2; Fy=max(-Ymax,min(Ymax,Fy));
    T=[Fx Fy Mz];
end

%% ----- USV 4-DOF dynamics -----
function [Vdot,eta_dot]=usv4dof(V,U,psi,phi,Pr)
    u2=V(1);v2=V(2);r2=V(3);p2=V(4);
    Fx=U(1);Fy=U(2);
    M=[Pr.m+Pr.mx 0 0 0; 0 Pr.m+Pr.my Pr.my*Pr.alphay-Pr.my*Pr.ly 0;
       0 Pr.my*Pr.alphay Pr.Iz+Pr.Jz 0; 0 -Pr.my*Pr.ly 0 Pr.Ix+Pr.Jx];
    C=[0 -(Pr.m+Pr.my)*r2 0 0; (Pr.m+Pr.mx)*r2 0 0 0; 0 0 0 0; 0 0 -Pr.mx*Pr.lx*u2 0];
    tau_d=[Pr.Xu*u2+Pr.Xuu*abs(u2)*u2; Pr.Yv*v2+Pr.Yvv*abs(v2)*v2;
           Pr.Nr*r2+Pr.Nrr*abs(r2)*r2; Pr.Kp*p2+Pr.Kpp*abs(p2)*p2];
    tau_g=[0;0;0;-Pr.W*Pr.GMT*phi];
    tau_c=[0;0;0;Pr.m*Pr.hCG*u2*r2];
    rhs=[U(1);U(2);U(3);U(4)]+tau_d+tau_g+tau_c-C*V;
    Vdot=M\rhs;
    eta_dot=[u2*cos(psi)-v2*sin(psi); u2*sin(psi)+v2*cos(psi); r2; p2];
end

%% ----- Camera detect (circular obstacle, grid units) -----
function [seen,meas]=camera_detect(pos,psi,obsDyn,camera)
    meas=struct('dist',[],'bearing',[],'pos_est',[]);
    seen=false;
    if ~obsDyn.active, return; end
    r_vo=obsDyn.pos-pos; d=norm(r_vo);
    if d<1e-6||d<camera.minRange||d>camera.maxRange, return; end
    bw=atan2(r_vo(2),r_vo(1));
    alpha=atan2(sin(bw-psi),cos(bw-psi));
    if abs(alpha)>camera.fov/2, return; end
    d_m=d+camera.sigma_r*randn(); a_m=alpha+camera.sigma_b*randn();
    be=psi+a_m; pe=pos+d_m*[cos(be),sin(be)];
    meas.dist=d_m; meas.bearing=a_m; meas.pos_est=pe; seen=true;
end

%% ----- Path utilities -----
function cum=cumulativeArc(P)
    if size(P,1)<2, cum=0;
    else, cum=[0;cumsum(sqrt(sum(diff(P).^2,2)))]; end
end

function [s_on,psi_path,p_ref,seg]=projectOnPath(P,cum,p)
    N=size(P,1); best=inf; s_on=0; seg=1; p_ref=P(1,:);
    for i=1:N-1
        A=P(i,:);B=P(i+1,:);AB=B-A;L=dot(AB,AB);if L<1e-9,continue;end
        tt=max(0,min(1,dot(p-A,AB)/L)); proj=A+tt*AB; d=norm(p-proj);
        if d<best, best=d;seg=i;s_on=cum(i)+tt*norm(AB);p_ref=proj; end
    end
    psi_path=atan2(P(seg+1,2)-P(seg,2),P(seg+1,1)-P(seg,1));
end

function e=crossTrackError(p,p1,p2)
    A=[p2(1)-p1(1);p2(2)-p1(2)]; B=[p(1)-p1(1);p(2)-p1(2)];
    e=(A(1)*B(2)-A(2)*B(1))/max(1e-6,norm(A));
end

%% ----- G2-CBS C² Smoothing -----
function Ps=smooth_path_g2cbs_c2(P,nPerSeg,epsRDP)
    if nargin<2,nPerSeg=40;end; if nargin<3,epsRDP=0;end
    P=remove_dups(P); if size(P,1)<=2,Ps=P;return;end
    if epsRDP>0&&size(P,1)>3, P=rdp(P,epsRDP);P=remove_dups(P);if size(P,1)<=2,Ps=P;return;end;end
    N=size(P,1); t=zeros(N,1);
    for i=2:N, t(i)=t(i-1)+norm(P(i,:)-P(i-1,:));end
    if t(end)<1e-9,Ps=P(1,:);return;end
    h=diff(t);
    Mx=natural_spline_second_derivs(t,P(:,1));
    My=natural_spline_second_derivs(t,P(:,2));
    mx=zeros(N,1);my=zeros(N,1);
    sx=diff(P(:,1))./h; sy=diff(P(:,2))./h;
    mx(1)=sx(1)-h(1)*(2*Mx(1)+Mx(2))/6; my(1)=sy(1)-h(1)*(2*My(1)+My(2))/6;
    for i=2:N-1
        mx(i)=0.5*((sx(i-1)+h(i-1)*(Mx(i-1)+2*Mx(i))/6)+(sx(i)-h(i)*(2*Mx(i)+Mx(i+1))/6));
        my(i)=0.5*((sy(i-1)+h(i-1)*(My(i-1)+2*My(i))/6)+(sy(i)-h(i)*(2*My(i)+My(i+1))/6));
    end
    mx(N)=sx(end)+h(end)*(Mx(end-1)+2*Mx(end))/6;
    my(N)=sy(end)+h(end)*(My(end-1)+2*My(end))/6;
    Ps=P(1,:);
    for i=1:N-1
        hi=h(i); b0=P(i,:); b3=P(i+1,:);
        b1=b0+(hi/3)*[mx(i),my(i)]; b2=b3-(hi/3)*[mx(i+1),my(i+1)];
        tau=linspace(0,1,nPerSeg)';
        B=(1-tau).^3.*b0+3*(1-tau).^2.*tau.*b1+3*(1-tau).*tau.^2.*b2+tau.^3.*b3;
        if i>1,B=B(2:end,:);end; Ps=[Ps;B]; %#ok<AGROW>
    end
    Ps=remove_dups(Ps);
end

function M=natural_spline_second_derivs(t,y)
    N=numel(y); h=diff(t); if N<=2,M=zeros(N,1);return;end
    A=zeros(N,N);d=zeros(N,1);
    A(1,1)=1;d(1)=0;A(N,N)=1;d(N)=0;
    for i=2:N-1
        A(i,i-1)=h(i-1);A(i,i)=2*(h(i-1)+h(i));A(i,i+1)=h(i);
        d(i)=6*((y(i+1)-y(i))/h(i)-(y(i)-y(i-1))/h(i-1));
    end
    M=A\d;
end

function [Pout,info]=enforce_safe_clearance(Pin,obs,safeDist,varargin)
    ip=inputParser; ip.addParameter('maxIter',80); ip.addParameter('gain',0.6);
    ip.addParameter('maxStep',0.8); ip.addParameter('lambda',0.15); ip.addParameter('ds',1.0);
    ip.parse(varargin{:}); prm=ip.Results;
    P=resample_by_arclength(Pin,prm.ds); N=size(P,1);
    if N<=2, Pout=P; info.min_clearance=Inf; info.iterations=0; return;end
    for it=1:prm.maxIter
        dP=zeros(N,2); vio=false(N,1);
        for i=2:N-1
            pi2=P(i,:); push=[0 0];
            for k=1:size(obs,1)
                c=obs(k,1:2);R=obs(k,3)+safeDist;
                v2=pi2-c;d2=norm(v2)+1e-9;clr2=d2-R;
                if clr2<0,dir=v2/d2;push=push+prm.gain*(-clr2)*dir;vio(i)=true;end
            end
            sm=prm.lambda*((P(i-1,:)+P(i+1,:))/2-P(i,:));
            dP(i,:)=push+sm;
        end
        for i=2:N-1
            nn=norm(dP(i,:)); if nn>prm.maxStep,dP(i,:)=dP(i,:)/nn*prm.maxStep;end
        end
        P(2:N-1,:)=P(2:N-1,:)+dP(2:N-1,:);
        if ~any(vio),break;end
    end
    Pout=remove_dups(P); info.iterations=it;
    info.min_clearance=min_clearance_poly(Pout,obs,safeDist);
end

function m=min_clearance_poly(P,obs,safeDist)
    m=inf;
    for k=1:size(obs,1)
        c=obs(k,1:2);R=obs(k,3)+safeDist;
        d2=sqrt((P(:,1)-c(1)).^2+(P(:,2)-c(2)).^2)-R; m=min(m,min(d2));
    end
end

function Q=resample_by_arclength(P,ds)
    if size(P,1)<=2||ds<=0,Q=P;return;end
    s=cumulativeArc(P); ss=0:ds:s(end);
    if ss(end)<s(end),ss=[ss s(end)];end
    Q=interp1(s,P,ss,'linear');
end

function Q=rdp(P,eps2)
    if size(P,1)<=2,Q=P;return;end
    [d2,idx2]=maxPointDist(P);
    if d2>eps2
        Q1=rdp(P(1:idx2,:),eps2); Q2=rdp(P(idx2:end,:),eps2);
        Q=[Q1(1:end-1,:);Q2];
    else
        Q=[P(1,:);P(end,:)];
    end
end

function [dmax,idx2]=maxPointDist(P)
    A=P(1,:);B=P(end,:);AB=B-A;L2=max(1e-12,sum(AB.^2));
    dmax=-1;idx2=1;
    for i=2:size(P,1)-1
        AP=P(i,:)-A;tt=max(0,min(1,(AP*AB')/L2));proj=A+tt*AB;
        d2=norm(P(i,:)-proj); if d2>dmax,dmax=d2;idx2=i;end
    end
    if dmax<0,dmax=0;idx2=1;end
end

function Q=remove_dups(Q)
    if isempty(Q),return;end
    keep=[true;vecnorm(diff(Q,1,1),2,2)>1e-8]; Q=Q(keep,:);
end
