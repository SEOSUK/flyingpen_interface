%% data_decryptor_specific_v4.m
% Name-based robust reader + flight-debug dashboard for logging_specific CSV
%
% Works with:
%  - NEW logger (kDataLen=47): pose + status + control/estimator signals + kalman/comp quats + stateD
%  - Also safely works if some columns are missing (fills NaNs)
%
% Adds:
%  - Extra Figure A: 3x1 position setpoint vs pose (X panel overlays accel norm)
%  - Extra Figure B: 3x1 current attitude (pose RPY)

clear; close all; clc;

%% 0) Pick CSV
defaultDir = fullfile(getenv("HOME"), "hitl_ws", "src", "flying_pen", "bag", "logging_specific");
if ~isfolder(defaultDir), defaultDir = pwd; end

[file, path] = uigetfile(fullfile(defaultDir, "*.csv"), "Select logging_specific CSV");
if isequal(file,0)
    disp("Canceled."); return;
end
csv_path = fullfile(path, file);
fprintf("[INFO] Reading: %s\n", csv_path);

%% 1) Read table robustly
opts = detectImportOptions(csv_path, 'Delimiter', ',');
for i = 1:numel(opts.VariableTypes)
    opts.VariableTypes{i} = 'double';
end
T = readtable(csv_path, opts);

if isempty(T) || height(T) < 5
    error("CSV has too few rows.");
end

vars = string(T.Properties.VariableNames);
fprintf("[INFO] Columns: %d\n", numel(vars));

time = T{:, "t_sec"};
if all(~isfinite(time))
    error("t_sec is all NaN.");
end
time = time - time(find(isfinite(time),1,'first'));

mask = [];
if any(strcmp(vars, "validity_bitmask"))
    % stored as numeric -> cast up
    mask = uint64(T{:, "validity_bitmask"});
end

%% 2) Helpers
vars = string(T.Properties.VariableNames);
get1 = @(name) local_get1(T, vars, name);

%% 3) Load signals (name-based columns; missing cols -> NaN)
% --- Pose ---
pose_xyz = [get1("pose_x"), get1("pose_y"), get1("pose_z")];
pose_rpy = [get1("pose_roll"), get1("pose_pitch"), get1("pose_yaw")];

% --- Status (optional) ---
batt_v = get1("status_battery_voltage");   % might be NaN if column doesn't exist

% --- Setpoint ---
sp_xyzyaw = [get1("sp_x"), get1("sp_y"), get1("sp_z"), get1("sp_yaw_sp")];

% --- Debug / command ---
su_cmd = [get1("su_use_vel_mode"), get1("su_cmd_fx")];

% --- Velocity from position ---
vfp = [get1("velFromPos_vx"), get1("velFromPos_vy"), get1("velFromPos_vz")];

% --- Attitude: Kalman quat / Comp quat ---
kal_q = [get1("kal_q0"), get1("kal_q1"), get1("kal_q2"), get1("kal_q3")];
qComp = [get1("kal_qComp0"), get1("kal_qComp1"), get1("kal_qComp2"), get1("kal_qComp3")];

% --- Kalman attitude error states ---
Dstate = [get1("kal_stateD0"), get1("kal_stateD1"), get1("kal_stateD2")];

% --- stateEstimate ---
est_v = [get1("est_vx"), get1("est_vy"), get1("est_vz")];
est_a = [get1("est_ax"), get1("est_ay"), get1("est_az")];

% --- tuning signals ---
body_v = [get1("body_vx"), get1("body_vy")];
gyro   = [get1("gyro_x"), get1("gyro_y"), get1("gyro_z")];
vel_des  = [get1("velDes_vx"), get1("velDes_vy"), get1("velDes_vz")];
att_des  = [get1("attDes_roll"), get1("attDes_pitch"), get1("attDes_yaw")];
rate_des = [get1("rateDes_p"), get1("rateDes_q"), get1("rateDes_r")];

% --- ages (optional) ---
age_pose      = get1("age_pose");
age_status    = get1("age_status");
age_setpoint  = get1("age_setpoint");
age_est_a     = get1("age_stateEst_acc");
age_kal_q     = get1("age_kalman_att_q");
age_kal_qComp = get1("age_kalman_att_qComp");

%% 4) Cleanup rows (valid time only)
validTime = isfinite(time);
time = time(validTime);

pose_xyz   = pose_xyz(validTime,:);
pose_rpy   = pose_rpy(validTime,:);
batt_v     = batt_v(validTime);

sp_xyzyaw  = sp_xyzyaw(validTime,:);
su_cmd     = su_cmd(validTime,:);
vfp        = vfp(validTime,:);

kal_q      = kal_q(validTime,:);
qComp      = qComp(validTime,:);
Dstate     = Dstate(validTime,:);

est_v      = est_v(validTime,:);
est_a      = est_a(validTime,:);

body_v     = body_v(validTime,:);
gyro       = gyro(validTime,:);
vel_des    = vel_des(validTime,:);
att_des    = att_des(validTime,:);
rate_des   = rate_des(validTime,:);

age_pose      = age_pose(validTime);
age_status    = age_status(validTime);
age_setpoint  = age_setpoint(validTime);
age_est_a     = age_est_a(validTime);
age_kal_q     = age_kal_q(validTime);
age_kal_qComp = age_kal_qComp(validTime);

if ~isempty(mask), mask = mask(validTime); end

N = numel(time);
fprintf("[INFO] Using %d rows.\n", N);

%% 5) Quaternion -> RPY (Kalman & Comp)
% NOTE: quat_to_rpy_batch expects q: Nx4 [w x y z]
rpy_kal  = quat_to_rpy_batch(kal_q);
rpy_comp = quat_to_rpy_batch(qComp);

% unwrap yaw for readability
rpy_kal(:,3)  = unwrap(rpy_kal(:,3));
rpy_comp(:,3) = unwrap(rpy_comp(:,3));

% rpy error (kal - comp)
rpy_err = rpy_kal - rpy_comp;
rpy_err(:,3) = wrapToPi(rpy_err(:,3)); % yaw diff wrapped

%% 6) Derived norms
D_norm = vecnorm(Dstate,2,2);
v_norm = vecnorm(est_v,2,2);
a_norm = vecnorm(est_a,2,2);

gyro_norm = vecnorm(gyro,2,2);
rate_des_norm = vecnorm(rate_des,2,2);

%% 7) Highlight logic (robust; works even if optional cols missing)
% New logger(47-cols) has no contact_diag/acc_ext/mode_flags.
% We shade "event-like" segments using:
%  - ||stateD|| large
%  - ||a|| large
%  - ||gyro|| large

validD = D_norm(isfinite(D_norm));
if isempty(validD), th_D = 0.2; else, th_D = max(0.2, prctile(validD, 90)); end

validA = a_norm(isfinite(a_norm));
if isempty(validA), th_a = 12.0; else, th_a = max(12.0, prctile(validA, 90)); end

validG = gyro_norm(isfinite(gyro_norm));
if isempty(validG), th_g = 1.0; else, th_g = max(1.0, prctile(validG, 90)); end

is_D_big = D_norm > th_D;
is_a_big = a_norm > th_a;
is_g_big = gyro_norm > th_g;

hi = is_D_big | is_a_big | is_g_big;

fprintf("[HILITE] th_D=%.3f, th_a=%.3f, th_g=%.3f\n", th_D, th_a, th_g);

%% 8) Plot dashboard (panel layout for flight debugging)
set(groot,'defaultFigureRenderer','painters');

% default window selection
twin = [30 70];
if (time(end)-time(1)) > 120
    twin = [max(time(1), time(1)+5) min(time(end), time(1)+120)];
else
    twin = [max(time(1), twin(1)) min(time(end), twin(2))];
end

f = figure('Name','Flight Debug Dashboard (Kalman vs Comp + Control/Estimator)','NumberTitle','off', ...
           'Color','w','Units','normalized','Position',[0 0 1 1]);

% 6 rows x 3 cols (18 panels)
tl = tiledlayout(f, 6, 3, 'TileSpacing','compact', 'Padding','compact');
axlist = gobjects(0);

% ---------------- Row 1: Attitude (kal vs comp) ----------------
ax = nexttile(tl,1); axlist(end+1)=ax;
plot(time, rpy_kal(:,1),'LineWidth',1.2); hold on;
plot(time, rpy_comp(:,1),'LineWidth',1.0);
grid on; xlim(twin);
title('Roll [rad] (kal vs comp)'); legend('kal','comp','Location','best');

ax = nexttile(tl,2); axlist(end+1)=ax;
plot(time, rpy_kal(:,2),'LineWidth',1.2); hold on;
plot(time, rpy_comp(:,2),'LineWidth',1.0);
grid on; xlim(twin);
title('Pitch [rad] (kal vs comp)'); legend('kal','comp','Location','best');

ax = nexttile(tl,3); axlist(end+1)=ax;
plot(time, rpy_kal(:,3),'LineWidth',1.2); hold on;
plot(time, rpy_comp(:,3),'LineWidth',1.0);
grid on; xlim(twin);
title('Yaw [rad] (kal vs comp)'); legend('kal','comp','Location','best');

% ---------------- Row 2: Attitude error (kal-comp) ----------------
ax = nexttile(tl,4); axlist(end+1)=ax;
plot(time, rpy_err(:,1),'LineWidth',1.2); grid on; xlim(twin);
title('Roll error (kal-comp) [rad]');

ax = nexttile(tl,5); axlist(end+1)=ax;
plot(time, rpy_err(:,2),'LineWidth',1.2); grid on; xlim(twin);
title('Pitch error (kal-comp) [rad]');

ax = nexttile(tl,6); axlist(end+1)=ax;
plot(time, rpy_err(:,3),'LineWidth',1.2); grid on; xlim(twin);
title('Yaw error (kal-comp) [rad]');

% ---------------- Row 3: Estimator core ----------------
ax = nexttile(tl,7); axlist(end+1)=ax;
plot(time, D_norm,'LineWidth',1.2); grid on; xlim(twin);
title('||stateD||');

ax = nexttile(tl,8); axlist(end+1)=ax;
plot(time, a_norm,'LineWidth',1.2); hold on;
yline(th_a,'--');
grid on; xlim(twin);
title(sprintf('||a|| (th=%.2f)', th_a));

ax = nexttile(tl,9); axlist(end+1)=ax;
plot(time, gyro_norm,'LineWidth',1.2); hold on;
yline(th_g,'--');
grid on; xlim(twin);
title(sprintf('||gyro|| (th=%.2f)', th_g));

% ---------------- Row 4: Ops / commands ----------------
ax = nexttile(tl,10); axlist(end+1)=ax;
plot(time, rate_des_norm,'LineWidth',1.2);
grid on; xlim(twin);
title('||rate\_des||');

ax = nexttile(tl,11); axlist(end+1)=ax;
plot(time, batt_v,'LineWidth',1.2);
grid on; xlim(twin);
title('battery voltage [V]');

ax = nexttile(tl,12); axlist(end+1)=ax;
plot(time, su_cmd(:,2),'LineWidth',1.2);
grid on; xlim(twin);
title('su\_cmd\_fx');

% ---------------- Row 5: vel tracking (des vs est) ----------------
ax = nexttile(tl,13); axlist(end+1)=ax;
plot(time, vel_des(:,1),'LineWidth',1.2); hold on;
plot(time, est_v(:,1),'LineWidth',1.0);
plot(time, vfp(:,1),'LineWidth',1.0);
grid on; xlim(twin);
title('Vx: des vs est vs vfp'); legend('des','est','vfp','Location','best');

ax = nexttile(tl,14); axlist(end+1)=ax;
plot(time, vel_des(:,2),'LineWidth',1.2); hold on;
plot(time, est_v(:,2),'LineWidth',1.0);
plot(time, vfp(:,2),'LineWidth',1.0);
grid on; xlim(twin);
title('Vy: des vs est vs vfp'); legend('des','est','vfp','Location','best');

ax = nexttile(tl,15); axlist(end+1)=ax;
plot(time, vel_des(:,3),'LineWidth',1.2); hold on;
plot(time, est_v(:,3),'LineWidth',1.0);
plot(time, vfp(:,3),'LineWidth',1.0);
grid on; xlim(twin);
title('Vz: des vs est vs vfp'); legend('des','est','vfp','Location','best');

% ---------------- Row 6: controller internals ----------------
ax = nexttile(tl,16); axlist(end+1)=ax;
plot(time, body_v(:,1),'LineWidth',1.2); hold on;
plot(time, body_v(:,2),'LineWidth',1.2);
grid on; xlim(twin);
title('state\_body\_vel (VX,VY)'); legend('bodyVX','bodyVY','Location','best'); xlabel('time [s]');

ax = nexttile(tl,17); axlist(end+1)=ax;
plot(time, att_des(:,1),'LineWidth',1.0); hold on;
plot(time, att_des(:,2),'LineWidth',1.0);
plot(time, att_des(:,3),'LineWidth',1.0);
grid on; xlim(twin);
title('att\_des (r,p,y)'); legend('roll','pitch','yaw','Location','best'); xlabel('time [s]');

ax = nexttile(tl,18); axlist(end+1)=ax;
plot(time, rate_des(:,1),'LineWidth',1.0); hold on;
plot(time, rate_des(:,2),'LineWidth',1.0);
plot(time, rate_des(:,3),'LineWidth',1.0);
grid on; xlim(twin);
title('rate\_des (p,q,r)'); legend('p','q','r','Location','best'); xlabel('time [s]');

% Link axes & highlight
linkaxes(axlist,'x');
add_highlight(axlist, time, hi);

% Overlay info text
try
    txt = sprintf("Shaded = event-like (stateD/acc/gyro high)");
    annotation(f,'textbox',[0.01 0.965 0.8 0.03],'String',txt,'EdgeColor','none');
end

%% 9) Quick prints
fprintf("\n[CHECK]\n");
fprintf("  kal_q finite rows: %d, qComp finite rows: %d\n", any(all(isfinite(kal_q),2)), any(all(isfinite(qComp),2)));
fprintf("  batt finite: %d\n", any(isfinite(batt_v)));
fprintf("  shaded ratio: %.2f %%\n", 100.0*mean(hi(isfinite(hi))));


%% 11) Extra Figure B: Current attitude (pose RPY) (3x1)
f3 = figure('Name','Current Attitude (pose RPY)','NumberTitle','off', ...
            'Color','w','Units','normalized','Position',[0.08 0.08 0.9 0.85]);

subplot(3,1,1);
plot(time, pose_rpy(:,1),'LineWidth',1.2);
grid on; xlim(twin); ylabel('roll [rad]'); title('pose roll');

subplot(3,1,2);
plot(time, pose_rpy(:,2),'LineWidth',1.2);
grid on; xlim(twin); ylabel('pitch [rad]'); title('pose pitch');

subplot(3,1,3);
plot(time, pose_rpy(:,3),'LineWidth',1.2);
grid on; xlim(twin); ylabel('yaw [rad]'); xlabel('time [s]'); title('pose yaw');

%% ===================== local helper functions =====================
function v = local_get1(T, vars, name)
    name = string(name);
    if any(strcmp(vars, name))
        v = T{:, name};
    else
        v = nan(height(T),1);
    end
end

function rpy = quat_to_rpy_batch(q)
% q: Nx4 [w x y z]
    N = size(q,1);
    rpy = nan(N,3);
    for i = 1:N
        q0=q(i,1); q1=q(i,2); q2=q(i,3); q3=q(i,4);
        if ~all(isfinite([q0 q1 q2 q3])), continue; end
        nrm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        if nrm < 1e-9, continue; end
        q0=q0/nrm; q1=q1/nrm; q2=q2/nrm; q3=q3/nrm;

        sinr = 2*(q0*q1 + q2*q3);
        cosr = 1 - 2*(q1*q1 + q2*q2);
        roll = atan2(sinr, cosr);

        sinp = 2*(q0*q2 - q3*q1);
        if abs(sinp) >= 1
            pitch = sign(sinp)*pi/2;
        else
            pitch = asin(sinp);
        end

        siny = 2*(q0*q3 + q1*q2);
        cosy = 1 - 2*(q2*q2 + q3*q3);
        yaw = atan2(siny, cosy);

        rpy(i,:) = [roll pitch yaw];
    end
end

function add_highlight(axlist, t, flag)
% shade contiguous segments where flag==true
    flag = flag(:) > 0.5;
    t = t(:);
    if numel(t) ~= numel(flag), return; end

    d = diff([false; flag; false]);
    starts = find(d == 1);
    ends   = find(d == -1) - 1;

    for si = 1:numel(starts)
        i0 = starts(si); i1 = ends(si);
        t0 = t(i0); t1 = t(i1);
        for k = 1:numel(axlist)
            ax = axlist(k);
            yl = ylim(ax);
            patch(ax, [t0 t1 t1 t0], [yl(1) yl(1) yl(2) yl(2)], 'k', ...
                  'FaceAlpha', 0.05, 'EdgeColor', 'none', 'HandleVisibility','off');
        end
    end
end


%% 10) Extra Figure A: Position cmd vs current pose (2x1) + acc norm
f2 = figure('Name','Position: setpoint vs pose (+ acc norm)','NumberTitle','off', ...
            'Color','w','Units','normalized','Position',[0.05 0.05 0.9 0.85]);

twin = [35 72];
% ---------------- subplot 1: position + acc norm ----------------
subplot(2,1,1);

plot(time, sp_xyzyaw(:,1)-0.8, 'r--', 'LineWidth', 1.4); hold on;   % cmd: red dashed
plot(time, pose_xyz(:,1)-0.8, 'b-',  'LineWidth', 1.2);           % pose: blue solid
plot(time, a_norm,        'k-',  'LineWidth', 2.0);           % acc norm: black solid
ylim([-0.2 1])
grid on; 
xlim(twin);
ylabel('x [m]');
title('X: setpoint vs pose + |acc|');
legend('p_{cmd,x}','p_x','|acc|','Location','best');

% ---------------- subplot 2: attitude (pitch) ----------------
subplot(2,1,2);

plot(time, pose_rpy(:,2), 'b-', 'LineWidth', 1.2);  % pitch: blue solid
ylim([-0.2 0.1])
grid on; 
xlim(twin);
ylabel('pitch [rad]');
xlabel('time [s]');
title('Pitch attitude');
legend('pitch attitude','Location','best');
