%% data_decryptor.m
% Robust reader + plotting for the logging CSV.
%
% Supported formats:
%
% [NEW format v5]  (acc_kalman added)
%   col1: t_sec
%   col2..col68: d0..d66 (67 payload)
%   col69: validity_bitmask
%
% [NEW format v4]  (kalman_decouple_var added)
%   col1: t_sec
%   col2..col65: d0..d63 (64 payload)
%   col66: validity_bitmask
%
% [NEW format v3]  (flow_predN/flow_measN added)
%   col1: t_sec
%   col2..col63: d0..d61 (62 payload)
%   col64: validity_bitmask
%
% [NEW format v2]  (state_body_vel added)
%   col1: t_sec
%   col2..col59: d0..d57 (58 payload)
%   col60: validity_bitmask
%
% [NEW format v1]
%   col1: t_sec
%   col2..col57: d0..d55 (56 payload)
%   col58: validity_bitmask
%
% [LEGACY]
%   heuristic (>=44 payload)

clear; close all; clc;

%% 0) Pick CSV
defaultDir = fullfile(getenv("HOME"), "hitl_ws", "src", "flying_pen", "bag", "logging");
if ~isfolder(defaultDir), defaultDir = pwd; end

[file, path] = uigetfile(fullfile(defaultDir, "*.csv"), "Select logging CSV");
if isequal(file,0), disp("Canceled."); return; end
csv_path = fullfile(path, file);
fprintf("[INFO] Reading: %s\n", csv_path);

%% 1) Read CSV
opts = detectImportOptions(csv_path, 'Delimiter', ',');
opts = setvartype(opts, 'double');
T = readtable(csv_path, opts);
A = table2array(T);

ncol = size(A,2);
fprintf("[INFO] Parsed CSV: %d cols\n", ncol);

%% 2) Detect format
time = []; values = []; mask = [];

% ---------- NEW v5 ----------
% 1 (t) + 67 payload + 1 mask = 69 cols
if ncol >= 69
    payload = A(:,2:68);
    if size(payload,2)==67 && mean(isfinite(payload(:)))>0.01
        time   = A(:,1);
        values = payload;
        mask   = uint32(A(:,69));
        fprintf("[INFO] Detected NEW v5 (67 payload)\n");
    end
end

% ---------- NEW v4 ----------
if isempty(time) && ncol >= 66
    payload = A(:,2:65);
    if size(payload,2)==64 && mean(isfinite(payload(:)))>0.01
        time   = A(:,1);
        values = payload;
        mask   = uint32(A(:,66));
        fprintf("[INFO] Detected NEW v4 (64 payload)\n");
    end
end

% ---------- NEW v3 ----------
if isempty(time) && ncol >= 64
    payload = A(:,2:63);
    if size(payload,2)==62 && mean(isfinite(payload(:)))>0.01
        time   = A(:,1);
        values = payload;
        mask   = uint32(A(:,64));
        fprintf("[INFO] Detected NEW v3 (62 payload)\n");
    end
end

% ---------- NEW v2 ----------
if isempty(time) && ncol >= 60
    payload = A(:,2:59);
    if size(payload,2)==58 && mean(isfinite(payload(:)))>0.01
        time   = A(:,1);
        values = payload;
        mask   = uint32(A(:,60));
        fprintf("[INFO] Detected NEW v2 (58 payload)\n");
    end
end

% ---------- NEW v1 ----------
if isempty(time) && ncol >= 58
    payload = A(:,2:57);
    if size(payload,2)==56 && mean(isfinite(payload(:)))>0.01
        time   = A(:,1);
        values = payload;
        mask   = uint32(A(:,58));
        fprintf("[INFO] Detected NEW v1 (56 payload)\n");
    end
end

% ---------- Legacy ----------
if isempty(time)
    time = A(:,1);
    if nanmedian(time)>1e6, time=time*1e-9; end
    values = A(:,3:46); % legacy heuristic: first 44 payload columns
    mask   = uint32(ones(size(values,1),1));
    fprintf("[INFO] Detected LEGACY format\n");
end

%% 3) Cleanup
ok = isfinite(time);
time=time(ok); values=values(ok,:); mask=mask(ok);
time=time-time(1);
N=size(values,1);

%% 4) Decode base payload (0~43) => values(:,1:44)
force_global_raw    = values(:,1:3);
position_meas       = values(:,4:6);
rpy_meas            = values(:,7:9);
acc_global          = values(:,10:12);
vel_global          = values(:,13:15);
vbat                = values(:,16:17);
cmd_position        = values(:,18:20);
cmd_yaw_deg         = values(:,21);
force_global_scaled = values(:,22:24);
final_cmd_position  = values(:,25:27);
final_cmd_yaw_deg   = values(:,28);
cmd_x_force         = values(:,29);
world_Fext_MOB      = values(:,30:32);
world_Fext_DOB      = values(:,33:35);
vel_from_pos        = values(:,36:38);
rpy_kalman          = values(:,39:41);
rpy_comp            = values(:,42:44);

%% 5) Extended payload (version-aware decode)
payload_cols = size(values,2);

gyro_fb            = nan(N,3);
state_body_vel     = nan(N,2);
vel_des            = nan(N,3);
att_des_deg        = nan(N,3);
rate_des           = nan(N,3);
flow_predN         = nan(N,2);
flow_measN         = nan(N,2);
aNormF             = nan(N,1);
alphaDecouple      = nan(N,1);
acc_kalman         = nan(N,3); % NEW v5

% v1: 56 payload (44 base + 12)
if payload_cols >= 56
    gyro_fb     = values(:,45:47);
    vel_des     = values(:,48:50);
    att_des_deg = values(:,51:53);
    rate_des    = values(:,54:56);
end

% v2: 58 payload (44 base + 14): adds state_body_vel(2) and shifts others
if payload_cols >= 58
    gyro_fb        = values(:,45:47);
    state_body_vel = values(:,48:49);
    vel_des        = values(:,50:52);
    att_des_deg    = values(:,53:55);
    rate_des       = values(:,56:58);
end

% v3: 62 payload: adds flow_predN(2), flow_measN(2)
if payload_cols >= 62
    flow_predN = values(:,59:60);
    flow_measN = values(:,61:62);
end

% v4: 64 payload: adds aNormF, alphaDecouple
if payload_cols >= 64
    aNormF        = values(:,63);
    alphaDecouple = values(:,64);
end

% v5: 67 payload: adds acc_kalman (accX_ext, accY_ext, accZ_ext)
if payload_cols >= 67
    acc_kalman = values(:,65:67);
end

att_des_rad       = att_des_deg*pi/180;
final_cmd_yaw_rad = final_cmd_yaw_deg*pi/180;

%% ---- Dashboard figure (desired vs actual overlaid) ----
twin = [25 50];

f = figure('Name','DataLogging Overview (desired vs actual overlays)','NumberTitle','off', ...
           'Color','w','Units','normalized','Position',[0 0 1 1]);

tl = tiledlayout(f, 7, 2, 'TileSpacing','compact', 'Padding','compact');

haveVelDes = any(isfinite(vel_des(:)));
haveAttDes = any(isfinite(att_des_rad(:)));
haveRate   = any(isfinite(gyro_fb(:))) && any(isfinite(rate_des(:)));
haveBodyV  = any(isfinite(state_body_vel(:)));
haveFlow   = any(isfinite(flow_measN(:))) && any(isfinite(flow_predN(:)));

% -------------------------
% LEFT TOP: Position xyz (desired vs actual)
% -------------------------
nexttile(tl, 1);
plot(time, final_cmd_position(:,1), 'r--','LineWidth',1.2); hold on;
plot(time, position_meas(:,1),      'b-','LineWidth',1.2);
grid on; xlim(twin);
title('Pos X (target vs meas)');
legend('target','meas','Location','best');

nexttile(tl, 3);
plot(time, final_cmd_position(:,2), 'r--','LineWidth',1.2); hold on;
plot(time, position_meas(:,2),      'b-','LineWidth',1.2);
grid on; xlim(twin);
title('Pos Y (target vs meas)');

nexttile(tl, 5);
plot(time, final_cmd_position(:,3), 'r--','LineWidth',1.2); hold on;
plot(time, position_meas(:,3),      'b-','LineWidth',1.2);
grid on; xlim(twin);
title('Pos Z (target vs meas)');
xlabel('time [s]');

% -------------------------
% LEFT MID: Velocity (desired vs actual)
% -------------------------
nexttile(tl, 7);
if haveBodyV
    plot(time, state_body_vel(:,1), 'b-','LineWidth',1.2); hold on;
else
    plot(time, vel_global(:,1), 'b-','LineWidth',1.2); hold on;
end
if haveVelDes, plot(time, vel_des(:,1), 'r--','LineWidth',1.2); end
grid on; xlim(twin);
title('Vel X (body actual vs targetVX)');
if haveVelDes, legend('actual','desired','Location','best'); end

nexttile(tl, 9);
if haveBodyV
    plot(time, state_body_vel(:,2), 'b-','LineWidth',1.2); hold on;
else
    plot(time, vel_global(:,2), 'b-','LineWidth',1.2); hold on;
end
if haveVelDes, plot(time, vel_des(:,2), 'r--','LineWidth',1.2); end
grid on; xlim(twin);
title('Vel Y (body actual vs targetVY)');

nexttile(tl, 11);
plot(time, vel_global(:,3), 'b-','LineWidth',1.2); hold on;
if haveVelDes, plot(time, vel_des(:,3), 'r--','LineWidth',1.2); end
grid on; xlim(twin);
title('Vel Z (actual vz vs targetVZ)');
xlabel('time [s]');

% -------------------------
% RIGHT TOP: Attitude rpy (desired vs actual)
% -------------------------
nexttile(tl, 2);
plot(time, rpy_meas(:,1), 'b-','LineWidth',1.2); hold on;
if haveAttDes, plot(time, att_des_rad(:,1), 'r--','LineWidth',1.2); end
grid on; xlim(twin);
title('Roll (meas vs des)');
if haveAttDes, legend('meas','des','Location','best'); end

nexttile(tl, 4);
plot(time, -rpy_meas(:,2), 'b-','LineWidth',1.2); hold on;
if haveAttDes, plot(time, att_des_rad(:,2), 'r--','LineWidth',1.2); end
grid on; xlim(twin);
title('Pitch (meas vs des)');

nexttile(tl, 6);
plot(time, rpy_meas(:,3), 'b-','LineWidth',1.2); hold on;
if haveAttDes, plot(time, att_des_rad(:,3), 'r--','LineWidth',1.2); end
plot(time, final_cmd_yaw_rad, 'k-','LineWidth',1.0);
grid on; xlim(twin);
title('Yaw (meas vs des vs cmd)');
if haveAttDes
    legend('meas','des','cmd','Location','best');
else
    legend('meas','cmd','Location','best');
end
xlabel('time [s]');

% -------------------------
% RIGHT MID: Rate (desired vs actual)
% -------------------------
nexttile(tl, 8);
if haveRate
    plot(time, gyro_fb(:,1),  'b-','LineWidth',1.2); hold on;
    plot(time, rate_des(:,1), 'r--','LineWidth',1.2);
    legend('actual','des','Location','best');
else
    plot(time, nan(size(time)), 'LineWidth',1.2);
end
grid on; xlim(twin);
title('Rate X (gyro vs des)');

nexttile(tl, 10);
if haveRate
    plot(time, gyro_fb(:,2),  'b-','LineWidth',1.2); hold on;
    plot(time, -rate_des(:,2), 'r--','LineWidth',1.2);
else
    plot(time, nan(size(time)), 'LineWidth',1.2);
end
grid on; xlim(twin);
title('Rate Y (gyro vs des)');

nexttile(tl, 12);
if haveRate
    plot(time, gyro_fb(:,3),  'b-','LineWidth',1.2); hold on;
    plot(time, rate_des(:,3), 'r--','LineWidth',1.2);
else
    plot(time, nan(size(time)), 'LineWidth',1.2);
end
grid on; xlim(twin);
title('Rate Z (gyro vs des)');
xlabel('time [s]');

% -------------------------
% Flow pred vs meas (NX/NY)
% -------------------------
nexttile(tl, 13);
if haveFlow
    plot(time, flow_measN(:,1), 'b-','LineWidth',1.2); hold on;
    plot(time, flow_predN(:,1), 'r--','LineWidth',1.2);
    legend('measNX','predNX','Location','best');
else
    plot(time, nan(size(time)), 'LineWidth',1.2);
end
grid on; xlim(twin);
title('Flow N-X (meas vs pred)');

nexttile(tl, 14);
if haveFlow
    plot(time, flow_measN(:,2), 'b-','LineWidth',1.2); hold on;
    plot(time, flow_predN(:,2), 'r--','LineWidth',1.2);
    legend('measNY','predNY','Location','best');
else
    plot(time, nan(size(time)), 'LineWidth',1.2);
end
grid on; xlim(twin);
title('Flow N-Y (meas vs pred)');
xlabel('time [s]');

set(findall(f,'Type','axes'),'FontSize',9,'Color','w');

disp("[DONE] Dashboard generated (pos/vel/att/rate/flow overlays).");

% ---- quick sanity print ----
if haveVelDes
    disp("[CHECK] vel_des is interpreted as posCtl.targetVX/VY/VZ (desired).");
end
if haveBodyV
    disp("[CHECK] state_body_vel is interpreted as posCtl.bodyVX/bodyVY (actual body yaw-aligned).");
else
    disp("[WARN] state_body_vel not found (likely old CSV). Vel X/Y plots use global vx/vy fallback.");
end
if haveFlow
    disp("[CHECK] flow_measN/predN are interpreted as kalman_pred.measN*, kalman_pred.predN*.");
else
    disp("[WARN] flow_measN/predN not found (likely old CSV).");
end

%% 11. acceleration global only (detailed)
figure('Name', 'Acceleration (world) detail', 'NumberTitle', 'off', 'Color', 'w');

subplot(3,1,1);
plot(time, acc_global(:,1), 'LineWidth', 1.4);
grid on; title('acc X_{world} [G]');

subplot(3,1,2);
plot(time, acc_global(:,2), 'LineWidth', 1.4);
grid on; title('acc Y_{world} [G]');

subplot(3,1,3);
plot(time, acc_global(:,3), 'LineWidth', 1.4);
grid on; title('acc Z_{world} [G]');
xlabel('time [s]');

disp("[DONE] Plots generated successfully.");

%% Interaction (5 panels)
figure('Name', 'Interaction (5 panels)', 'NumberTitle', 'off', 'Color', 'w');

subplot(4,1,1);
plot(time, final_cmd_position(:,1), 'k-', 'LineWidth', 1.4); hold on; grid on;
plot(time, position_meas(:,1), 'b-', 'LineWidth', 1.2);
plot(time, aNormF, 'r-', 'LineWidth', 1.2);
plot(time, alphaDecouple, 'g-', 'LineWidth', 1.2);
legend('cmd pos', 'pos', 'aNormF', 'alphaDecouple');

subplot(4,1,2);
plot(time, rpy_meas(:,2), '-', 'LineWidth', 1.4); hold on; grid on;
plot(time, rpy_comp(:,2), '-', 'LineWidth', 1.6);
legend('meas (pose->rpy)', 'comp (qComp->rpy)', 'Location', 'best');

subplot(4,1,3);
plot(time, acc_global(:,1), 'LineWidth', 1.4);
grid on; title('acc [G, 9.81 m/s^2]');
legend('acceleration x');

subplot(4,1,4);
plot(time, vel_from_pos(:,1), 'LineWidth', 1.4); grid on; title('velocity [m/s]');
hold on;
plot(time, vel_global(:,1), 'LineWidth', 1.4);
legend('velocity (position diff)', 'velocity (EKF, from Firmware)');

%% 12) Acc norm + aNormF compare (and NEW kalman acc debug)
figure('Name', 'Acceleration norm debug (acc_global vs aNormF vs acc_kalman)', ...
       'NumberTitle', 'off', 'Color', 'w');

% ---------- norms ----------
acc_global_norm = sqrt( ...
    acc_global(:,1).^2 + ...
    acc_global(:,2).^2 + ...
    acc_global(:,3).^2 );

acc_kalman_norm = sqrt( ...
    acc_kalman(:,1).^2 + ...
    acc_kalman(:,2).^2 + ...
    acc_kalman(:,3).^2 );

subplot(4,1,1);
plot(time, acc_global_norm, 'k', 'LineWidth', 1.6); hold on;
plot(time, alphaDecouple, 'g-', 'LineWidth', 1.2);
plot(time, aNormF, 'r-', 'LineWidth', 1.2);
grid on; title('||acc_{global}|| [G] vs aNormF');
legend('norm(acc_global)','aNormF','Location','best');
xlim(twin);

subplot(4,1,2);
plot(time, acc_global(:,1), 'LineWidth', 1.2); hold on;
plot(time, acc_global(:,2), 'LineWidth', 1.2);
plot(time, acc_global(:,3), 'LineWidth', 1.2);
grid on; title('acc_global axes [G]');
legend('ax','ay','az','Location','best');
xlim(twin);

subplot(4,1,3);
if any(isfinite(acc_kalman(:)))
    plot(time, acc_kalman(:,1), 'LineWidth', 1.2); hold on;
    plot(time, acc_kalman(:,2), 'LineWidth', 1.2);
    plot(time, acc_kalman(:,3), 'LineWidth', 1.2);
    grid on; title('acc_kalman (kalman.acc*_ext) axes [G]');
    legend('accX\_ext','accY\_ext','accZ\_ext','Location','best');
else
    plot(time, nan(size(time)), 'LineWidth', 1.2);
    grid on; title('acc_kalman not found (need v5 payload)');
end
xlim(twin);

subplot(4,1,4);
if any(isfinite(acc_kalman(:)))
    plot(time, acc_kalman_norm, 'LineWidth', 1.6); hold on;
    plot(time, aNormF, 'r-', 'LineWidth', 1.2);
    grid on; title('||acc_kalman|| [G] vs aNormF');
    legend('norm(acc_kalman)','aNormF','Location','best');
else
    plot(time, aNormF, 'r-', 'LineWidth', 1.2);
    grid on; title('aNormF only');
end
xlabel('time [s]');
xlim(twin);

disp("[DONE] Acc norm debug plot generated (global vs kalman_ext vs aNormF).");

%% 11. Position
figure('Name', 'Position (world) detail', 'NumberTitle', 'off', 'Color', 'w');

subplot(3,1,1);
plot(time, final_cmd_position(:,1), 'r--','LineWidth',1.2); hold on;
plot(time, position_meas(:,1),      'b-','LineWidth',1.2);
grid on; title('Pos X_{world} [G]');

subplot(3,1,2);
plot(time, final_cmd_position(:,2), 'r--','LineWidth',1.2); hold on;
plot(time, position_meas(:,2),      'b-','LineWidth',1.2);
grid on; title('Pos Y_{world} [G]');

subplot(3,1,3);
plot(time, final_cmd_position(:,3), 'r--','LineWidth',1.2); hold on;
plot(time, position_meas(:,3),      'b-','LineWidth',1.2);
grid on; title('Pos Z_{world} [G]');
xlabel('time [s]');

disp("[DONE] Plots generated successfully.");