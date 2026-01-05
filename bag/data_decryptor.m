%% data_decryptor.m
% Robust reader + plotting for the logging CSV.
% Supports formats:
%
% [NEW format - UPDATED v1]
%   col1: t_sec [s]
%   col2..col57: d0..d55 (56 doubles)
%   col58: validity_bitmask
%
% [NEW format - UPDATED v2]  (state_body_vel added)
%   col1: t_sec [s]
%   col2..col59: d0..d57 (58 doubles)
%   col60: validity_bitmask
%
% [OLD/LEGACY format - heuristic]
%   col1: time_ns or time_sec
%   payload starts at col3 (values = data(:,3:end)) and should have >=44 cols
%   (for legacy, we only take first 44 as before)

clear; close all; clc;

%% 0) Pick CSV
defaultDir = fullfile(getenv("HOME"), "hitl_ws", "src", "flying_pen", "bag", "logging");
if ~isfolder(defaultDir)
    defaultDir = pwd;
end

[file, path] = uigetfile(fullfile(defaultDir, "*.csv"), "Select logging CSV");
if isequal(file,0)
    disp("Canceled.");
    return;
end
csv_path = fullfile(path, file);
fprintf("[INFO] Reading: %s\n", csv_path);

%% 1) Read CSV robustly
opts = detectImportOptions(csv_path, 'Delimiter', ',');
opts = setvartype(opts, 'double');  % Force numeric when possible (text -> NaN)
T = readtable(csv_path, opts);

if isempty(T) || height(T) == 0
    error("CSV has no data rows (header-only or empty file). Check logger is writing data.");
end

A = table2array(T);
if isempty(A) || size(A,1) == 0
    error("Parsed array is empty. CSV may be malformed or non-numeric.");
end

ncol = size(A,2);
nrow = size(A,1);
fprintf("[INFO] Parsed table: %d rows x %d cols\n", nrow, ncol);

%% 2) Detect format and extract (time, values, mask)
time = [];
values = [];
mask = [];

% ---------- NEW format v2 (58 payload + mask) ----------
% Expect: col1=t, col2..col59=d0..d57 (58), col60=mask
if ncol >= 60
    t_candidate       = A(:,1);
    payload_candidate = A(:,2:59);
    mask_candidate    = A(:,60);

    finite_ratio = mean(isfinite(payload_candidate(:)));
    if size(payload_candidate,2)==58 && finite_ratio > 0.01
        time   = t_candidate;
        values = payload_candidate;
        mask   = uint32(mask_candidate);
        fprintf("[INFO] Detected NEW format v2: (t_sec, d0..d57(58), mask)\n");
    end
end

% ---------- NEW format v1 (56 payload + mask) ----------
% Expect: col1=t, col2..col57=d0..d55 (56), col58=mask
if isempty(time) && ncol >= 58
    t_candidate       = A(:,1);
    payload_candidate = A(:,2:57);
    mask_candidate    = A(:,58);

    finite_ratio = mean(isfinite(payload_candidate(:)));
    if size(payload_candidate,2)==56 && finite_ratio > 0.01
        time   = t_candidate;
        values = payload_candidate;
        mask   = uint32(mask_candidate);
        fprintf("[INFO] Detected NEW format v1: (t_sec, d0..d55(56), mask)\n");
    end
end

% ---------- Legacy heuristic ----------
if isempty(time)
    % Legacy: time in col1, payload starts at col3
    if ncol >= (2 + 44)
        t_candidate = A(:,1);

        if nanmedian(t_candidate) > 1e6  % ns-scale heuristic
            time = t_candidate * 1e-9;
            fprintf("[INFO] Legacy time detected as ns -> converted to seconds\n");
        else
            time = t_candidate;
            fprintf("[INFO] Legacy time detected as seconds\n");
        end

        payload = A(:,3:end);
        if size(payload,2) < 44
            error("Legacy format assumed but payload columns are %d (<44).", size(payload,2));
        end

        values = payload(:,1:44);                 % legacy only has 44 mapping
        mask   = uint32(ones(size(values,1),1));  % synthesize
        fprintf("[INFO] Detected LEGACY format: (time, payload first 44, no mask)\n");
    else
        error("Unknown CSV format. Columns=%d. Need NEW(v1>=58 or v2>=60) or LEGACY(>=46-ish).", ncol);
    end
end

%% 3) Cleanup
validTime = isfinite(time);
time   = time(validTime);
values = values(validTime,:);
mask   = mask(validTime);

if numel(time) == 0
    error("All time entries are NaN. CSV may not contain numeric time.");
end

time = time - time(1);

N = size(values,1);
fprintf("[INFO] Using %d rows after cleanup.\n", N);

%% 4) Optional: drop rows where payload seems invalid (NEW format uses bit0)
SIZE_OK = bitshift(uint32(1),0);
ok = bitand(mask, SIZE_OK) ~= 0;

if any(ok)
    time   = time(ok);
    values = values(ok,:);
    mask   = mask(ok);
    N      = size(values,1);
end

if N < 5
    error("Too few valid rows (%d). Logger may have stopped early or file is mostly empty.", N);
end

%% 5) Renderer (vector PDF)
set(groot,'defaultFigureRenderer','painters');

%% 6) Decode payload mapping
payload_cols = size(values,2);
if payload_cols < 44
    error("values column size is %d, expected >=44.", payload_cols);
end

isNew56 = (payload_cols >= 56);
isNew58 = (payload_cols >= 58);

% --- allocate ---
force_global_raw       = zeros(N,3);
position_meas          = zeros(N,3);
rpy_meas               = zeros(N,3);
acc_global             = zeros(N,3);
vel_global             = zeros(N,3);
vbat                   = zeros(N,2);
cmd_position           = zeros(N,3);
cmd_yaw_deg            = zeros(N,1);
cmd_yaw                = zeros(N,1);
force_global_scaled    = zeros(N,3);
final_cmd_position     = zeros(N,3);
final_cmd_yaw_deg      = zeros(N,1);
final_cmd_yaw_rad      = zeros(N,1);
cmd_x_force            = zeros(N,1);
world_Fext_MOB         = zeros(N,3);
world_Fext_DOB         = zeros(N,3);
vel_from_pos           = zeros(N,3);
rpy_kalman             = zeros(N,3);
rpy_comp               = zeros(N,3);

gyro_fb                = nan(N,3);

% NEW v2: state_body_vel actual (body yaw-aligned)
state_body_vel         = nan(N,2);   % [bodyVX, bodyVY]

% velocity desired: posCtl.targetVX/VY/VZ  (네 말대로 이거!)
vel_des                = nan(N,3);

% attitude desired: controller.roll/pitch/yaw  (deg)
att_des_deg            = nan(N,3);
att_des_rad            = nan(N,3);

% rate desired: controller.rollRate/pitchRate/yawRate
rate_des               = nan(N,3);

% --- 0-2 ---
force_global_raw(:,1:3) = values(:,1:3);

% --- 3-5 ---
position_meas(:,1:3) = values(:,4:6);

% --- 6-8 ---
rpy_meas(:,1:3) = values(:,7:9);

% --- 9-11 ---
acc_global(:,1:3) = values(:,10:12);

% --- 12-14 ---
vel_global(:,1:3) = values(:,13:15);

% --- 15-16 ---
vbat(:,1) = values(:,16);
vbat(:,2) = values(:,17);

% --- 17-19 ---
cmd_position(:,1:3) = values(:,18:20);

% --- 20 ---
cmd_yaw_deg = values(:,21);
cmd_yaw     = cmd_yaw_deg * pi/180;

% --- 21-23 ---
force_global_scaled(:,1:3) = values(:,22:24);

% --- 24-26 ---
final_cmd_position(:,1:3) = values(:,25:27);

% --- 27 ---
final_cmd_yaw_deg = values(:,28);
final_cmd_yaw_rad = final_cmd_yaw_deg * pi/180;

% --- 28 ---
cmd_x_force(:,1) = values(:,29);

% --- 29-31 ---
world_Fext_MOB(:,1:3) = values(:,30:32);

% --- 32-34 ---
world_Fext_DOB(:,1:3) = values(:,33:35);

% --- 35-37 ---
vel_from_pos(:,1:3) = values(:,36:38);

% --- 38-40 ---
rpy_kalman(:,1:3) = values(:,39:41);

% --- 41-43 ---
rpy_comp(:,1:3) = values(:,42:44);

% ========= NEW appended (v1/v2 분기) =========
if isNew56
    if isNew58
        % v2 (58 payload): indices
        % 44-46: gyro_fb       -> col 45:47
        % 47-48: state_body_vel-> col 48:49
        % 49-51: vel_des       -> col 50:52
        % 52-54: att_des_deg   -> col 53:55
        % 55-57: rate_des      -> col 56:58

        gyro_fb(:,1:3)        = values(:,45:47);
        state_body_vel(:,1:2) = values(:,48:49);
        vel_des(:,1:3)        = values(:,50:52);
        att_des_deg(:,1:3)    = values(:,53:55);
        rate_des(:,1:3)       = values(:,56:58);
        att_des_rad           = att_des_deg * pi/180;

        fprintf("[INFO] Payload decode: NEW v2 (58 payload) mapping applied.\n");
    else
        % v1 (56 payload): indices
        % 44-46: gyro_fb     -> col 45:47
        % 47-49: vel_des     -> col 48:50
        % 50-52: att_des_deg -> col 51:53
        % 53-55: rate_des    -> col 54:56

        gyro_fb(:,1:3)     = values(:,45:47);
        vel_des(:,1:3)     = values(:,48:50);
        att_des_deg(:,1:3) = values(:,51:53);
        rate_des(:,1:3)    = values(:,54:56);
        att_des_rad        = att_des_deg * pi/180;

        fprintf("[INFO] Payload decode: NEW v1 (56 payload) mapping applied.\n");
    end
end

%% ---- Dashboard figure (desired vs actual overlaid) ----
twin = [time(1) time(end)];

f = figure('Name','DataLogging Overview (desired vs actual overlays)','NumberTitle','off', ...
           'Color','w','Units','normalized','Position',[0 0 1 1]);

tl = tiledlayout(f, 6, 2, 'TileSpacing','compact', 'Padding','compact');

haveVelDes = any(isfinite(vel_des(:)));
haveAttDes = any(isfinite(att_des_rad(:)));
haveRate   = any(isfinite(gyro_fb(:))) && any(isfinite(rate_des(:)));
haveBodyV  = any(isfinite(state_body_vel(:))); % v2에서만 의미있음

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
% LEFT BOTTOM: Velocity (desired vs actual OVERLAY)
%  - VX/VY: desired=posCtl.targetVX/VY (yaw-aligned body)  -> actual=posCtl.bodyVX/VY (state_body_vel)
%  - VZ:    desired=posCtl.targetVZ    -> actual=stateEstimate.vz (vel_global(:,3))
% -------------------------
nexttile(tl, 7);
if haveBodyV
    plot(time, state_body_vel(:,1), 'b-','LineWidth',1.2); hold on;
else
    % fallback: global vx (프레임 mismatch일 수 있음)
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
% RIGHT TOP: Attitude rpy (desired vs actual OVERLAY)
%   actual: rpy_meas (from pose)
%   desired: att_des_rad (controller.roll/pitch/yaw [deg] -> rad)
%   yaw: also overlay FW yaw cmd for reference
% -------------------------
nexttile(tl, 2);
plot(time, rpy_meas(:,1), 'b-','LineWidth',1.2); hold on;
if haveAttDes, plot(time, att_des_rad(:,1), 'r--','LineWidth',1.2); end
grid on; xlim(twin);
title('Roll (meas vs des)');
if haveAttDes, legend('meas','des','Location','best'); end

nexttile(tl, 4);
plot(time, rpy_meas(:,2), 'b-','LineWidth',1.2); hold on;
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
% RIGHT BOTTOM: Rate (desired vs actual OVERLAY)
%   actual: gyro_fb (gyro.x/y/z)
%   desired: rate_des (controller.rollRate/pitchRate/yawRate)
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
    plot(time, rate_des(:,2), 'r--','LineWidth',1.2);
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

set(findall(f,'Type','axes'),'FontSize',9,'Color','w');

disp("[DONE] Dashboard generated (pos/vel/att/rate: desired vs actual overlays).");

% ---- quick sanity print ----
if haveVelDes
    disp("[CHECK] vel_des is interpreted as posCtl.targetVX/VY/VZ (desired).");
end
if haveBodyV
    disp("[CHECK] state_body_vel is interpreted as posCtl.bodyVX/bodyVY (actual body yaw-aligned).");
else
    disp("[WARN] state_body_vel not found (likely old CSV). Vel X/Y plots use global vx/vy fallback.");
end

%% ---- 아래부터(기존 개별 figure들) 네 코드 그대로 두면 됨 ----
% ... (너가 올린 나머지 plot 코드들) ...

disp("[DONE] Plots generated successfully.");

%% 13. dv/dt acc vs force
acc_from_vel = zeros(N,3);
acc_from_vel(:,1) = gradient(vel_global(:,1), time);
acc_from_vel(:,2) = gradient(vel_global(:,2), time);
acc_from_vel(:,3) = gradient(vel_global(:,3), time);

figure('Name','d(vel)/dt Acceleration vs Force','NumberTitle','off','Color','w');

subplot(3,1,1);
plot(time, mass * acc_from_vel(:,1),'LineWidth',1.4); grid on; hold on;
plot(time, force_global_scaled(:,1),'LineWidth',1.4);
plot(time, force_scale .* force_global_raw(:,1),'LineWidth',1.4);
title('X axis (m dv_x/dt vs force)');
legend('m dv_x/dt','F_x^{scaled}','F_x^{(v-based) scaled}','Location','best');

subplot(3,1,2);
plot(time, mass * acc_from_vel(:,2),'LineWidth',1.4); grid on; hold on;
plot(time, force_global_scaled(:,2),'LineWidth',1.4);
plot(time, force_scale .* force_global_raw(:,2),'LineWidth',1.4);
title('Y axis (m dv_y/dt vs force)');

subplot(3,1,3);
plot(time, mass * (acc_from_vel(:,3) + g),'LineWidth',1.4); grid on; hold on;
plot(time, force_global_scaled(:,3),'LineWidth',1.4);
plot(time, force_scale .* force_global_raw(:,3),'LineWidth',1.4);
title('Z axis (m(dv_z/dt + g) vs force)');
xlabel('time [s]');

%% 15. Velocity EKF vs suVelFromPos
figure('Name','Velocity EKF vs suVelFromPos','NumberTitle','off','Color','w');
subplot(3,1,1);
plot(time, vel_global(:,1),'k-','LineWidth',1.4); hold on;
plot(time, vel_from_pos(:,1),'r--','LineWidth',1.4);
grid on; title('v_x (EKF vs pos-diff)'); legend('EKF','pos-diff','Location','best');

subplot(3,1,2);
plot(time, vel_global(:,2),'k-','LineWidth',1.4); hold on;
plot(time, vel_from_pos(:,2),'r--','LineWidth',1.4);
grid on; title('v_y (EKF vs pos-diff)'); legend('EKF','pos-diff','Location','best');

subplot(3,1,3);
plot(time, vel_global(:,3),'k-','LineWidth',1.4); hold on;
plot(time, vel_from_pos(:,3),'r--','LineWidth',1.4);
grid on; title('v_z (EKF vs pos-diff)'); legend('EKF','pos-diff','Location','best'); xlabel('time [s]');

%% 16. RPY compare
figure('Name','RPY compare (kalman vs comp)','NumberTitle','off','Color','w');
subplot(3,1,1);
plot(time, rpy_kalman(:,1), 'LineWidth',1.4); hold on; grid on;
plot(time, rpy_comp(:,1),   'LineWidth',1.6);
title('Roll [rad]'); legend('kalman','comp','Location','best');

subplot(3,1,2);
plot(time, rpy_kalman(:,2), 'LineWidth',1.4); hold on; grid on;
plot(time, rpy_comp(:,2),   'LineWidth',1.6);
title('Pitch [rad]'); legend('kalman','comp','Location','best');

subplot(3,1,3);
plot(time, rpy_kalman(:,3), 'LineWidth',1.4); hold on; grid on;
plot(time, rpy_comp(:,3),   'LineWidth',1.6);
title('Yaw [rad]'); xlabel('time [s]'); legend('kalman','comp','Location','best');

disp("[DONE] Plots generated successfully.");
