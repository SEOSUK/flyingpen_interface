clear all;
close all;
clc;

%% 1. CSV 읽기
csv_path = 'data_logging_msg.csv';
data = readtable(csv_path, 'VariableNamingRule', 'preserve');

% 시간 (ns -> s)
time = data{:, 1} * 1e-9;

% 나머지 값들 (Float64MultiArray: push index 0~43 → values(:,1)~values(:,44))
values = data{:, 3:end};
N = size(values, 1);

%% 포맷 sanity check (0~43 → 총 44개)
if size(values, 2) < 44
    error('values column size is %d, but expected >= 44 (0~43). CSV logging format mismatch.', ...
        size(values, 2));
end

%% 렌더러 설정 (벡터 PDF용)
set(groot, 'defaultFigureRenderer', 'painters');

%% 2. ROS2 data_logging_msg 포맷 정리 (DataLoggingNode.loopOnce() 기준)
%  0- 2 : global_force_input_ (Fx, Fy, Fz) [World, raw]
%  3- 5 : global_position_meas_ (px, py, pz) [World]
%  6- 8 : rpy_angle_meas_ (roll, pitch, yaw) [rad] (from pose)
%  9-11 : global_acc_meas_ (ax, ay, az) [World] (Gs)
% 12-14 : global_vel_meas_ (vx, vy, vz) [World, EKF] [m/s]
% 15-16 : vbat_raw_, vbat_filtered_ [V]
% 17-19 : cmd_position_ (x,y,z) [ROS cmd_position]
% 20    : cmd_yaw_deg_ [deg]
% 21-23 : global_force_input_scaled_ (Fx, Fy, Fz) [World, scaled]
% 24-26 : final_setpoint_pos_ (x,y,z) [ctrl target in FW]
% 27    : final_setpoint_yaw_deg_ [deg]
% 28    : su_cmd_fx_ [force command from FW param su_cmd.cmd_fx]
% 29-31 : world_Fext_MOB (MOB World 힘 추정값 [N])
% 32-34 : world_Fext_DOB (DOB World 힘 추정값 [N])
% 35-37 : vel_from_pos (suVelFromPos vx,vy,vz [m/s])
% 38-40 : rpy_kalman (roll,pitch,yaw) [rad] (kalman.q -> RPY)
% 41-43 : rpy_comp (roll,pitch,yaw) [rad] (kalman.qComp -> RPY)

force_global_raw      = zeros(N, 3);
position_meas         = zeros(N, 3);
rpy_meas              = zeros(N, 3);
acc_global            = zeros(N, 3);   % world accel (Gs 단위)
vel_global            = zeros(N, 3);   % EKF velocity [m/s]
vbat                  = zeros(N, 2);
cmd_position          = zeros(N, 3);
cmd_yaw_deg           = zeros(N, 1);
cmd_yaw               = zeros(N, 1);
force_global_scaled   = zeros(N, 3);
final_cmd_position    = zeros(N, 3);
final_cmd_yaw_deg     = zeros(N, 1);
final_cmd_yaw_rad     = zeros(N, 1);
cmd_x_force           = zeros(N, 1);
world_Fext_MOB        = zeros(N, 3);
world_Fext_DOB        = zeros(N, 3);
vel_from_pos          = zeros(N, 3);

% NEW
rpy_kalman            = zeros(N, 3);
rpy_comp              = zeros(N, 3);

%% 3. 매핑 (C++ push 순서와 1:1 대응)
% values(:,1) == push index 0

% 0-2: global_force_input_
force_global_raw(:, 1) = values(:, 1);
force_global_raw(:, 2) = values(:, 2);
force_global_raw(:, 3) = values(:, 3);

% 3-5: global_position_meas_
position_meas(:, 1) = values(:, 4);
position_meas(:, 2) = values(:, 5);
position_meas(:, 3) = values(:, 6);

% 6-8: rpy_angle_meas_ [rad]
rpy_meas(:, 1) = values(:, 7);
rpy_meas(:, 2) = values(:, 8);
rpy_meas(:, 3) = values(:, 9);

% 9-11: global_acc_meas_ [world, Gs]
acc_global(:, 1) = values(:, 10);
acc_global(:, 2) = values(:, 11);
acc_global(:, 3) = values(:, 12);

% 12-14: global_vel_meas_ [world, EKF]
vel_global(:, 1) = values(:, 13);
vel_global(:, 2) = values(:, 14);
vel_global(:, 3) = values(:, 15);

% 15-16: vbat_raw_, vbat_filtered_
vbat(:, 1) = values(:, 16);     % raw
vbat(:, 2) = values(:, 17);     % filtered

% 17-19: cmd_position_ (ROS 명령 x,y,z)
cmd_position(:, 1) = values(:, 18);
cmd_position(:, 2) = values(:, 19);
cmd_position(:, 3) = values(:, 20);

% 20: cmd_yaw_deg_ (ROS yaw [deg])
cmd_yaw_deg = values(:, 21);
cmd_yaw     = cmd_yaw_deg * pi / 180;  % [rad]

% 21-23: global_force_input_scaled_
force_global_scaled(:, 1) = values(:, 22);
force_global_scaled(:, 2) = values(:, 23);
force_global_scaled(:, 3) = values(:, 24);

% 24-26: final_setpoint_pos_
final_cmd_position(:, 1) = values(:, 25);
final_cmd_position(:, 2) = values(:, 26);
final_cmd_position(:, 3) = values(:, 27);

% 27: final_setpoint_yaw_deg_
final_cmd_yaw_deg = values(:, 28);         % [deg]
final_cmd_yaw_rad = final_cmd_yaw_deg*pi/180; % [rad]

% 28: su_cmd_fx_
cmd_x_force(:, 1) = values(:, 29);

% 29-31: world_Fext_MOB
world_Fext_MOB(:, 1) = values(:, 30);
world_Fext_MOB(:, 2) = values(:, 31);
world_Fext_MOB(:, 3) = values(:, 32);

% 32-34: world_Fext_DOB
world_Fext_DOB(:, 1) = values(:, 33);
world_Fext_DOB(:, 2) = values(:, 34);
world_Fext_DOB(:, 3) = values(:, 35);

% 35-37: vel_from_pos (suVelFromPos)
vel_from_pos(:, 1) = values(:, 36);
vel_from_pos(:, 2) = values(:, 37);
vel_from_pos(:, 3) = values(:, 38);

% 38-40: rpy_kalman (roll,pitch,yaw) [rad]
rpy_kalman(:, 1) = values(:, 39);
rpy_kalman(:, 2) = values(:, 40);
rpy_kalman(:, 3) = values(:, 41);

% 41-43: rpy_comp (roll,pitch,yaw) [rad]
rpy_comp(:, 1) = values(:, 42);
rpy_comp(:, 2) = values(:, 43);
rpy_comp(:, 3) = values(:, 44);

%% (추가) force_global_scaled_filt 생성 (원래 코드에서 미정의 변수 사용하던 부분 해결)
% 필요없으면 아래 2줄 지워도 됨
force_global_scaled_filt = movmean(force_global_scaled, 7, 1); % simple smoothing
% force_global_scaled_filt = force_global_scaled; % (대체) 필터 없이 그대로

%% 5. 대시보드
twin = [time(1) time(end)];

f = figure('Name', 'DataLogging Overview', 'NumberTitle', 'off', ...
    'Color', 'w', 'Units', 'normalized', 'Position', [0 0 1 1]);

left = 0.04; right = 0.02; top = 0.04; bottom = 0.06;
hgap = 0.03; vgap = 0.05;
ncol = 3; nrow = 2;

w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;

getPos = @(row, col)[ ...
    left + (col-1)*(w+hgap), ...
    1 - top - row*h - (row-1)*vgap, ...
    w, h];

% (1,1) 위치 커맨드 vs 위치 (ROS / FW / meas)
p11  = uipanel('Parent', f, 'Position', getPos(1,1), 'BackgroundColor', 'w');
tl11 = tiledlayout(p11, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl11, 1);
plot(time, cmd_position(:,1), 'g:', 'LineWidth', 1.0); hold on;
plot(time, final_cmd_position(:,1), 'r--', 'LineWidth', 1.2);
plot(time, position_meas(:,1), 'b-', 'LineWidth', 1.2);
grid on; xlim(twin);
title('X pos (ROS cmd / FW target / meas)');
legend('ROS cmd', 'FW target', 'meas', 'Location', 'best');

nexttile(tl11, 2);
plot(time, cmd_position(:,2), 'g:', 'LineWidth', 1.0); hold on;
plot(time, final_cmd_position(:,2), 'r--', 'LineWidth', 1.2);
plot(time, position_meas(:,2), 'b-', 'LineWidth', 1.2);
grid on; xlim(twin);
title('Y pos (ROS cmd / FW target / meas)');

nexttile(tl11, 3);
plot(time, cmd_position(:,3), 'g:', 'LineWidth', 1.0); hold on;
plot(time, final_cmd_position(:,3), 'r--', 'LineWidth', 1.2);
plot(time, position_meas(:,3), 'b-', 'LineWidth', 1.2);
grid on; xlim(twin);
title('Z pos (ROS cmd / FW target / meas)');
xlabel('time [s]');

% (1,2) 속도 (world, EKF)
p12  = uipanel('Parent', f, 'Position', getPos(1,2), 'BackgroundColor', 'w');
tl12 = tiledlayout(p12, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl12, 1);
plot(time, vel_global(:,1), 'LineWidth', 1.2);
grid on; xlim(twin);
title('vel x_{world} [m/s] (EKF)');

nexttile(tl12, 2);
plot(time, vel_global(:,2), 'LineWidth', 1.2);
grid on; xlim(twin);
title('vel y_{world} [m/s] (EKF)');

nexttile(tl12, 3);
plot(time, vel_global(:,3), 'LineWidth', 1.2);
grid on; xlim(twin);
title('vel z_{world} [m/s] (EKF)');
xlabel('time [s]');

% (1,3) 자세 (RPY + FW yaw cmd)
p13  = uipanel('Parent', f, 'Position', getPos(1,3), 'BackgroundColor', 'w');
tl13 = tiledlayout(p13, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl13, 1);
plot(time, rpy_meas(:,1), 'LineWidth', 1.2);
grid on; xlim(twin);
title('roll [rad] (meas)');

nexttile(tl13, 2);
plot(time, rpy_meas(:,2), 'LineWidth', 1.2);
grid on; xlim(twin);
title('pitch [rad] (meas)');

nexttile(tl13, 3);
plot(time, rpy_meas(:,3), 'LineWidth', 1.2); hold on;
plot(time, final_cmd_yaw_rad, 'k-', 'LineWidth', 1.2);
grid on; xlim(twin);
title('yaw (meas / FW cmd) [rad]');
legend('meas', 'FW cmd', 'Location', 'best');
xlabel('time [s]');

% (2,1) F_input_scaled (World)
p21  = uipanel('Parent', f, 'Position', getPos(2,1), 'BackgroundColor', 'w');
tl21 = tiledlayout(p21, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl21, 1);
plot(time, force_global_scaled(:,1), 'LineWidth', 1.2);
grid on; xlim(twin);
title('F_x^{scaled} (world) [arb/N]');

nexttile(tl21, 2);
plot(time, force_global_scaled(:,2), 'LineWidth', 1.2);
grid on; xlim(twin);
title('F_y^{scaled} (world) [arb/N]');

nexttile(tl21, 3);
plot(time, force_global_scaled(:,3), 'LineWidth', 1.2);
grid on; xlim(twin);
title('F_z^{scaled} (world) [arb/N]');
xlabel('time [s]');

% (2,2) m·a vs F_input_scaled
p22  = uipanel('Parent', f, 'Position', getPos(2,2), 'BackgroundColor', 'w');
tl22 = tiledlayout(p22, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

mass_dash = 0.04;   % [kg]
g_dash    = 9.81;   % [m/s^2]

ma_x = acc_global(:,1) * mass_dash * g_dash;
ma_y = acc_global(:,2) * mass_dash * g_dash;
ma_z = acc_global(:,3) * mass_dash * g_dash;

nexttile(tl22, 1);
plot(time, ma_x, 'LineWidth', 1.2); hold on;
plot(time, force_global_scaled(:,1), 'LineWidth', 1.2);
grid on; xlim(twin);
title('X: m a_x vs F_x^{scaled}');
legend('m a_x', 'F_x^{scaled}', 'Location', 'best');

nexttile(tl22, 2);
plot(time, ma_y, 'LineWidth', 1.2); hold on;
plot(time, force_global_scaled(:,2), 'LineWidth', 1.2);
grid on; xlim(twin);
title('Y: m a_y vs F_y^{scaled}');

nexttile(tl22, 3);
plot(time, ma_z + mass_dash*g_dash, 'LineWidth', 1.2); hold on;
plot(time, force_global_scaled(:,3), 'LineWidth', 1.2);
grid on; xlim(twin);
title('Z: m a_z + mg vs F_z^{scaled}');
xlabel('time [s]');

% (2,3) F_ext (외란 추정: MOB vs DOB)
p23  = uipanel('Parent', f, 'Position', getPos(2,3), 'BackgroundColor', 'w');
tl23 = tiledlayout(p23, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile(tl23, 1);
plot(time, world_Fext_MOB(:,1), 'LineWidth', 1.2); hold on;
plot(time, world_Fext_DOB(:,1), '--', 'LineWidth', 1.2);
grid on; xlim(twin);
title('F_{ext,x} [N]');
legend('MOB', 'DOB', 'Location', 'best');

nexttile(tl23, 2);
plot(time, world_Fext_MOB(:,2), 'LineWidth', 1.2); hold on;
plot(time, world_Fext_DOB(:,2), '--', 'LineWidth', 1.2);
grid on; xlim(twin);
title('F_{ext,y} [N]');

nexttile(tl23, 3);
plot(time, world_Fext_MOB(:,3), 'LineWidth', 1.2); hold on;
plot(time, world_Fext_DOB(:,3), '--', 'LineWidth', 1.2);
grid on; xlim(twin);
title('F_{ext,z} [N]');
xlabel('time [s]');

% 공통 스타일
set(findall(f, 'Type', 'axes'), 'FontSize', 9, 'Color', 'w');

%% 6. Attitude only (meas + cmd)
figure('Name', 'Attitude detail (meas + cmd)', 'NumberTitle', 'off', 'Color', 'w');

subplot(3,1,1);
plot(time, rpy_meas(:,1), 'b-', 'LineWidth', 1.2);
xlim([32 52])
ylim([-0.15 0.15])
grid on; title('roll [rad]');

subplot(3,1,2);
plot(time, rpy_meas(:,2), 'b-', 'LineWidth', 1.2);
xlim([32 52])
ylim([-0.15 0.15])
grid on; title('pitch [rad]');

subplot(3,1,3);
plot(time, rpy_meas(:,3), 'b-', 'LineWidth', 1.2); hold on;
plot(time, final_cmd_yaw_rad, 'r--', 'LineWidth', 1.4);
xlim([32 52])
ylim([-0.15 0.15])
grid on; xlabel('time [s]'); title('yaw [rad]');
legend('meas', 'cmd', 'Location', 'best');

%% 7. Position only (ROS cmd vs FW target vs meas)
figure('Name', 'Position detail', 'NumberTitle', 'off', 'Color', 'w');

% 1) norm 계산
% 1) norm 계산
acc_global_norm = vecnorm(acc_global, 2, 2);   % sqrt(ax^2 + ay^2 + az^2)

% 2) low-pass filter (1st-order IIR)
fc = 0.3;                           % [Hz] 컷오프 (원하는 값으로 조절)
dt = median(diff(time));          % [s] 샘플 타임 (time이 초 단위라고 가정)
tau = 1/(2*pi*fc);                % time constant
alpha = dt/(tau + dt);            % 0~1

acc_global_norm_lpf = zeros(size(acc_global_norm));
acc_global_norm_lpf(1) = acc_global_norm(1);
for k = 2:length(acc_global_norm)
    acc_global_norm_lpf(k) = acc_global_norm_lpf(k-1) + alpha*(acc_global_norm(k) - acc_global_norm_lpf(k-1));
end

subplot(3,1,1);
plot(time, final_cmd_position(:,1), 'r--', 'LineWidth', 1.4); hold on;
plot(time, position_meas(:,1), 'b-', 'LineWidth', 1.2);
plot(time, acc_global_norm_lpf, 'k', 'LineWidth', 1.6);
grid on;
legend('cmd x', 'meas x', '|acc| lpf');

subplot(3,1,2);
plot(time, final_cmd_position(:,2), 'r--', 'LineWidth', 1.4); hold on;
plot(time, position_meas(:,2), 'b-', 'LineWidth', 1.2);
grid on;
legend('cmd y', 'meas y');

subplot(3,1,3);
plot(time, final_cmd_position(:,3), 'r--', 'LineWidth', 1.4); hold on;
plot(time, position_meas(:,3), 'b-', 'LineWidth', 1.2);
grid on;
legend('cmd z', 'meas z');


%% 13. Velocity (world, EKF) detail
figure('Name', 'Velocity (world, EKF) detail', 'NumberTitle', 'off', 'Color', 'w');

subplot(3,1,1);
plot(time, vel_global(:,1), 'LineWidth', 1.4);
grid on; title('vel X_{world} [m/s] (EKF)');
xlim([17 58])

subplot(3,1,2);
plot(time, vel_global(:,2), 'LineWidth', 1.4);
grid on; title('vel Y_{world} [m/s] (EKF)');
xlim([17 58])

subplot(3,1,3);
plot(time, vel_global(:,3), 'LineWidth', 1.4);
grid on; title('vel Z_{world} [m/s] (EKF)');
xlabel('time [s]');
xlim([17 58])

%% 8. Force scale fitting (호버 구간에서 Fz↔m*g 맞추기)
mass = 0.04;  % [kg]
g    = 9.81;

time_start = 28;
time_end   = 86;

idx = (time >= time_start) & (time <= time_end);

v_seg  = vbat(idx, 2);              % 전압 (filtered)
Fz_seg = force_global_raw(idx, 3);  % raw Fz (world)

valid  = abs(Fz_seg) > 1e-3;
v_seg  = v_seg(valid);
Fz_seg = Fz_seg(valid);

y = mass * g * ones(size(Fz_seg));
X = [v_seg .* Fz_seg, Fz_seg];  % [N×2]

theta = X \ y;
a = theta(1);
b = theta(2);

force_scale = a * vbat(:,2) + b;

fprintf('\n[Force scale fitting result]\n');
fprintf('Time window: %.1f ~ %.1f s\n', time_start, time_end);
fprintf('Mass: %.3f kg, g: %.2f m/s^2\n', mass, g);
fprintf('----------------------------------\n');
fprintf('a = %.6f\n', a);
fprintf('b = %.6f\n\n', b);

%% 8-acc. Acceleration 기반 Force scale fitting (F = m a 매칭)
acc_time_start = 30;
acc_time_end   = 85;

idx_acc = (time >= acc_time_start) & (time <= acc_time_end);

ax_seg = acc_global(idx_acc, 1) * g;  % [m/s^2]
ay_seg = acc_global(idx_acc, 2) * g;  % [m/s^2]
az_seg = acc_global(idx_acc, 3) * g;  % [m/s^2]

Fx_seg = force_global_scaled(idx_acc, 1);
Fy_seg = force_global_scaled(idx_acc, 2);
Fz_seg = force_global_scaled(idx_acc, 3);

valid_x = abs(Fx_seg) > 1e-3;
valid_y = abs(Fy_seg) > 1e-3;
valid_z = abs(Fz_seg) > 1e-3;

y_x = mass * ax_seg(valid_x);
X_x = Fx_seg(valid_x);
kx  = X_x \ y_x;

y_y = mass * ay_seg(valid_y);
X_y = Fy_seg(valid_y);
ky  = X_y \ y_y;

y_z = mass * (az_seg(valid_z) + g);
X_z = Fz_seg(valid_z);
kz  = X_z \ y_z;

fprintf('[Acceleration-based force scaling]\n');
fprintf('Time window: %.1f ~ %.1f s\n', acc_time_start, acc_time_end);
fprintf('Mass: %.3f kg, g: %.2f m/s^2\n', mass, g);
fprintf('----------------------------------\n');
fprintf('kx (X axis) = %.6f [N / arb]\n', kx);
fprintf('ky (Y axis) = %.6f [N / arb]\n', ky);
fprintf('kz (Z axis) = %.6f [N / arb]\n\n', kz);

ax_full = acc_global(:,1) * g;
ay_full = acc_global(:,2) * g;
az_full = acc_global(:,3) * g;

force_from_acc = zeros(N, 3);
force_from_acc(:,1) = kx * force_global_scaled(:,1);
force_from_acc(:,2) = ky * force_global_scaled(:,2);
force_from_acc(:,3) = kz * force_global_scaled(:,3);

%% 9. force only + force scaling
figure('Name', 'Force detail', 'NumberTitle', 'off', 'Color', 'w');

subplot(3,2,1);
plot(time, force_global_raw(:,1), 'LineWidth', 1.4);
grid on; title('Fx raw [N]');

subplot(3,2,3);
plot(time, force_global_raw(:,2), 'LineWidth', 1.4);
grid on; title('Fy raw [N]');

subplot(3,2,5);
plot(time, force_global_raw(:,3), 'LineWidth', 1.4);
grid on; title('Fz raw [N]');
xlabel('time [s]');

subplot(3,2,2);
plot(time, force_scale .* force_global_raw(:,1), 'LineWidth', 1.4);
grid on; title('Fx scaled [N]');

subplot(3,2,4);
plot(time, force_scale .* force_global_raw(:,2), 'LineWidth', 1.4);
grid on; title('Fy scaled [N]');

subplot(3,2,6);
plot(time, force_scale .* force_global_raw(:,3), 'LineWidth', 1.4);
grid on; title('Fz scaled [N]');
xlabel('time [s]');

%% 10. acceleration * mass vs Force_scaled // F=ma 확인
figure('Name', 'Acceleration vs Force', 'NumberTitle', 'off', 'Color', 'w');

subplot(3,1,1);
plot(time, acc_global(:,1) * mass * g, 'LineWidth', 1.4);
grid on; hold on;
plot(time, force_global_scaled(:,1), 'LineWidth', 1.4);
plot(time, force_scale .* force_global_raw(:,1), 'LineWidth', 1.4);
title('X axis (m a_{x,world} vs force)');
legend('m a_x', 'FW scaled', 'F^{(vbat) scaled}', 'Location', 'best');

subplot(3,1,2);
plot(time, acc_global(:,2) * mass * g, 'LineWidth', 1.4);
grid on; hold on;
plot(time, force_global_scaled(:,2), 'LineWidth', 1.4);
plot(time, force_scale .* force_global_raw(:,2), 'LineWidth', 1.4);
title('Y axis (m a_{y,world} vs force)');

subplot(3,1,3);
plot(time, acc_global(:,3) * mass * g + mass * g, 'LineWidth', 1.4);
grid on; hold on;
plot(time, force_global_scaled(:,3), 'LineWidth', 1.4);
plot(time, force_scale .* force_global_raw(:,3), 'LineWidth', 1.4);
title('Z axis (m (a_{z,world} + g) vs force)');
xlabel('time [s]');

%% 11. acceleration global only (detailed)
figure('Name', 'Acceleration (world) detail', 'NumberTitle', 'off', 'Color', 'w');

subplot(3,1,1);
plot(time, acc_global(:,1), 'LineWidth', 1.4);
grid on; title('acc X_{world} [G]');
xlim([17 58])
ylim([-1.2 0.2])
subplot(3,1,2);
plot(time, acc_global(:,2), 'LineWidth', 1.4);
grid on; title('acc Y_{world} [G]');
xlim([17 58])
ylim([-0.6 0.6])

subplot(3,1,3);
plot(time, acc_global(:,3), 'LineWidth', 1.4);
grid on; title('acc Z_{world} [G]');
xlabel('time [s]');
xlim([17 58])
ylim([-0.6 0.6])


%% 12. Wrench Observation (MOB/DOB + Force command)
figure('Name', 'Force Estimation', 'NumberTitle', 'off', 'Color', 'w');

subplot(3,1,1);
plot(time, world_Fext_MOB(:,1), 'LineWidth', 1.4); hold on;
plot(time, world_Fext_DOB(:,1), '--', 'LineWidth', 1.4);
xlim([30 200]); ylim([-0.03 0.03]);
grid on; title('F_{ext,x}^{MOB/DOB}');
legend('MOB', 'DOB', 'Location', 'best');

subplot(3,1,2);
plot(time, world_Fext_MOB(:,2), 'LineWidth', 1.4); hold on;
plot(time, world_Fext_DOB(:,2), '--', 'LineWidth', 1.4);
xlim([30 200]); ylim([-0.03 0.03]);
grid on; title('F_{ext,y}^{MOB/DOB}');

subplot(3,1,3);
plot(time, world_Fext_MOB(:,3) - 0.006, 'LineWidth', 1.4); hold on;
plot(time, world_Fext_DOB(:,3) - 0.006, '--', 'LineWidth', 1.4);
xlim([30 200]); ylim([-0.03 0.03]);
grid on; title('F_{ext,z}^{MOB/DOB}');
xlabel('time [s]');

%% 12b. voltage and force command
figure('Name', 'Voltage & Force command', 'NumberTitle', 'off', 'Color', 'w');

subplot(3,1,1);
plot(time, vbat(:,1), 'LineWidth', 1.4);
grid on; title('vbat raw [V]');

subplot(3,1,2);
plot(time, vbat(:,2), 'LineWidth', 1.4);
grid on; title('vbat filtered [V]');
xlabel('time [s]');

subplot(3,1,3);
plot(time, cmd_x_force(:,1), 'LineWidth', 1.4);
grid on; title('Force command su\_cmd\_fx [N or arb.]');
xlabel('time [s]');

%% 13. vel을 미분해서 얻은 acc로 F = m a 확인 (d v / dt 기반)
acc_from_vel = zeros(N, 3);  % [m/s^2]
acc_from_vel(:,1) = gradient(vel_global(:,1), time);
acc_from_vel(:,2) = gradient(vel_global(:,2), time);
acc_from_vel(:,3) = gradient(vel_global(:,3), time);

figure('Name', 'd(vel)/dt Acceleration vs Force', 'NumberTitle', 'off', 'Color', 'w');

subplot(3,1,1);
plot(time, mass * acc_from_vel(:,1), 'LineWidth', 1.4);
grid on; hold on;
plot(time, force_global_scaled(:,1), 'LineWidth', 1.4);
plot(time, force_scale .* force_global_raw(:,1), 'LineWidth', 1.4);
title('X axis (m \cdot d v_x / dt vs force)');
legend('m d v_x / dt', 'F_x^{scaled}', 'F_x^{(v-based) scaled}', 'Location', 'best');

subplot(3,1,2);
plot(time, mass * acc_from_vel(:,2), 'LineWidth', 1.4);
grid on; hold on;
plot(time, force_global_scaled(:,2), 'LineWidth', 1.4);
plot(time, force_scale .* force_global_raw(:,2), 'LineWidth', 1.4);
title('Y axis (m \cdot d v_y / dt vs force)');
legend('m d v_y / dt', 'F_y^{scaled}', 'F_y^{(v-based) scaled}', 'Location', 'best');

subplot(3,1,3);
plot(time, mass * (acc_from_vel(:,3) + g), 'LineWidth', 1.4);
grid on; hold on;
plot(time, force_global_scaled(:,3), 'LineWidth', 1.4);
plot(time, force_scale .* force_global_raw(:,3), 'LineWidth', 1.4);
title('Z axis (m (d v_z / dt + g) vs force)');
xlabel('time [s]');
legend('m (d v_z / dt + g)', 'F_z^{scaled}', 'F_z^{(v-based) scaled}', 'Location', 'best');

%% 14. d(vel)/dt로 얻은 가속도만 따로 보기
figure('Name', 'Acc from Velocity (world) detail', 'NumberTitle', 'off', 'Color', 'w');

subplot(3,1,1);
plot(time, acc_from_vel(:,1), 'LineWidth', 1.4);
grid on; title('d v_x / dt (world) [m/s^2]');

subplot(3,1,2);
plot(time, acc_from_vel(:,2), 'LineWidth', 1.4);
grid on; title('d v_y / dt (world) [m/s^2]');

subplot(3,1,3);
plot(time, acc_from_vel(:,3), 'LineWidth', 1.4);
grid on; title('d v_z / dt (world) [m/s^2]');
xlabel('time [s]');

%% 15. Velocity EKF vs suVelFromPos (pos-diff+LPF 비교)
figure('Name', 'Velocity EKF vs suVelFromPos', 'NumberTitle', 'off', 'Color', 'w');

subplot(3,1,1);
plot(time, vel_global(:,1), 'k-', 'LineWidth', 1.4); hold on;
plot(time, vel_from_pos(:,1), 'r--', 'LineWidth', 1.4);
grid on; title('v_x (EKF vs suVelFromPos) [m/s]');
legend('EKF v_x', 'pos-diff v_x', 'Location', 'best');

subplot(3,1,2);
plot(time, vel_global(:,2), 'k-', 'LineWidth', 1.4); hold on;
plot(time, vel_from_pos(:,2), 'r--', 'LineWidth', 1.4);
grid on; title('v_y (EKF vs suVelFromPos) [m/s]');
legend('EKF v_y', 'pos-diff v_y', 'Location', 'best');

subplot(3,1,3);
plot(time, vel_global(:,3), 'k-', 'LineWidth', 1.4); hold on;
plot(time, vel_from_pos(:,3), 'r--', 'LineWidth', 1.4);
grid on; title('v_z (EKF vs suVelFromPos) [m/s]');
xlabel('time [s]');
legend('EKF v_z', 'pos-diff v_z', 'Location', 'best');

%% (추가) 간단 오버레이 플롯 (원래 너가 쓰던 것 유지)
figure;
plot(time, world_Fext_MOB(:,1), 'LineWidth', 3); hold on;
plot(time, world_Fext_DOB(:,1), '--', 'LineWidth', 1.4);
plot(time, rpy_meas(:,2), 'LineWidth', 1.4);
plot(time, vel_from_pos(:,1), 'r--', 'LineWidth', 1.4);
plot(time, force_global_scaled(:,1), 'LineWidth', 3);
grid on;
title('overlay: Fext / pitch / vel / force (x)');
legend('F_{ext} MOB (x)', 'F_{ext} DOB (x)', 'pitch [rad]', ...
    'vel-from-pos (x)', 'Force scaled (x)', 'Location', 'best');

%% Interaction (5 panels)
figure('Name', 'Interaction (5 panels)', 'NumberTitle', 'off', 'Color', 'w');

subplot(5,1,1);
plot(time, world_Fext_MOB(:,1), 'LineWidth', 1.4); hold on; grid on;
plot(time, world_Fext_DOB(:,1), '--', 'LineWidth', 1.4);
legend('MOB', 'DOB');

subplot(5,1,2);
plot(time, final_cmd_position(:,1), 'k-', 'LineWidth', 1.4); hold on; grid on;
plot(time, position_meas(:,1), 'b-', 'LineWidth', 1.2);
legend('cmd pos', 'pos');

subplot(5,1,3);
plot(time, rpy_meas(:,2), '-', 'LineWidth', 1.4); hold on; grid on;
plot(time, rpy_comp(:,2), '-', 'LineWidth', 1.6);
legend('kalman (q->rpy)', 'comp (qComp->rpy)', 'Location', 'best');

subplot(5,1,4);
plot(time, acc_global(:,1), 'LineWidth', 1.4);
grid on; title('acc [G, 9.81 m/s^2]');
legend('acceleration x');

subplot(5,1,5);
plot(time, vel_from_pos(:,1), 'LineWidth', 1.4); grid on; title('velocity [m/s]');
hold on;
plot(time, vel_global(:,1), 'LineWidth', 1.4);
legend('velocity (position diff)', 'velocity (EKF, from Firmware)');

%% 16. RPY 3종세트 비교 (meas vs kalman vs comp)
figure('Name', 'RPY compare (meas vs kalman vs comp)', 'NumberTitle', 'off', 'Color', 'w');

subplot(3,1,1);
plot(time, rpy_kalman(:,1), '-', 'LineWidth', 1.4); hold on; grid on;
plot(time, rpy_comp(:,1), '-', 'LineWidth', 1.6);
title('Roll [rad]');
legend('kalman (q->rpy)', 'comp (qComp->rpy)', 'Location', 'best');

subplot(3,1,2);
plot(time, rpy_meas(:,2), '-', 'LineWidth', 1.4); hold on; grid on;
plot(time, rpy_comp(:,2), '-', 'LineWidth', 1.6);
title('Pitch [rad]');
legend('kalman (q->rpy)', 'comp (qComp->rpy)', 'Location', 'best');

subplot(3,1,3);
plot(time, rpy_kalman(:,3), '-', 'LineWidth', 1.4); hold on; grid on;
plot(time, rpy_comp(:,3) + 3.141592, '-', 'LineWidth', 1.6);
title('Yaw [rad] (meas / kalman / comp / FW cmd)');
xlabel('time [s]');
legend('kalman (q->rpy)', 'comp (qComp->rpy)', 'Location', 'best');

%% R P Acc
figure('Name', 'Flow Deck Experiment', 'NumberTitle', 'off', 'Color', 'w');

subplot(3,2,1);
plot(time, position_meas(:,1), 'b', 'LineWidth', 1.0); hold on; grid on;
plot(time, final_cmd_position(:,1), 'r--', 'LineWidth', 1.3);
title('X posiiton');
legend('cmd x', 'meas x');
xlim([140 160]); ylim([-0.2 1.2]);

subplot(3,2,3);
plot(time, position_meas(:,2), 'b', 'LineWidth', 1.0); hold on; grid on;
plot(time, final_cmd_position(:,2), 'r--', 'LineWidth', 1.3);
title('Y posiiton');
legend('cmd y', 'meas y');
xlim([150 160]); ylim([0.15 0.35]);

subplot(3,2,5);
plot(time, position_meas(:,3), 'b', 'LineWidth', 1.0); hold on; grid on;
plot(time, final_cmd_position(:,3), 'r--', 'LineWidth', 1.3);
title('Z posiiton');
legend('cmd z', 'meas z');
xlim([140 160]); ylim([0.0 1.4]);

subplot(3,2,2);
plot(time, rpy_meas(:,1), 'r', 'LineWidth', 1.3); hold on; grid on;
title('Roll [rad]');
xlim([140 160]); ylim([-0.15 0.15]);

subplot(3,2,4);
plot(time, rpy_meas(:,2), 'r', 'LineWidth', 1.3); hold on; grid on;
title('Pitch [rad]');
xlim([140 160]); ylim([-0.15 0.15]);

subplot(3,2,6);
plot(time, rpy_meas(:,3), 'r', 'LineWidth', 1.3); hold on; grid on;
title('Yaw [rad]');
xlim([140 160]); ylim([-0.05 0.05]);
