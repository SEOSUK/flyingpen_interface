%% data_decryptor.m
% Robust reader + full plotting for the logging CSV.
% Supports BOTH formats:
%
% [NEW format]
%   col1: t_sec [s]
%   col2..col45: d0..d43 (44 doubles)
%   col46: validity_bitmask
%
% [OLD/LEGACY format - heuristic]
%   col1: time_ns or time_sec
%   payload starts at col3 (values = data(:,3:end)) and should have >=44 cols

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
% Force numeric when possible (text -> NaN)
opts = setvartype(opts, 'double');

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

%% 2) Detect format and extract (time, values[44], mask)
time = [];
values = [];
mask = [];

% --- NEW format check: 46 columns minimum, and col1 looks like seconds (smallish) ---
if ncol >= 46
    % Candidate new format:
    t_candidate = A(:,1);
    payload_candidate = A(:,2:45);
    mask_candidate = A(:,46);

    % Heuristic: payload has 44 cols and at least some finite numbers
    finite_ratio = mean(isfinite(payload_candidate(:)));
    if size(payload_candidate,2)==44 && finite_ratio > 0.01
        time = t_candidate;
        values = payload_candidate;
        mask = uint32(mask_candidate);
        fprintf("[INFO] Detected NEW format: (t_sec, d0..d43, mask)\n");
    end
end

% --- If not NEW, try legacy heuristic ---
if isempty(time)
    % Legacy: often time is ns in col1, payload starts at col3
    if ncol >= (2 + 44)
        t_candidate = A(:,1);

        % if looks like ns (very large), convert to seconds
        if nanmedian(t_candidate) > 1e6  % ns-scale heuristic
            time = t_candidate * 1e-9;
            fprintf("[INFO] Legacy time detected as ns -> converted to seconds\n");
        else
            time = t_candidate; % already sec
            fprintf("[INFO] Legacy time detected as seconds\n");
        end

        values = A(:,3:end);

        if size(values,2) < 44
            error("Legacy format assumed but payload columns are %d (<44).", size(values,2));
        end

        % Take first 44 as payload (0..43)
        values = values(:,1:44);

        % No mask in legacy; synthesize mask=1 for all rows
        mask = uint32(ones(size(values,1),1));
        fprintf("[INFO] Detected LEGACY format: (time, payload from col3.., no mask)\n");
    else
        error("Unknown CSV format. Columns=%d. Need NEW(>=46) or LEGACY(>=46-like with payload).", ncol);
    end
end

%% 3) Sanity checks
if isempty(time) || isempty(values)
    error("Failed to extract time/values. Format detection failed.");
end

if size(values,2) < 44
    error("values column size is %d, expected 44 (d0..d43).", size(values,2));
end

% Remove rows where time is NaN
validTime = isfinite(time);
time = time(validTime);
values = values(validTime,:);
mask = mask(validTime);

if numel(time) == 0
    error("All time entries are NaN. CSV may not contain numeric time.");
end

% Normalize time to start at 0
time = time - time(1);

N = size(values,1);
fprintf("[INFO] Using %d rows after cleanup.\n", N);

%% 4) Optional: drop rows where payload seems invalid (NEW format uses bit0)
% If mask is synthetic (legacy), this keeps all rows.
SIZE_OK = bitshift(uint32(1),0);
ok = bitand(mask, SIZE_OK) ~= 0;

% If almost everything becomes false (legacy mask=1), ok remains true anyway.
if any(ok)
    time = time(ok);
    values = values(ok,:);
    mask = mask(ok);
    N = size(values,1);
end

if N < 5
    error("Too few valid rows (%d). Logger may have stopped early or file is mostly empty.", N);
end

%% 5) Renderer (vector PDF)
set(groot,'defaultFigureRenderer','painters');

%% 6) Decode payload mapping (push index 0..43 => values(:,1)..values(:,44))
%  0- 2 : global_force_input_        (Fx, Fy, Fz)      [World, raw]
%  3- 5 : global_position_meas_      (px, py, pz)      [World]
%  6- 8 : rpy_angle_meas_            (roll, pitch, yaw) [rad]  (from pose)
%  9-11 : global_acc_meas_           (ax, ay, az)      [World] (Gs)
% 12-14 : global_vel_meas_           (vx, vy, vz)      [World, EKF] [m/s]
% 15-16 : vbat_raw_, vbat_filtered_  [V]
% 17-19 : cmd_position_              (x,y,z)           [ROS cmd_position]
% 20    : cmd_yaw_deg_               [deg]
% 21-23 : global_force_input_scaled_ (Fx, Fy, Fz)      [World, scaled]
% 24-26 : final_setpoint_pos_        (x,y,z)           [ctrl target in FW]
% 27    : final_setpoint_yaw_deg_    [deg]
% 28    : su_cmd_fx_                 [force command from FW param su_cmd.cmd_fx]
% 29-31 : world_Fext_MOB             (MOB World 힘 추정값 [N])
% 32-34 : world_Fext_DOB             (DOB World 힘 추정값 [N])
% 35-37 : vel_from_pos               (suVelFromPos vx,vy,vz [m/s])
% 38-40 : rpy_kalman                 (roll,pitch,yaw) [rad]
% 41-43 : rpy_comp                   (roll,pitch,yaw) [rad]

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

% 0-2
force_global_raw(:,1:3) = values(:,1:3);

% 3-5
position_meas(:,1:3) = values(:,4:6);

% 6-8
rpy_meas(:,1:3) = values(:,7:9);

% 9-11
acc_global(:,1:3) = values(:,10:12);

% 12-14
vel_global(:,1:3) = values(:,13:15);

% 15-16
vbat(:,1) = values(:,16);
vbat(:,2) = values(:,17);

% 17-19
cmd_position(:,1:3) = values(:,18:20);

% 20
cmd_yaw_deg = values(:,21);
cmd_yaw     = cmd_yaw_deg * pi/180;

% 21-23
force_global_scaled(:,1:3) = values(:,22:24);

% 24-26
final_cmd_position(:,1:3) = values(:,25:27);

% 27
final_cmd_yaw_deg = values(:,28);
final_cmd_yaw_rad = final_cmd_yaw_deg * pi/180;

% 28
cmd_x_force(:,1) = values(:,29);

% 29-31
world_Fext_MOB(:,1:3) = values(:,30:32);

% 32-34
world_Fext_DOB(:,1:3) = values(:,33:35);

% 35-37
vel_from_pos(:,1:3) = values(:,36:38);

% 38-40
rpy_kalman(:,1:3) = values(:,39:41);

% 41-43
rpy_comp(:,1:3) = values(:,42:44);

%% Optional smoothing
force_global_scaled_filt = movmean(force_global_scaled, 7, 1);

%% ---- Dashboard figure ----
twin = [time(1) time(end)];

f = figure('Name','DataLogging Overview','NumberTitle','off', ...
           'Color','w','Units','normalized','Position',[0 0 1 1]);

left=0.04; right=0.02; top=0.04; bottom=0.06;
hgap=0.03; vgap=0.05;
ncol=3; nrow=2;
w = (1-left-right-hgap*(ncol-1))/ncol;
h = (1-top-bottom-vgap*(nrow-1))/nrow;
getPos=@(row,col)[ left+(col-1)*(w+hgap), ...
                   1-top-row*h-(row-1)*vgap, ...
                   w, h ];

% (1,1) Position cmd vs target vs meas
p11 = uipanel('Parent',f,'Position',getPos(1,1),'BackgroundColor','w');
tl11 = tiledlayout(p11,3,1,'TileSpacing','compact','Padding','compact');

nexttile(tl11,1);
plot(time, cmd_position(:,1), 'g:','LineWidth',1.0); hold on;
plot(time, final_cmd_position(:,1), 'r--','LineWidth',1.2);
plot(time, position_meas(:,1), 'b-','LineWidth',1.2);
grid on; xlim(twin);
title('X pos (ROS cmd / FW target / meas)');
legend('ROS cmd','FW target','meas','Location','best');

nexttile(tl11,2);
plot(time, cmd_position(:,2), 'g:','LineWidth',1.0); hold on;
plot(time, final_cmd_position(:,2), 'r--','LineWidth',1.2);
plot(time, position_meas(:,2), 'b-','LineWidth',1.2);
grid on; xlim(twin);
title('Y pos (ROS cmd / FW target / meas)');

nexttile(tl11,3);
plot(time, cmd_position(:,3), 'g:','LineWidth',1.0); hold on;
plot(time, final_cmd_position(:,3), 'r--','LineWidth',1.2);
plot(time, position_meas(:,3), 'b-','LineWidth',1.2);
grid on; xlim(twin);
title('Z pos (ROS cmd / FW target / meas)');
xlabel('time [s]');

% (1,2) Velocity EKF
p12 = uipanel('Parent',f,'Position',getPos(1,2),'BackgroundColor','w');
tl12 = tiledlayout(p12,3,1,'TileSpacing','compact','Padding','compact');

nexttile(tl12,1);
plot(time, vel_global(:,1),'LineWidth',1.2); grid on; xlim(twin);
title('vel x_{world} [m/s] (EKF)');

nexttile(tl12,2);
plot(time, vel_global(:,2),'LineWidth',1.2); grid on; xlim(twin);
title('vel y_{world} [m/s] (EKF)');

nexttile(tl12,3);
plot(time, vel_global(:,3),'LineWidth',1.2); grid on; xlim(twin);
title('vel z_{world} [m/s] (EKF)'); xlabel('time [s]');

% (1,3) Attitude meas + FW cmd yaw
p13 = uipanel('Parent',f,'Position',getPos(1,3),'BackgroundColor','w');
tl13 = tiledlayout(p13,3,1,'TileSpacing','compact','Padding','compact');

nexttile(tl13,1);
plot(time, rpy_meas(:,1),'LineWidth',1.2); grid on; xlim(twin);
title('roll [rad] (meas)');

nexttile(tl13,2);
plot(time, rpy_meas(:,2),'LineWidth',1.2); grid on; xlim(twin);
title('pitch [rad] (meas)');

nexttile(tl13,3);
plot(time, rpy_meas(:,3),'LineWidth',1.2); hold on;
plot(time, final_cmd_yaw_rad,'k-','LineWidth',1.2);
grid on; xlim(twin);
title('yaw (meas / FW cmd) [rad]');
legend('meas','FW cmd','Location','best');
xlabel('time [s]');

% (2,1) Force scaled
p21 = uipanel('Parent',f,'Position',getPos(2,1),'BackgroundColor','w');
tl21 = tiledlayout(p21,3,1,'TileSpacing','compact','Padding','compact');

nexttile(tl21,1);
plot(time, force_global_scaled(:,1),'LineWidth',1.2); grid on; xlim(twin);
title('F_x^{scaled} (world) [arb/N]');

nexttile(tl21,2);
plot(time, force_global_scaled(:,2),'LineWidth',1.2); grid on; xlim(twin);
title('F_y^{scaled} (world) [arb/N]');

nexttile(tl21,3);
plot(time, force_global_scaled(:,3),'LineWidth',1.2); grid on; xlim(twin);
title('F_z^{scaled} (world) [arb/N]'); xlabel('time [s]');

% (2,2) m*a vs Force scaled
p22 = uipanel('Parent',f,'Position',getPos(2,2),'BackgroundColor','w');
tl22 = tiledlayout(p22,3,1,'TileSpacing','compact','Padding','compact');

mass_dash = 0.04;
g_dash    = 9.81;

ma_x = acc_global(:,1) * mass_dash * g_dash;
ma_y = acc_global(:,2) * mass_dash * g_dash;
ma_z = acc_global(:,3) * mass_dash * g_dash;

nexttile(tl22,1);
plot(time, ma_x,'LineWidth',1.2); hold on;
plot(time, force_global_scaled(:,1),'LineWidth',1.2);
grid on; xlim(twin);
title('X: m a_x vs F_x^{scaled}');
legend('m a_x','F_x^{scaled}','Location','best');

nexttile(tl22,2);
plot(time, ma_y,'LineWidth',1.2); hold on;
plot(time, force_global_scaled(:,2),'LineWidth',1.2);
grid on; xlim(twin);
title('Y: m a_y vs F_y^{scaled}');

nexttile(tl22,3);
plot(time, ma_z + mass_dash * g_dash,'LineWidth',1.2); hold on;
plot(time, force_global_scaled(:,3),'LineWidth',1.2);
grid on; xlim(twin);
title('Z: m a_z + mg vs F_z^{scaled}');
xlabel('time [s]');

% (2,3) F_ext MOB vs DOB
p23 = uipanel('Parent',f,'Position',getPos(2,3),'BackgroundColor','w');
tl23 = tiledlayout(p23,3,1,'TileSpacing','compact','Padding','compact');

nexttile(tl23,1);
plot(time, world_Fext_MOB(:,1),'LineWidth',1.2); hold on;
plot(time, world_Fext_DOB(:,1),'--','LineWidth',1.2);
grid on; xlim(twin);
title('F_{ext,x} [N]');
legend('MOB','DOB','Location','best');

nexttile(tl23,2);
plot(time, world_Fext_MOB(:,2),'LineWidth',1.2); hold on;
plot(time, world_Fext_DOB(:,2),'--','LineWidth',1.2);
grid on; xlim(twin);
title('F_{ext,y} [N]');

nexttile(tl23,3);
plot(time, world_Fext_MOB(:,3),'LineWidth',1.2); hold on;
plot(time, world_Fext_DOB(:,3),'--','LineWidth',1.2);
grid on; xlim(twin);
title('F_{ext,z} [N]');
xlabel('time [s]');

set(findall(f,'Type','axes'),'FontSize',9,'Color','w');

%% 6. Attitude detail
figure('Name','Attitude detail (meas + cmd)','NumberTitle','off','Color','w');
subplot(3,1,1); plot(time, rpy_meas(:,1),'LineWidth',1.4); grid on; title('roll [rad]');
subplot(3,1,2); plot(time, rpy_meas(:,2),'LineWidth',1.4); grid on; title('pitch [rad]');
subplot(3,1,3);
plot(time, rpy_meas(:,3),'LineWidth',1.4); hold on;
plot(time, final_cmd_yaw_rad,'k-','LineWidth',1.4);
grid on; xlabel('time [s]'); title('yaw [rad]'); legend('meas','cmd','Location','best');

%% 7. Position detail
figure('Name','Position detail','NumberTitle','off','Color','w');
subplot(3,1,1);
plot(time, final_cmd_position(:,1),'k-','LineWidth',1.4); hold on;
plot(time, position_meas(:,1),'b-','LineWidth',1.2);
grid on; title('X'); legend('cmd x','meas x');

subplot(3,1,2);
plot(time, final_cmd_position(:,2),'k-','LineWidth',1.4); hold on;
plot(time, position_meas(:,2),'b-','LineWidth',1.2);
grid on; title('Y'); legend('cmd y','meas y');

subplot(3,1,3);
plot(time, final_cmd_position(:,3),'k-','LineWidth',1.4); hold on;
plot(time, position_meas(:,3),'b-','LineWidth',1.2);
grid on; title('Z'); legend('cmd z','meas z');

%% 13. Velocity detail
figure('Name','Velocity (world, EKF) detail','NumberTitle','off','Color','w');
subplot(3,1,1); plot(time, vel_global(:,1),'LineWidth',1.4); grid on; title('vel X_{world} [m/s] (EKF)');
subplot(3,1,2); plot(time, vel_global(:,2),'LineWidth',1.4); grid on; title('vel Y_{world} [m/s] (EKF)');
subplot(3,1,3); plot(time, vel_global(:,3),'LineWidth',1.4); grid on; title('vel Z_{world} [m/s] (EKF)'); xlabel('time [s]');

%% 8. Force scale fitting (hover window)
mass = 0.04;
g    = 9.81;
time_start = 28;
time_end   = 86;

idx = (time >= time_start) & (time <= time_end);

v_seg  = vbat(idx, 2);
Fz_seg = force_global_raw(idx, 3);

valid  = abs(Fz_seg) > 1e-3 & isfinite(v_seg) & isfinite(Fz_seg);
v_seg  = v_seg(valid);
Fz_seg = Fz_seg(valid);

if numel(Fz_seg) < 10
    warning("Force scale fit: too few valid samples in the window. Skipping fit.");
    force_scale = ones(size(vbat(:,2)));
else
    y = mass * g * ones(size(Fz_seg));
    X = [v_seg .* Fz_seg, Fz_seg];
    theta = X \ y;
    a = theta(1); b = theta(2);
    force_scale = a * vbat(:,2) + b;

    fprintf('\n[Force scale fitting result]\n');
    fprintf('Time window: %.1f ~ %.1f s\n', time_start, time_end);
    fprintf('Mass: %.3f kg, g: %.2f m/s^2\n', mass, g);
    fprintf('----------------------------------\n');
    fprintf('a = %.6f\n', a);
    fprintf('b = %.6f\n\n', b);
end

%% 9. Force detail + scaling
figure('Name','Force detail','NumberTitle','off','Color','w');
subplot(3,2,1); plot(time, force_global_raw(:,1),'LineWidth',1.4); grid on; title('Fx raw [N]');
subplot(3,2,3); plot(time, force_global_raw(:,2),'LineWidth',1.4); grid on; title('Fy raw [N]');
subplot(3,2,5); plot(time, force_global_raw(:,3),'LineWidth',1.4); grid on; title('Fz raw [N]'); xlabel('time [s]');

subplot(3,2,2); plot(time, force_scale .* force_global_raw(:,1),'LineWidth',1.4); grid on; title('Fx scaled [N]');
subplot(3,2,4); plot(time, force_scale .* force_global_raw(:,2),'LineWidth',1.4); grid on; title('Fy scaled [N]');
subplot(3,2,6); plot(time, force_scale .* force_global_raw(:,3),'LineWidth',1.4); grid on; title('Fz scaled [N]'); xlabel('time [s]');

%% 10. F=ma check
figure('Name','Acceleration vs Force','NumberTitle','off','Color','w');
subplot(3,1,1);
plot(time, acc_global(:,1) * mass * g,'LineWidth',1.4); grid on; hold on;
plot(time, force_global_scaled(:,1),'LineWidth',1.4);
plot(time, force_scale .* force_global_raw(:,1),'LineWidth',1.4);
title('X axis (m a_{x,world} vs force)');
legend('m a_x','FW scaled','F^{(vbat) scaled}','Location','best');

subplot(3,1,2);
plot(time, acc_global(:,2) * mass * g,'LineWidth',1.4); grid on; hold on;
plot(time, force_global_scaled(:,2),'LineWidth',1.4);
plot(time, force_scale .* force_global_raw(:,2),'LineWidth',1.4);
title('Y axis (m a_{y,world} vs force)');

subplot(3,1,3);
plot(time, acc_global(:,3) * mass * g + mass * g,'LineWidth',1.4); grid on; hold on;
plot(time, force_global_scaled(:,3),'LineWidth',1.4);
plot(time, force_scale .* force_global_raw(:,3),'LineWidth',1.4);
title('Z axis (m (a_{z,world} + g) vs force)');
xlabel('time [s]');

%% 12. Wrench Observation (MOB/DOB)
figure('Name','Force Estimation','NumberTitle','off','Color','w');

subplot(3,1,1);
plot(time, world_Fext_MOB(:,1),'LineWidth',1.4); hold on;
plot(time, world_Fext_DOB(:,1),'--','LineWidth',1.4);
grid on; title('F_{ext,x}^{MOB/DOB}');
legend('MOB','DOB','Location','best');

subplot(3,1,2);
plot(time, world_Fext_MOB(:,2),'LineWidth',1.4); hold on;
plot(time, world_Fext_DOB(:,2),'--','LineWidth',1.4);
grid on; title('F_{ext,y}^{MOB/DOB}');

subplot(3,1,3);
plot(time, world_Fext_MOB(:,3),'LineWidth',1.4); hold on;
plot(time, world_Fext_DOB(:,3),'--','LineWidth',1.4);
grid on; title('F_{ext,z}^{MOB/DOB}');
xlabel('time [s]');

%% 12b. Voltage & Force command
figure('Name','Voltage & Force command','NumberTitle','off','Color','w');
subplot(3,1,1); plot(time, vbat(:,1),'LineWidth',1.4); grid on; title('vbat raw [V]');
subplot(3,1,2); plot(time, vbat(:,2),'LineWidth',1.4); grid on; title('vbat filtered [V]'); xlabel('time [s]');
subplot(3,1,3); plot(time, cmd_x_force(:,1),'LineWidth',1.4); grid on; title('Force command su\_cmd\_fx'); xlabel('time [s]');

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
