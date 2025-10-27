%% 
clear all;
clc;

% Select the configuration file to use
% 'loop', 'roll', or 'straight'
config_to_run = 'loop';

run(['trajectory_configs/config_' config_to_run '.m']);

addpath('DFL_controller');
addpath('models');
addpath('utilities');
addpath('trajectory_configs');

%% Define global variables for quadrotor controller
global m Ix Iy Iz g Ax Ay Az Ap Aq Ar
m = quad_params.m;
Ix = quad_params.Ix;
Iy = quad_params.Iy;
Iz = quad_params.Iz;
g = quad_params.g;
Ax = quad_params.Ax;
Ay = quad_params.Ay;
Az = quad_params.Az;
Ap = quad_params.Ap;
Aq = quad_params.Aq;
Ar = quad_params.Ar;

%% Initial Conditions
% Fixed-wing
fw_x0 = fw_initial.x0;

% Quadrotor
% Start the quadrotor at the same position and velocity as the fixed-wing for a smoother start.
x0_quad = fw_x0(1) -0.01; y0_quad = fw_x0(2) -0.001 ; z0_quad = fw_x0(3)-0.001;
q0_quad = 1; q1_quad = 0; q2_quad = 0; q3_quad = 0;
u0_quad = fw_initial.u0; v0_quad = fw_initial.v0; w0_quad = fw_initial.w0;
p_quad = 0; q_quad = 0; r_quad = 0;
zeta = quad_params.m*quad_params.g; xi = 0;
phi_g = 0; theta_g = 0;
% phi_g_dot and theta_g_dot are no longer states
quad_initial_state = [x0_quad; y0_quad; z0_quad; q0_quad; q1_quad; q2_quad; q3_quad; u0_quad; v0_quad; w0_quad; p_quad; q_quad; r_quad; phi_g; theta_g; zeta; xi];

% Combined state vector
initial_state = [quad_initial_state; fw_x0];

%% Simulation
clear unified_dynamics; % Clear persistent variables
clear quadrotor_dynamics_realtime; % Clear persistent variables
ode_options = odeset('RelTol', 1e-4, 'AbsTol', 1e-4);
[t, state] = ode45(@(t,s) unified_dynamics(t, s, fw_params, fw_controls, dfl_gains), t_sim, initial_state, ode_options);

%% Post-processing and Plotting
% Unpack states
quad_state = state(:, 1:17);
fw_state = state(:, 18:30);

% Extract quadrotor and fixed-wing positions
x_quad = quad_state(:, 1:3);
x_fw = fw_state(:, 1:3);

% Plot 3D Trajectory (NED Frame)
% Save plots
fig1 = figure('Name', '3D Trajectory Tracking (NED Frame)', 'NumberTitle', 'off');
% Plot with Z-axis inverted to show altitude upwards
p1 = plot3(x_quad(:,1), x_quad(:,2), -x_quad(:,3), 'b', 'LineWidth', 1.5);
p1.Color(4) = 0.5; % Set transparency
hold on;
p2 = plot3(x_fw(:,1), x_fw(:,2), -x_fw(:,3), 'r--', 'LineWidth', 1.5);
p2.Color(4) = 0.5; % Set transparency
grid on;
title(['Quadrotor Tracking Fixed-Wing Trajectory (' config_to_run ')']);
xlabel('North (m)');
ylabel('East (m)');
zlabel('Altitude (m)');
axis equal;
view(45, 25);

legend_added = false;
legend_added_stl = false;

% Load STL model for the fixed-wing aircraft
[F, V, ~] = stlread('CAD/aero.stl');
V = V - mean(V); % Center the model
V = V / 20;
% Define a scaling factor for the model size
model_scale = 0.7; % Adjust this value to make the model larger or smaller
V = V * model_scale;

% Apply a corrective rotation to align the STL model with the body frame
% (x-forward, y-right, z-down). This is a common correction needed when
% the STL model is created in a different coordinate system (e.g., y-forward, z-up).
yaw_angle = -pi; % Rotate -180 degrees around Z
R_yaw = [cos(yaw_angle) -sin(yaw_angle) 0;
         sin(yaw_angle)  cos(yaw_angle) 0;
         0               0              1];
pitch_angle = 20 * pi / 180; % Pitch down 20 degrees
R_pitch = [cos(pitch_angle) 0 sin(pitch_angle);
           0                1 0;
           -sin(pitch_angle) 0 cos(pitch_angle)];
R_correction = R_pitch * R_yaw;
V = (R_correction * V')';



% Apply a 180-degree roll rotation to align STL coordinate system with body frame
%roll_angle = pi; % 180 degrees
%R_roll_180 = [1 0 0;
%              0 cos(roll_angle) -sin(roll_angle);
%              0 sin(roll_angle) cos(roll_angle)];
%V = (R_roll_180 * V')';

% Add vector plots for forward directions and the STL model
for i = 1:10:length(t) % Plot every 50th point to avoid clutter
    % Fixed-wing position and orientation
    pos_fw = [x_fw(i,1), x_fw(i,2), -x_fw(i,3)];
    q_fw = fw_state(i, 7:10); % [q0, q1, q2, q3]
    q0=q_fw(1); q1=q_fw(2); q2=q_fw(3); q3=q_fw(4);
    R_fw = [1 - 2*(q2^2 + q3^2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
            2*(q1*q2 + q0*q3), 1 - 2*(q1^2 + q3^2), 2*(q2*q3 - q0*q1);
            2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1^2 + q2^2)];
    
    % Rotate and translate vertices of the STL model
    V_rotated = (R_fw' * V')';
    V_translated = V_rotated + pos_fw;
    
    % Plot aircraft model
    if ~legend_added_stl
        p_stl = patch('Faces', F, 'Vertices', V_translated, 'FaceColor', [0.8 0.2 0.2], 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'FaceAlpha', 0.1, 'DisplayName', 'Ghost FW aeroplane');
        legend_added_stl = true;
    else
        patch('Faces', F, 'Vertices', V_translated, 'FaceColor', [0.8 0.2 0.2], 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'FaceAlpha', 0.1, 'HandleVisibility','off');
    end

    % Drone gimbal axis
    q_quad = quad_state(i, 4:7); % [q0, q1, q2, q3]
    q0=q_quad(1); q1=q_quad(2); q2=q_quad(3); q3=q_quad(4);
    R_quad = [1 - 2*(q2^2 + q3^2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
              2*(q1*q2 + q0*q3), 1 - 2*(q1^2 + q3^2), 2*(q2*q3 - q0*q1);
              2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1^2 + q2^2)];
              
    phi_g = quad_state(i, 14);
    theta_g = quad_state(i, 15);
    
    % This assumes a Z-Y rotation sequence for the gimbal, which is consistent
    % with how the reference angles are now calculated in the controller.
    R_gb = [cos(phi_g)*cos(theta_g), -sin(phi_g), cos(phi_g)*sin(theta_g);
            sin(phi_g)*cos(theta_g),  cos(phi_g), sin(phi_g)*sin(theta_g);
           -sin(theta_g),                 0,       cos(theta_g)];
    R_gimbal_w = R_quad * R_gb;

    % Plot gimbal frame basis vectors
    scale_gimbal = 2.5; % Make arrows slightly longer than drone axis
    line_width_gimbal = 1.5; % Make arrows slightly thinner than drone axis
    dark_red = [0.6, 0, 0];
    dark_green = [0, 0.6, 0];
    dark_blue = [0, 0, 0.6];
    pos_quad = [x_quad(i,1), x_quad(i,2), -x_quad(i,3)];
    gimbal_x = R_gimbal_w(:,1) * scale_gimbal;
    gimbal_y = R_gimbal_w(:,2) * scale_gimbal;
    gimbal_z = R_gimbal_w(:,3) * scale_gimbal;

    quiver3(pos_quad(1), pos_quad(2), pos_quad(3), gimbal_x(1), gimbal_x(2), -gimbal_x(3), 'Color', dark_red, 'LineWidth', line_width_gimbal, 'MaxHeadSize', 0.5, 'ShowArrowHead', 'on', 'HandleVisibility','off');
    quiver3(pos_quad(1), pos_quad(2), pos_quad(3), gimbal_y(1), gimbal_y(2), -gimbal_y(3), 'Color', dark_green, 'LineWidth', line_width_gimbal, 'MaxHeadSize', 0.5, 'ShowArrowHead', 'on', 'HandleVisibility','off');
    quiver3(pos_quad(1), pos_quad(2), pos_quad(3), gimbal_z(1), gimbal_z(2), -gimbal_z(3), 'Color', dark_blue, 'LineWidth', line_width_gimbal, 'MaxHeadSize', 0.5, 'ShowArrowHead', 'on', 'HandleVisibility','off');

    % Plot drone body frame
    scale_drone = 2; % Make arrows shorter
    drone_x = R_quad(:,1) * scale_drone;
    drone_y = R_quad(:,2) * scale_drone;
    drone_z = R_quad(:,3) * scale_drone;
    
    offset = [0, 0, 2]; % Offset from the drone's center
    pos_quad_offset = pos_quad + offset;

    quiver3(pos_quad_offset(1), pos_quad_offset(2), pos_quad_offset(3), drone_x(1), drone_x(2), -drone_x(3), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'ShowArrowHead', 'on', 'HandleVisibility','off');
    quiver3(pos_quad_offset(1), pos_quad_offset(2), pos_quad_offset(3), drone_y(1), drone_y(2), -drone_y(3), 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'ShowArrowHead', 'on', 'HandleVisibility','off');
    quiver3(pos_quad_offset(1), pos_quad_offset(2), pos_quad_offset(3), drone_z(1), drone_z(2), -drone_z(3), 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'ShowArrowHead', 'on', 'HandleVisibility','off');
    
    % Plot vectors
    if ~legend_added
        legend([p1, p2, p_stl], 'Quadrotor Path', 'Fixed-Wing Path', 'Ghost FW aeroplane');
        legend_added = true;
    end
end
light('Position',[1 0 0],'Style','infinite');
light('Position',[-1 0 0],'Style','infinite');
saveas(fig1, 'results/3d_trajectory.pdf');


% Gimbal Angles (raw from state)
phi_g_raw = quad_state(:, 14);
theta_g_raw = quad_state(:, 15);

% --- Calculate Reference Angles and Errors Post-Simulation ---
phi_g_ref_history = zeros(length(t), 1);
theta_g_ref_history = zeros(length(t), 1);

% New history vectors for full orientation tracking
gimbal_global_roll_hist = zeros(length(t), 1);
gimbal_global_pitch_hist = zeros(length(t), 1);
gimbal_global_yaw_hist = zeros(length(t), 1);
fw_global_roll_hist = zeros(length(t), 1);
fw_global_pitch_hist = zeros(length(t), 1);
fw_global_yaw_hist = zeros(length(t), 1);

last_phi_g_ref = 0; % Initialize for post-processing unwrapping
last_theta_g_ref = 0;

% Initialize last angles for smoothing global plots
last_gimbal_roll = 0; last_gimbal_pitch = 0; last_gimbal_yaw = 0;
last_fw_roll = 0; last_fw_pitch = 0; last_fw_yaw = 0;
% Initialize last angles for smoothing body-frame plots
last_phi_g = 0; last_theta_g = 0;

for i = 1:length(t)
    % Quadrotor orientation
    q_bw = quad_state(i, 4:7);
    q0=q_bw(1); q1=q_bw(2); q2=q_bw(3); q3=q_bw(4);
    R_bw = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
            2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
            2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];

    % --- NEW: Full Orientation Tracking Post-Processing ---
    
    % Get fixed-wing orientation in world frame
    q_fw = fw_state(i, 7:10);
    q0_fw=q_fw(1); q1_fw=q_fw(2); q2_fw=q_fw(3); q3_fw=q_fw(4);
    R_fw_w = [q0_fw^2+q1_fw^2-q2_fw^2-q3_fw^2, 2*(q1_fw*q2_fw-q0_fw*q3_fw), 2*(q1_fw*q3_fw+q0_fw*q2_fw);
              2*(q1_fw*q2_fw+q0_fw*q3_fw), q0_fw^2-q1_fw^2+q2_fw^2-q3_fw^2, 2*(q2_fw*q3_fw-q0_fw*q1_fw);
              2*(q1_fw*q3_fw-q0_fw*q2_fw), 2*(q2_fw*q3_fw+q0_fw*q1_fw), q0_fw^2-q1_fw^2-q2_fw^2+q3_fw^2];

    % Calculate gimbal's orientation in world frame
    phi_g_actual = quad_state(i, 14);
    theta_g_actual = quad_state(i, 15);
    % This assumes a Z-Y rotation sequence for the gimbal, which is consistent
    % with how the reference angles are now calculated in the controller.
    R_gb = [cos(phi_g_actual)*cos(theta_g_actual), -sin(phi_g_actual), cos(phi_g_actual)*sin(theta_g_actual);
            sin(phi_g_actual)*cos(theta_g_actual),  cos(phi_g_actual), sin(phi_g_actual)*sin(theta_g_actual);
           -sin(theta_g_actual),                 0,                cos(theta_g_actual)];
    R_gimbal_w = R_bw * R_gb;

    % --- Angle Smoothing Logic ---
    jump_threshold = 170 * pi / 180;

    % Raw angle calculations
    gimbal_yaw_raw = atan2(R_gimbal_w(2,1), R_gimbal_w(1,1));
    gimbal_pitch_raw = asin(-R_gimbal_w(3,1));
    gimbal_roll_raw = atan2(R_gimbal_w(3,2), R_gimbal_w(3,3));
    fw_yaw_raw = atan2(R_fw_w(2,1), R_fw_w(1,1));
    fw_pitch_raw = asin(-R_fw_w(3,1));
    fw_roll_raw = atan2(R_fw_w(3,2), R_fw_w(3,3));

    if i == 1
        last_gimbal_yaw = gimbal_yaw_raw;
        last_gimbal_pitch = gimbal_pitch_raw;
        last_gimbal_roll = gimbal_roll_raw;
        last_fw_yaw = fw_yaw_raw;
        last_fw_pitch = fw_pitch_raw;
        last_fw_roll = fw_roll_raw;
    end

    % Smooth and store gimbal angles
    delta = gimbal_yaw_raw - last_gimbal_yaw; delta = mod(delta + pi, 2*pi) - pi;
    if abs(delta) < jump_threshold, last_gimbal_yaw = last_gimbal_yaw + delta; end
    gimbal_global_yaw_hist(i) = last_gimbal_yaw;

    delta = gimbal_pitch_raw - last_gimbal_pitch; delta = mod(delta + pi, 2*pi) - pi;
    if abs(delta) < jump_threshold, last_gimbal_pitch = last_gimbal_pitch + delta; end
    gimbal_global_pitch_hist(i) = last_gimbal_pitch;

    delta = gimbal_roll_raw - last_gimbal_roll; delta = mod(delta + pi, 2*pi) - pi;
    if abs(delta) < jump_threshold, last_gimbal_roll = last_gimbal_roll + delta; end
    gimbal_global_roll_hist(i) = last_gimbal_roll;

    % Smooth and store fixed-wing angles
    delta = fw_yaw_raw - last_fw_yaw; delta = mod(delta + pi, 2*pi) - pi;
    if abs(delta) < jump_threshold, last_fw_yaw = last_fw_yaw + delta; end
    fw_global_yaw_hist(i) = last_fw_yaw;

    delta = fw_pitch_raw - last_fw_pitch; delta = mod(delta + pi, 2*pi) - pi;
    if abs(delta) < jump_threshold, last_fw_pitch = last_fw_pitch + delta; end
    fw_global_pitch_hist(i) = last_fw_pitch;

    delta = fw_roll_raw - last_fw_roll; delta = mod(delta + pi, 2*pi) - pi;
    if abs(delta) < jump_threshold, last_fw_roll = last_fw_roll + delta; end
    fw_global_roll_hist(i) = last_fw_roll;

    % --- Calculate Body-Frame Reference Angles (for original plots) ---
    R_gb_ref = R_bw' * R_fw_w;
    theta_g_ref_raw = asin(-R_gb_ref(3,1));
    phi_g_ref_raw = atan2(R_gb_ref(2,1), R_gb_ref(1,1));
    
    if i == 1
        last_phi_g_ref = phi_g_ref_raw;
        last_theta_g_ref = theta_g_ref_raw;
    end
    
    delta_phi = phi_g_ref_raw - last_phi_g_ref;
    delta_phi = mod(delta_phi + pi, 2*pi) - pi;
    if abs(delta_phi) > (170 * pi / 180)
        phi_g_ref = last_phi_g_ref;
    else
        phi_g_ref = last_phi_g_ref + delta_phi;
    end
    last_phi_g_ref = phi_g_ref;

    delta_theta = theta_g_ref_raw - last_theta_g_ref;
    delta_theta = mod(delta_theta + pi, 2*pi) - pi;
    if abs(delta_theta) > (170 * pi / 180)
        theta_g_ref = last_theta_g_ref;
    else
        theta_g_ref = last_theta_g_ref + delta_theta;
    end
    last_theta_g_ref = theta_g_ref;
    
    phi_g_ref_history(i) = phi_g_ref;
    theta_g_ref_history(i) = theta_g_ref;

    % --- Smooth Actual Body-Frame Gimbal Angles ---
    if i == 1
        last_phi_g = phi_g_raw(i);
        last_theta_g = theta_g_raw(i);
    end
    delta = phi_g_raw(i) - last_phi_g; delta = mod(delta + pi, 2*pi) - pi;
    if abs(delta) < jump_threshold, last_phi_g = last_phi_g + delta; end
    phi_g(i) = last_phi_g;

    delta = theta_g_raw(i) - last_theta_g; delta = mod(delta + pi, 2*pi) - pi;
    if abs(delta) < jump_threshold, last_theta_g = last_theta_g + delta; end
    theta_g(i) = last_theta_g;
end

% --- Combined Position and Orientation Plot ---
fig_combined = figure('Name', 'Position and Orientation Tracking', 'NumberTitle', 'off');
set(fig_combined, 'Position', [100, 100, 800, 1200]); % Adjust figure size

% X Position
subplot(3,2,1);
plot(t, x_quad(:,1), 'b', 'LineWidth', 1.5);
hold on;
plot(t, x_fw(:,1), 'r--', 'LineWidth', 1.5);
grid on;
title('X Position');
ylabel('Position (m)');
legend('Gimbal', 'Fixed-Wing');

% Y Position
subplot(3,2,3);
plot(t, x_quad(:,2), 'b', 'LineWidth', 1.5);
hold on;
plot(t, x_fw(:,2), 'r--', 'LineWidth', 1.5);
grid on;
title('Y Position');
ylabel('Position (m)');
legend('Gimbal', 'Fixed-Wing');

% Z Position
subplot(3,2,5);
plot(t, -x_quad(:,3), 'b', 'LineWidth', 1.5);
hold on;
plot(t, -x_fw(:,3), 'r--', 'LineWidth', 1.5);
grid on;
title('Z Position (Altitude)');
xlabel('Time (s)');
ylabel('Position (m)');
legend('Gimbal', 'Fixed-Wing');

% Roll Angle
subplot(3,2,2);
plot(t, gimbal_global_roll_hist * 180/pi, 'b', 'LineWidth', 1.5);
hold on;
plot(t, fw_global_roll_hist * 180/pi, 'r--', 'LineWidth', 1.5);
grid on;
title('Global Roll Angle');
ylabel('Angle (degrees)');
legend('Gimbal', 'Fixed-Wing');

% Pitch Angle
subplot(3,2,4);
plot(t, gimbal_global_pitch_hist * 180/pi, 'b', 'LineWidth', 1.5);
hold on;
plot(t, fw_global_pitch_hist * 180/pi, 'r--', 'LineWidth', 1.5);
grid on;
title('Global Pitch Angle');
ylabel('Angle (degrees)');
legend('Gimbal', 'Fixed-Wing');

% Yaw Angle
subplot(3,2,6);
plot(t, gimbal_global_yaw_hist * 180/pi, 'b', 'LineWidth', 1.5);
hold on;
plot(t, fw_global_yaw_hist * 180/pi, 'r--', 'LineWidth', 1.5);
grid on;
title('Global Yaw Angle');
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('Gimbal', 'Fixed-Wing');

saveas(fig_combined, 'results/position_and_orientation.pdf');

% --- Control Input Plots ---
control_history = quadrotor_dynamics_realtime('get_history');
time_hist = control_history(:, 1);
thrust_hist = control_history(:, 2);
u_p_hist = control_history(:, 3);
u_q_hist = control_history(:, 4);
u_r_hist = control_history(:, 5);
p_hist = control_history(:, 6);
q_hist = control_history(:, 7);
r_hist = control_history(:, 8);
u_phi_g_hist = control_history(:, 9);
u_theta_g_hist = control_history(:, 10);

fig_control_inputs = figure('Name', 'Control Inputs', 'NumberTitle', 'off');
set(fig_control_inputs, 'Position', [100, 100, 800, 1000]); % Adjust figure size

subplot(3,2,1);
plot(time_hist, thrust_hist, 'k', 'LineWidth', 1.5);
grid on;
title('Total Thrust');
ylabel('Force (N)');

subplot(3,2,2);
plot(time_hist, u_p_hist, 'r', 'LineWidth', 1.5);
grid on;
title('Roll Control Moment');
ylabel('Nm');
legend('u_p');

subplot(3,2,3);
plot(time_hist, u_q_hist, 'g', 'LineWidth', 1.5);
grid on;
title('Pitch Control Moment');
ylabel('Nm');
legend('u_q');

subplot(3,2,4);
plot(time_hist, u_r_hist, 'b', 'LineWidth', 1.5);
grid on;
title('Yaw Control Moment');
ylabel('Nm');
legend('u_r');

subplot(3,2,5);
plot(time_hist, u_phi_g_hist, 'm', 'LineWidth', 1.5);
grid on;
title('Gimbal Roll Control Input');
xlabel('Time (s)');
ylabel('rad/s');

subplot(3,2,6);
plot(time_hist, u_theta_g_hist, 'c', 'LineWidth', 1.5);
grid on;
title('Gimbal Pitch Control Input');
xlabel('Time (s)');
ylabel('rad/s');

saveas(fig_control_inputs, 'results/control_inputs.pdf');


disp('Real-time simulation and plotting complete.');
