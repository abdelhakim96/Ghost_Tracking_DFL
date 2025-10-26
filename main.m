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
x0_quad = fw_x0(1) -0.1; y0_quad = fw_x0(2) -0.001 ; z0_quad = fw_x0(3)-0.001;
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
model_scale = 0.5; % Adjust this value to make the model larger or smaller
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

% --- Create a filtered copy of gimbal angles for 3D plotting ---
phi_g_for_3d = unwrap(quad_state(:, 14));
theta_g_for_3d = unwrap(quad_state(:, 15));

% Add vector plots for forward directions and the STL model
for i = 1:8:length(t) % Plot every 50th point to avoid clutter
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

    % Gimbal forward vector (actual pointing direction)
    q_quad = quad_state(i, 4:7); % [q0, q1, q2, q3]
    q0=q_quad(1); q1=q_quad(2); q2=q_quad(3); q3=q_quad(4);
    R_quad = [1 - 2*(q2^2 + q3^2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
              2*(q1*q2 + q0*q3), 1 - 2*(q1^2 + q3^2), 2*(q2*q3 - q0*q1);
              2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1^2 + q2^2)];
              
    % Use filtered angles for 3D plot
    phi_g_i = phi_g_for_3d(i);
    theta_g_i = theta_g_for_3d(i);
    
    % Plot Gimbal body frame basis vectors
    c_p = cos(phi_g_i); s_p = sin(phi_g_i);
    c_t = cos(theta_g_i); s_t = sin(theta_g_i);
    
    R_gb = [ c_p*c_t, -s_p, c_p*s_t;
             s_p*c_t,  c_p, s_p*s_t;
                -s_t,    0,     c_t ];
    
    R_gimbal_world = R_quad * R_gb;
    
    pos_quad = [x_quad(i,1), x_quad(i,2), -x_quad(i,3)];
    scale_gimbal = 3; % Scaling factor for the vectors
    gimbal_x = R_gimbal_world(:,1) * scale_gimbal;
    gimbal_y = R_gimbal_world(:,2) * scale_gimbal;
    gimbal_z = R_gimbal_world(:,3) * scale_gimbal;

    quiver3(pos_quad(1), pos_quad(2), pos_quad(3), gimbal_x(1), gimbal_x(2), -gimbal_x(3), 'r', 'LineWidth', 1, 'HandleVisibility','off'); % Red for x
    quiver3(pos_quad(1), pos_quad(2), pos_quad(3), gimbal_y(1), gimbal_y(2), -gimbal_y(3), 'g', 'LineWidth', 1, 'HandleVisibility','off'); % Green for y
    quiver3(pos_quad(1), pos_quad(2), pos_quad(3), gimbal_z(1), gimbal_z(2), -gimbal_z(3), 'b', 'LineWidth', 1, 'HandleVisibility','off'); % Blue for z

    % Plot vectors
    if ~legend_added
        legend([p1, p2, p_stl], 'Quadrotor Path', 'Fixed-Wing Path', 'Ghost FW aeroplane');
        legend_added = true;
    end
end
light('Position',[1 0 0],'Style','infinite');
light('Position',[-1 0 0],'Style','infinite');
saveas(fig1, 'results/3d_trajectory.pdf');

% Position Error
error = x_fw - x_quad;
fig2 = figure('Name', 'Position Error', 'NumberTitle', 'off');
plot(t, error(:,1), 'r', t, error(:,2), 'g', t, error(:,3), 'b');
grid on;
title('Position Tracking Error');
xlabel('Time (s)');
ylabel('Error (m)');
legend('x error', 'y error', 'z error');
saveas(fig2, 'results/position_error.pdf');

% Gimbal Angles
phi_g = quad_state(:, 14);
theta_g = quad_state(:, 15);

% --- Calculate Reference Angles and Errors Post-Simulation ---
phi_g_ref_raw_history = zeros(length(t), 1);
theta_g_ref_raw_history = zeros(length(t), 1);

for i = 1:length(t)
    % Quadrotor orientation
    q_bw = quad_state(i, 4:7);
    q0=q_bw(1); q1=q_bw(2); q2=q_bw(3); q3=q_bw(4);
    R_bw = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
            2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
            2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];

    % The reference is the fixed-wing's velocity vector
    q_fw_plot = fw_state(i, 7:10);
    R_fw_plot = [1 - 2*(q_fw_plot(3)^2 + q_fw_plot(4)^2), 2*(q_fw_plot(2)*q_fw_plot(3) - q_fw_plot(1)*q_fw_plot(4)), 2*(q_fw_plot(2)*q_fw_plot(4) + q_fw_plot(1)*q_fw_plot(3));
                 2*(q_fw_plot(2)*q_fw_plot(3) + q_fw_plot(1)*q_fw_plot(4)), 1 - 2*(q_fw_plot(2)^2 + q_fw_plot(4)^2), 2*(q_fw_plot(3)*q_fw_plot(4) - q_fw_plot(1)*q_fw_plot(2));
                 2*(q_fw_plot(2)*q_fw_plot(4) - q_fw_plot(1)*q_fw_plot(3)), 2*(q_fw_plot(3)*q_fw_plot(4) + q_fw_plot(1)*q_fw_plot(2)), 1 - 2*(q_fw_plot(2)^2 + q_fw_plot(3)^2)];
    vel_body_plot = fw_state(i, 4:6)';
    fw_velocity_vec_w = R_fw_plot * (vel_body_plot / (norm(vel_body_plot) + 1e-9));

    % Transform the fw velocity vector to the quad's body frame
    fw_forward_vec_b = R_bw' * fw_velocity_vec_w;
    
    % --- Raw Reference Angle Calculation ---
    xy_norm = sqrt(fw_forward_vec_b(1)^2 + fw_forward_vec_b(2)^2);
    phi_g_ref_raw_history(i) = atan2(fw_forward_vec_b(2), fw_forward_vec_b(1));
    theta_g_ref_raw_history(i) = atan2(-fw_forward_vec_b(3), xy_norm + 1e-9);
end

% Filter the reference angles to remove jumps
phi_g_ref_history = phi_g_ref_raw_history;
theta_g_ref_history = theta_g_ref_raw_history;
jump_threshold_ref = 1.0; % rad
for i = 2:length(phi_g_ref_history)
    if abs(phi_g_ref_history(i) - phi_g_ref_history(i-1)) > jump_threshold_ref
        phi_g_ref_history(i) = phi_g_ref_history(i-1);
    end
    if abs(theta_g_ref_history(i) - theta_g_ref_history(i-1)) > jump_threshold_ref
        theta_g_ref_history(i) = theta_g_ref_history(i-1);
    end
end

% --- Post-processing filter to remove jumps ---
jump_threshold = 1.0; % rad
for i = 2:length(phi_g)-1
    if abs(phi_g(i) - phi_g(i-1)) > jump_threshold
        phi_g(i) = phi_g(i-1);
    end
    if abs(theta_g(i) - theta_g(i-1)) > jump_threshold
        theta_g(i) = theta_g(i-1);
    end
end

% Plot Actual vs Reference
fig3 = figure('Name', 'Gimbal Angles vs Reference', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t, phi_g, 'r', t, phi_g_ref_history, 'r--');
grid on;
title('Gimbal Roll Angle');
ylabel('Angle (rad)');
legend('Actual', 'Reference');
saveas(fig3, 'results/gimbal_angles.pdf');

subplot(2,1,2);
plot(t, theta_g, 'g', t, theta_g_ref_history, 'g--');
grid on;
title('Gimbal Pitch Angle');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('Actual', 'Reference');

% Gimbal Angle Errors
phi_g_error = phi_g_ref_history - phi_g;
theta_g_error = theta_g_ref_history - theta_g;

fig4 = figure('Name', 'Gimbal Angle Errors', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t, phi_g_error, 'r');
grid on;
title('Gimbal Roll Error');
ylabel('Error (rad)');
legend('Roll Error');

subplot(2,1,2);
plot(t, theta_g_error, 'g');
grid on;
title('Gimbal Pitch Error');
xlabel('Time (s)');
ylabel('Error (rad)');
legend('Pitch Error');
saveas(fig4, 'results/gimbal_angle_errors.pdf');

% --- World Frame Orientation Plot ---
% Pre-allocate history arrays
cam_roll_hist = zeros(length(t), 1);
cam_pitch_hist = zeros(length(t), 1);
cam_yaw_hist = zeros(length(t), 1);
fw_roll_hist = zeros(length(t), 1);
fw_pitch_hist = zeros(length(t), 1);
fw_yaw_hist = zeros(length(t), 1);

for i = 1:length(t)
    % Quadrotor orientation
    q_quad = quad_state(i, 4:7);
    q0=q_quad(1); q1=q_quad(2); q2=q_quad(3); q3=q_quad(4);
    R_quad = [1 - 2*(q2^2 + q3^2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
              2*(q1*q2 + q0*q3), 1 - 2*(q1^2 + q3^2), 2*(q2*q3 - q0*q1);
              2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1^2 + q2^2)];

    % Gimbal orientation
    phi_g_i = phi_g(i);
    theta_g_i = theta_g(i);
    c_p = cos(phi_g_i); s_p = sin(phi_g_i);
    c_t = cos(theta_g_i); s_t = sin(theta_g_i);
    R_gb = [ c_p*c_t, -s_p, c_p*s_t;
             s_p*c_t,  c_p, s_p*s_t;
                -s_t,    0,     c_t ];
    
    % Combined camera orientation in world frame
    R_camera_world = R_quad * R_gb;
    
    % Convert camera rotation matrix to Euler angles
    cam_roll_hist(i) = atan2(R_camera_world(3,2), R_camera_world(3,3));
    cam_pitch_hist(i) = asin(-R_camera_world(3,1));
    cam_yaw_hist(i) = atan2(R_camera_world(2,1), R_camera_world(1,1));

    % Fixed-wing orientation
    q_fw = fw_state(i, 7:10);
    q0=q_fw(1); q1=q_fw(2); q2=q_fw(3); q3=q_fw(4);
    R_fw = [1 - 2*(q2^2 + q3^2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
            2*(q1*q2 + q0*q3), 1 - 2*(q1^2 + q3^2), 2*(q2*q3 - q0*q1);
            2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1^2 + q2^2)];
            
    % Convert fixed-wing rotation matrix to Euler angles
    fw_roll_hist(i) = atan2(R_fw(3,2), R_fw(3,3));
    fw_pitch_hist(i) = asin(-R_fw(3,1));
    fw_yaw_hist(i) = atan2(R_fw(2,1), R_fw(1,1));
end

% --- Filter World Frame Angles to Remove Jumps ---
jump_threshold_world = 1.0; % rad
for i = 2:length(t)
    % Camera angles
    if abs(cam_roll_hist(i) - cam_roll_hist(i-1)) > jump_threshold_world
        cam_roll_hist(i) = cam_roll_hist(i-1);
    end
    if abs(cam_pitch_hist(i) - cam_pitch_hist(i-1)) > jump_threshold_world
        cam_pitch_hist(i) = cam_pitch_hist(i-1);
    end
    if abs(cam_yaw_hist(i) - cam_yaw_hist(i-1)) > jump_threshold_world
        cam_yaw_hist(i) = cam_yaw_hist(i-1);
    end
    % Fixed-wing angles
    if abs(fw_roll_hist(i) - fw_roll_hist(i-1)) > jump_threshold_world
        fw_roll_hist(i) = fw_roll_hist(i-1);
    end
    if abs(fw_pitch_hist(i) - fw_pitch_hist(i-1)) > jump_threshold_world
        fw_pitch_hist(i) = fw_pitch_hist(i-1);
    end
    if abs(fw_yaw_hist(i) - fw_yaw_hist(i-1)) > jump_threshold_world
        fw_yaw_hist(i) = fw_yaw_hist(i-1);
    end
end


% Plot World Frame Orientations
fig5 = figure('Name', 'World Frame Orientation', 'NumberTitle', 'off');
subplot(3,1,1);
plot(t, unwrap(cam_roll_hist), 'r', t, unwrap(fw_roll_hist), 'r--');
grid on;
title('Camera vs. Fixed-Wing Roll Angle (World Frame)');
ylabel('Angle (rad)');
legend('Camera', 'Fixed-Wing');

subplot(3,1,2);
plot(t, unwrap(cam_pitch_hist), 'g', t, unwrap(fw_pitch_hist), 'g--');
grid on;
title('Camera vs. Fixed-Wing Pitch Angle (World Frame)');
ylabel('Angle (rad)');
legend('Camera', 'Fixed-Wing');

subplot(3,1,3);
plot(t, unwrap(cam_yaw_hist), 'b', t, unwrap(fw_yaw_hist), 'b--');
grid on;
title('Camera vs. Fixed-Wing Yaw Angle (World Frame)');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('Camera', 'Fixed-Wing');
saveas(fig5, 'results/world_frame_orientation.pdf');


disp('Real-time simulation and plotting complete.');
