clear all;
clc;

% Select the configuration file to use
% 'loop', 'roll', or 'straight'
config_to_run = 'immelman';

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

    % Plot FW body frame basis vectors
    scale = 5; % Scaling factor for the vectors
    body_x = R_fw(:,1) * scale;
    body_y = R_fw(:,2) * scale;
    body_z = R_fw(:,3) * scale;

    quiver3(pos_fw(1), pos_fw(2), pos_fw(3), body_x(1), body_x(2), -body_x(3), 'r', 'LineWidth', 1, 'HandleVisibility','off');
    quiver3(pos_fw(1), pos_fw(2), pos_fw(3), body_y(1), body_y(2), -body_y(3), 'g', 'LineWidth', 1, 'HandleVisibility','off');
    quiver3(pos_fw(1), pos_fw(2), pos_fw(3), body_z(1), body_z(2), -body_z(3), 'b', 'LineWidth', 1, 'HandleVisibility','off');

    % Gimbal forward vector (actual pointing direction)
    q_quad = quad_state(i, 4:7); % [q0, q1, q2, q3]
    q0=q_quad(1); q1=q_quad(2); q2=q_quad(3); q3=q_quad(4);
    R_quad = [1 - 2*(q2^2 + q3^2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
              2*(q1*q2 + q0*q3), 1 - 2*(q1^2 + q3^2), 2*(q2*q3 - q0*q1);
              2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1^2 + q2^2)];
              
    phi_g = quad_state(i, 14);
    theta_g = quad_state(i, 15);
    % Reconstruct gimbal pointing vector in body frame from gimbal angles
    % Assumes gimbal is mounted forwards
    gimbal_vec_body = [cos(theta_g)*cos(phi_g); cos(theta_g)*sin(phi_g); -sin(theta_g)];
    gimbal_forward_vec = R_quad * gimbal_vec_body;

    % Plot vectors
    if ~legend_added
        q2 = quiver3(x_quad(i,1), x_quad(i,2), -x_quad(i,3), gimbal_forward_vec(1), gimbal_forward_vec(2), -gimbal_forward_vec(3), 5, 'c', 'LineWidth', 2, 'DisplayName', 'Gimbal Pointing');
        legend([p1, p2, q2, p_stl], 'Quadrotor Path', 'Fixed-Wing Path', 'Gimbal Pointing', 'Ghost FW aeroplane');
        legend_added = true;
    else
        quiver3(x_quad(i,1), x_quad(i,2), -x_quad(i,3), gimbal_forward_vec(1), gimbal_forward_vec(2), -gimbal_forward_vec(3), 5, 'c', 'LineWidth', 2, 'HandleVisibility','off');
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
phi_g_ref_history = zeros(length(t), 1);
theta_g_ref_history = zeros(length(t), 1);
last_phi_g_ref = 0; % Initialize for post-processing unwrapping
last_theta_g_ref = 0;

for i = 1:length(t)
    % Quadrotor orientation
    q_bw = quad_state(i, 4:7);
    q0=q_bw(1); q1=q_bw(2); q2=q_bw(3); q3=q_bw(4);
    R_bw = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
            2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
            2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];

    % The reference is the fixed-wing's velocity vector, which is already calculated
    % for the quiver plot. We can reuse that.
    q_fw_plot = fw_state(i, 7:10);
    R_fw_plot = [1 - 2*(q_fw_plot(3)^2 + q_fw_plot(4)^2), 2*(q_fw_plot(2)*q_fw_plot(3) - q_fw_plot(1)*q_fw_plot(4)), 2*(q_fw_plot(2)*q_fw_plot(4) + q_fw_plot(1)*q_fw_plot(3));
                 2*(q_fw_plot(2)*q_fw_plot(3) + q_fw_plot(1)*q_fw_plot(4)), 1 - 2*(q_fw_plot(2)^2 + q_fw_plot(4)^2), 2*(q_fw_plot(3)*q_fw_plot(4) - q_fw_plot(1)*q_fw_plot(2));
                 2*(q_fw_plot(2)*q_fw_plot(4) - q_fw_plot(1)*q_fw_plot(3)), 2*(q_fw_plot(3)*q_fw_plot(4) + q_fw_plot(1)*q_fw_plot(2)), 1 - 2*(q_fw_plot(2)^2 + q_fw_plot(3)^2)];
    vel_body_plot = fw_state(i, 4:6)';
    fw_velocity_vec_w = R_fw_plot * (vel_body_plot / (norm(vel_body_plot) + 1e-9));

    % Transform the fw velocity vector (our reference) to the quad's body frame
    fw_forward_vec_b = R_bw' * fw_velocity_vec_w;
    
    % --- Reference Angle Calculation with Singularity Avoidance ---
    xy_norm = sqrt(fw_forward_vec_b(1)^2 + fw_forward_vec_b(2)^2);
    
    if i == 1
        if xy_norm < 1e-6
            last_phi_g_ref = 0;
        else
            last_phi_g_ref = atan2(fw_forward_vec_b(2), fw_forward_vec_b(1));
        end
        last_theta_g_ref = atan2(-fw_forward_vec_b(3), xy_norm + 1e-9);
    end

    if xy_norm < 1e-6
        phi_g_ref = last_phi_g_ref;
    else
        phi_g_ref_raw = atan2(fw_forward_vec_b(2), fw_forward_vec_b(1));
        delta_phi = phi_g_ref_raw - last_phi_g_ref;
        delta_phi = mod(delta_phi + pi, 2*pi) - pi;
        if abs(delta_phi) > (170 * pi / 180)
            phi_g_ref = last_phi_g_ref;
        else
            phi_g_ref = last_phi_g_ref + delta_phi;
        end
    end
    last_phi_g_ref = phi_g_ref;

    theta_g_ref_raw = atan2(-fw_forward_vec_b(3), xy_norm + 1e-9);
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
end

% Position Error
error = x_fw - x_quad;
figure('Name', 'Position Error', 'NumberTitle', 'off');
plot(t, error(:,1), 'r', t, error(:,2), 'g', t, error(:,3), 'b');
grid on;
title('Position Tracking Error');
xlabel('Time (s)');
ylabel('Error (m)');
legend('x error', 'y error', 'z error');

% Gimbal Angles
phi_g = quad_state(:, 14);
theta_g = quad_state(:, 15);

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

disp('Real-time simulation and plotting complete.');
