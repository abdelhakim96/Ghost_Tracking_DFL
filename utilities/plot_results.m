%% Post-processing and Plotting
set(0, 'DefaultFigureWindowStyle', 'docked'); % or 'normal' for floating windows

% Unpack states
quad_state = state(:, 1:17);
fw_state = state(:, 18:30);

% Create results directory if it doesn't exist
if ~exist('results', 'dir')
   mkdir('results')
end
results_dir = ['results/results_' config_to_run];
if ~exist(results_dir, 'dir')
   mkdir(results_dir)
end

% Extract quadrotor and fixed-wing positions
x_quad = quad_state(:, 1:3);
x_fw = fw_state(:, 1:3);

%% Plot 3D Trajectory (NED Frame)
fig1 = figure('Name', '3D Trajectory Tracking (NED Frame)', 'NumberTitle', 'off');
p1 = plot3(x_quad(:,1), x_quad(:,2), -x_quad(:,3), 'b', 'LineWidth', 1.5);
p1.Color(4) = 0.5; hold on;
p2 = plot3(x_fw(:,1), x_fw(:,2), -x_fw(:,3), 'r--', 'LineWidth', 1.5);
p2.Color(4) = 0.5; grid on;
title(['Quadrotor Tracking Fixed-Wing Trajectory (' config_to_run ')']);
xlabel('North (m)'); ylabel('East (m)'); zlabel('Altitude (m)');
axis equal; view(45, 25);

legend_added = false;
legend_added_stl = false;
legend_added_quad_stl = false;

% Load STL model for the fixed-wing aircraft
[F_fw, V_fw, ~] = stlread('../CAD/aero.stl');

% Load STL model for the quadrotor
[F_quad, V_quad, ~] = stlread('../CAD/quad.stl');
V_quad = V_quad / 1000; % Assuming the model is in mm
V_quad = V_quad - mean(V_quad);
quad_model_scale = 1.5;
V_quad = V_quad * quad_model_scale;


V_fw = V_fw - mean(V_fw);
V_fw = V_fw / 20;
model_scale = 0.7;
V_fw = V_fw * model_scale;

% Corrective rotation
yaw_angle = -pi;
R_yaw = [cos(yaw_angle) -sin(yaw_angle) 0;
         sin(yaw_angle)  cos(yaw_angle) 0;
         0               0              1];
pitch_angle = 20 * pi / 180;
R_pitch = [cos(pitch_angle) 0 sin(pitch_angle);
           0 1 0;
           -sin(pitch_angle) 0 cos(pitch_angle)];
R_correction = R_pitch * R_yaw;
V_fw = (R_correction * V_fw')';

for i = 1:10:length(t)
    % Fixed-wing pose
    pos_fw = [x_fw(i,1), x_fw(i,2), -x_fw(i,3)];
    q_fw = fw_state(i, 7:10);
    q0=q_fw(1); q1=q_fw(2); q2=q_fw(3); q3=q_fw(4);
    R_fw = [1 - 2*(q2^2 + q3^2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
            2*(q1*q2 + q0*q3), 1 - 2*(q1^2 + q3^2), 2*(q2*q3 - q0*q1);
            2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1^2 + q2^2)];
    V_rotated_fw = (R_fw' * V_fw')';
    V_translated_fw = V_rotated_fw + pos_fw;
    
    if ~legend_added_stl
        p_stl = patch('Faces', F_fw, 'Vertices', V_translated_fw, 'FaceColor', [0.8 0.2 0.2], 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'FaceAlpha', 0.1, 'DisplayName', 'Ghost FW aeroplane');
        legend_added_stl = true;
    else
        patch('Faces', F_fw, 'Vertices', V_translated_fw, 'FaceColor', [0.8 0.2 0.2], 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'FaceAlpha', 0.1, 'HandleVisibility','off');
    end

    % Gimbal and quad plotting (same as before)
    q_quad = quad_state(i, 4:7);
    q0=q_quad(1); q1=q_quad(2); q2=q_quad(3); q3=q_quad(4);
    R_quad = [1 - 2*(q2^2 + q3^2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
              2*(q1*q2 + q0*q3), 1 - 2*(q1^2 + q3^2), 2*(q2*q3 - q0*q1);
              2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1^2 + q2^2)];
    phi_g = quad_state(i, 14);
    theta_g = quad_state(i, 15);
    R_gb = [cos(phi_g)*cos(theta_g), -sin(phi_g), cos(phi_g)*sin(theta_g);
            sin(phi_g)*cos(theta_g),  cos(phi_g), sin(phi_g)*sin(theta_g);
           -sin(theta_g), 0, cos(theta_g)];
    R_gimbal_w = R_quad * R_gb;
    pos_quad = [x_quad(i,1), x_quad(i,2), -x_quad(i,3)];

    % Quadrotor model
    V_rotated_quad = (R_quad' * V_quad')';
    V_translated_quad = V_rotated_quad + pos_quad;

    if ~legend_added_quad_stl
        p_quad_stl = patch('Faces', F_quad, 'Vertices', V_translated_quad, 'FaceColor', [0.2 0.2 0.8], 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'FaceAlpha', 1.0, 'DisplayName', 'Quadrotor');
        legend_added_quad_stl = true;
    else
        patch('Faces', F_quad, 'Vertices', V_translated_quad, 'FaceColor', [0.2 0.2 0.8], 'EdgeColor', 'none', 'FaceLighting', 'gouraud', 'FaceAlpha', 1.0, 'HandleVisibility','off');
    end
    
    % Gimbal axes
    scale_gimbal = 2.5; line_width_gimbal = 1.5;
    dark_red = [0.6, 0, 0]; dark_green = [0, 0.6, 0]; dark_blue = [0, 0, 0.6];
    gimbal_x = R_gimbal_w(:,1) * scale_gimbal;
    gimbal_y = R_gimbal_w(:,2) * scale_gimbal;
    gimbal_z = R_gimbal_w(:,3) * scale_gimbal;
    quiver3(pos_quad(1), pos_quad(2), pos_quad(3), gimbal_x(1), gimbal_x(2), -gimbal_x(3), 'Color', dark_red, 'LineWidth', line_width_gimbal, 'HandleVisibility','off');
    quiver3(pos_quad(1), pos_quad(2), pos_quad(3), gimbal_y(1), gimbal_y(2), -gimbal_y(3), 'Color', dark_green, 'LineWidth', line_width_gimbal, 'HandleVisibility','off');
    quiver3(pos_quad(1), pos_quad(2), pos_quad(3), gimbal_z(1), gimbal_z(2), -gimbal_z(3), 'Color', dark_blue, 'LineWidth', line_width_gimbal, 'HandleVisibility','off');
    
    if ~legend_added
        legend([p1, p2, p_stl, p_quad_stl], 'Quadrotor Path', 'Fixed-Wing Path', 'Ghost FW aeroplane', 'Quadrotor');
        legend_added = true;
    end
end
light('Position',[1 0 0],'Style','infinite');
light('Position',[-1 0 0],'Style','infinite');
saveas(fig1, [results_dir '/3d_trajectory_' config_to_run '.pdf']);
drawnow; figure(fig1); pause(0.1);

%% Orientation and Reference Calculations
phi_g_raw = quad_state(:, 14);
theta_g_raw = quad_state(:, 15);
phi_g_ref_history = zeros(length(t), 1);
theta_g_ref_history = zeros(length(t), 1);
gimbal_global_roll_hist = zeros(length(t), 1);
gimbal_global_pitch_hist = zeros(length(t), 1);
gimbal_global_yaw_hist = zeros(length(t), 1);
fw_global_roll_hist = zeros(length(t), 1);
fw_global_pitch_hist = zeros(length(t), 1);
fw_global_yaw_hist = zeros(length(t), 1);
drone_global_roll_hist = zeros(length(t), 1);
drone_global_pitch_hist = zeros(length(t), 1);
last_phi_g_ref = 0; last_theta_g_ref = 0;

% Initialize last angles for unwrapping
last_gimbal_roll = 0;
last_gimbal_pitch = 0;
last_gimbal_yaw = 0;

for i = 1:length(t)
    % Quaternion to Euler and reference calculations (unchanged)
    q_bw = quad_state(i, 4:7);
    q0=q_bw(1); q1=q_bw(2); q2=q_bw(3); q3=q_bw(4);
    R_bw = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
            2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
            2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];
    drone_global_roll_hist(i) = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
    drone_global_pitch_hist(i) = asin(2*(q0*q2 - q3*q1));

    q_fw = fw_state(i, 7:10);
    q0_fw=q_fw(1); q1_fw=q_fw(2); q2_fw=q_fw(3); q3_fw=q_fw(4);
    R_fw_w = [q0_fw^2+q1_fw^2-q2_fw^2-q3_fw^2, 2*(q1_fw*q2_fw-q0_fw*q3_fw), 2*(q1_fw*q3_fw+q0_fw*q2_fw);
              2*(q1_fw*q2_fw+q0_fw*q3_fw), q0_fw^2-q1_fw^2+q2_fw^2-q3_fw^2, 2*(q2_fw*q3_fw-q0_fw*q1_fw);
              2*(q1_fw*q3_fw-q0_fw*q2_fw), 2*(q2_fw*q3_fw+q0_fw*q1_fw), q0_fw^2-q1_fw^2-q2_fw^2+q3_fw^2];

    phi_g_actual = quad_state(i, 14);
    theta_g_actual = quad_state(i, 15);
    R_gb = [cos(phi_g_actual)*cos(theta_g_actual), -sin(phi_g_actual), cos(phi_g_actual)*sin(theta_g_actual);
            sin(phi_g_actual)*cos(theta_g_actual),  cos(phi_g_actual), sin(phi_g_actual)*sin(theta_g_actual);
           -sin(theta_g_actual),                 0,                cos(theta_g_actual)];
    R_gimbal_w = R_bw * R_gb;

    % Calculate raw angles with gimbal lock check
    pitch_val = -R_gimbal_w(3,1);
    if abs(pitch_val) > 0.9999
        % Gimbal lock case
        raw_yaw = atan2(R_gimbal_w(1,2), R_gimbal_w(2,2));
        raw_pitch = asin(pitch_val);
        raw_roll = 0; % Roll is not well-defined in this case
    else
        % Standard case
        raw_yaw = atan2(R_gimbal_w(2,1), R_gimbal_w(1,1));
        raw_pitch = asin(pitch_val);
        raw_roll = atan2(R_gimbal_w(3,2), R_gimbal_w(3,3));
    end

    % Unwrap angles for smooth plotting
    gimbal_global_yaw_hist(i) = unwrapAngle(raw_yaw, last_gimbal_yaw);
    gimbal_global_pitch_hist(i) = unwrapAngle(raw_pitch, last_gimbal_pitch);
    gimbal_global_roll_hist(i) = unwrapAngle(0.0, last_gimbal_roll);

    % Update last angles
    last_gimbal_yaw = gimbal_global_yaw_hist(i);
    last_gimbal_pitch = gimbal_global_pitch_hist(i);
    last_gimbal_roll = gimbal_global_roll_hist(i);

    fw_global_yaw_hist(i) = atan2(R_fw_w(2,1), R_fw_w(1,1));
    fw_global_pitch_hist(i) = asin(-R_fw_w(3,1));
    fw_global_roll_hist(i) = atan2(R_fw_w(3,2), R_fw_w(3,3));
end

%% Correct for angle jumps
gimbal_global_roll_hist = neglectAngleJump(gimbal_global_roll_hist);
gimbal_global_pitch_hist = neglectAngleJump(gimbal_global_pitch_hist);
gimbal_global_yaw_hist = neglectAngleJump(gimbal_global_yaw_hist);
fw_global_roll_hist = neglectAngleJump(fw_global_roll_hist);
fw_global_pitch_hist = neglectAngleJump(fw_global_pitch_hist);
fw_global_yaw_hist = neglectAngleJump(fw_global_yaw_hist);
drone_global_roll_hist = neglectAngleJump(drone_global_roll_hist);
drone_global_pitch_hist = neglectAngleJump(drone_global_pitch_hist);


%% Combined Position and Orientation Plot
fig_combined = figure('Name', 'Position and Orientation Tracking', 'NumberTitle', 'off');
set(fig_combined, 'Position', [100, 100, 800, 1200]);

subplot(3,2,1); plot(t, x_quad(:,1), 'b', t, x_fw(:,1), 'r--', 'LineWidth', 1.5);
title('X Position'); ylabel('m'); grid on; legend('Gimbal', 'Fixed-Wing');
subplot(3,2,3); plot(t, x_quad(:,2), 'b', t, x_fw(:,2), 'r--', 'LineWidth', 1.5);
title('Y Position'); ylabel('m'); grid on;
subplot(3,2,5); plot(t, -x_quad(:,3), 'b', t, -x_fw(:,3), 'r--', 'LineWidth', 1.5);
title('Z Position'); ylabel('m'); xlabel('Time (s)'); grid on;
subplot(3,2,2); plot(t, gimbal_global_roll_hist*180/pi, 'b', t, fw_global_roll_hist*180/pi, 'r--', 'LineWidth', 1.5);
title('Global Roll'); ylabel('deg'); grid on;
subplot(3,2,4); plot(t, gimbal_global_pitch_hist*180/pi, 'b', t, fw_global_pitch_hist*180/pi, 'r--', 'LineWidth', 1.5);
title('Global Pitch'); ylabel('deg'); grid on;
subplot(3,2,6); plot(t, gimbal_global_yaw_hist*180/pi, 'b', t, fw_global_yaw_hist*180/pi, 'r--', 'LineWidth', 1.5);
title('Global Yaw'); ylabel('deg'); xlabel('Time (s)'); grid on;
saveas(fig_combined, [results_dir '/position_and_orientation_' config_to_run '.pdf']);
drawnow; figure(fig_combined); pause(0.1);

%% Control Inputs
control_history = quadrotor_dynamics_realtime('get_history');
time_hist = control_history(:, 1);
thrust_hist = control_history(:, 2);
u_p_hist = control_history(:, 3);
u_q_hist = control_history(:, 4);
u_r_hist = control_history(:, 5);
u_phi_g_hist = control_history(:, 9);
u_theta_g_hist = control_history(:, 10);
gimbal_roll_from_history = control_history(:, 11);

fig_control_inputs = figure('Name', 'Control Inputs', 'NumberTitle', 'off');
subplot(3,2,1); plot(time_hist, thrust_hist, 'k', 'LineWidth', 1.5); title('Total Thrust'); grid on;
subplot(3,2,2); plot(time_hist, u_p_hist, 'r', 'LineWidth', 1.5); title('Roll Moment'); grid on;
subplot(3,2,3); plot(time_hist, u_q_hist, 'g', 'LineWidth', 1.5); title('Pitch Moment'); grid on;
subplot(3,2,4); plot(time_hist, u_r_hist, 'b', 'LineWidth', 1.5); title('Yaw Moment'); grid on;
subplot(3,2,5); plot(time_hist, u_phi_g_hist, 'm', 'LineWidth', 1.5); title('Gimbal Roll Input'); xlabel('s'); grid on;
subplot(3,2,6); plot(time_hist, u_theta_g_hist, 'c', 'LineWidth', 1.5); title('Gimbal Pitch Input'); xlabel('s'); grid on;
saveas(fig_control_inputs, [results_dir '/control_inputs_' config_to_run '.pdf']);
drawnow; figure(fig_control_inputs); pause(0.1);

%% Drone State Plot
fig_drone_state = figure('Name', 'Drone State vs Time', 'NumberTitle', 'off');
subplot(2,1,1);
plot(t, x_quad(:,1), 'r', t, x_quad(:,2), 'g', t, -x_quad(:,3), 'b', 'LineWidth', 1.5);
grid on; title('Drone Position'); xlabel('s'); ylabel('m'); legend('x','y','z');
subplot(2,1,2);
plot(t, gimbal_global_roll_hist*180/pi, 'r', t, gimbal_global_pitch_hist*180/pi, 'g', t, gimbal_global_yaw_hist*180/pi, 'b', 'LineWidth', 1.5);
grid on; title('Drone Orientation'); xlabel('s'); ylabel('deg'); legend('Roll','Pitch','Yaw');
saveas(fig_drone_state, [results_dir '/drone_state_' config_to_run '.pdf']);
drawnow; figure(fig_drone_state); pause(0.1);

%% Debug Roll Comparison
fig_debug = figure('Name', 'Global Roll Angle Comparison', 'NumberTitle', 'off');
plot(t, gimbal_global_roll_hist*180/pi, 'b-', 'LineWidth', 1.5);
hold on; plot(time_hist, gimbal_roll_from_history*180/pi, 'r--', 'LineWidth', 1.5);
plot(t, drone_global_roll_hist*180/pi, 'g-.', 'LineWidth', 1.5);
plot(t, fw_global_roll_hist*180/pi, 'm:', 'LineWidth', 1.5);
grid on; title('Roll Comparison'); xlabel('s'); ylabel('deg');
legend('Post-Proc','Sim','Drone', 'Fixed-Wing');
saveas(fig_debug, [results_dir '/debug_roll_comparison_' config_to_run '.pdf']);
drawnow; figure(fig_debug); pause(0.1);

%% Debug Pitch Comparison
fig_pitch_debug = figure('Name', 'Global Pitch Angle Comparison', 'NumberTitle', 'off');
plot(t, gimbal_global_pitch_hist*180/pi, 'b-', 'LineWidth', 1.5);
hold on; plot(t, fw_global_pitch_hist*180/pi, 'r--', 'LineWidth', 1.5);
plot(t, drone_global_pitch_hist*180/pi, 'g-.', 'LineWidth', 1.5);
grid on; title('Pitch Comparison'); xlabel('s'); ylabel('deg');
legend('Gimbal','FW','Drone');
saveas(fig_pitch_debug, [results_dir '/debug_pitch_comparison_' config_to_run '.pdf']);
drawnow; figure(fig_pitch_debug); pause(0.1);

disp('✅ Real-time simulation and plotting complete. All figures are visible in separate tabs.');
