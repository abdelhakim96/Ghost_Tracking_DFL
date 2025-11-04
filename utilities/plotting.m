function plotting(t, state, config_to_run)
% Post-processing and Plotting
state = real(state);
% Unpack states
quad_state = state(:, 1:18);
fw_state = state(:, 19:31);

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

    % Plot FW body frame basis vectors
    scale = 5; % Scaling factor for the vectors
    body_x = R_fw(:,1) * scale;
    body_y = R_fw(:,2) * scale;
    body_z = R_fw(:,3) * scale;

    quiver3(pos_fw(1), pos_fw(2), pos_fw(3), body_x(1), body_x(2), -body_x(3), 'Color', [0.6, 0, 0], 'LineWidth', 1, 'HandleVisibility','off');
    quiver3(pos_fw(1), pos_fw(2), pos_fw(3), body_y(1), body_y(2), -body_y(3), 'Color', [0, 0.6, 0], 'LineWidth', 1, 'HandleVisibility','off');
    q_fw_axes = quiver3(pos_fw(1), pos_fw(2), pos_fw(3), body_z(1), body_z(2), -body_z(3), 'Color', [0, 0, 0.6], 'LineWidth', 1, 'DisplayName', 'Fixed-Wing Frame');

    % Gimbal forward vector (actual pointing direction)
    q_quad = quad_state(i, 4:7); % [q0, q1, q2, q3]
    q0=q_quad(1); q1=q_quad(2); q2=q_quad(3); q3=q_quad(4);
    R_quad = [1 - 2*(q2^2 + q3^2), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
              2*(q1*q2 + q0*q3), 1 - 2*(q1^2 + q3^2), 2*(q2*q3 - q0*q1);
              2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1^2 + q2^2)];
              
    phi_g = quad_state(i, 14);   % Yaw
    theta_g = quad_state(i, 15); % Pitch
    gamma_g = quad_state(i, 16); % Roll
    
    % Reconstruct gimbal rotation matrix (body to gimbal) using ZYX Euler sequence
    R_z = [cos(phi_g) -sin(phi_g) 0; sin(phi_g) cos(phi_g) 0; 0 0 1];
    R_y = [cos(theta_g) 0 sin(theta_g); 0 1 0; -sin(theta_g) 0 cos(theta_g)];
    R_x = [1 0 0; 0 cos(gamma_g) -sin(gamma_g); 0 sin(gamma_g) cos(gamma_g)];
    R_gb = R_z * R_y * R_x;
    
    % Gimbal orientation in world frame
    R_gimbal_w = R_quad * R_gb;

    % Gimbal body frame basis vectors
    gimbal_scale = 2; % Scaling factor for the vectors
    gimbal_x = R_gimbal_w(:,1) * gimbal_scale;
    gimbal_y = R_gimbal_w(:,2) * gimbal_scale;
    gimbal_z = R_gimbal_w(:,3) * gimbal_scale;

    % Plot gimbal axes
    pos_quad = [x_quad(i,1), x_quad(i,2), -x_quad(i,3) - 1]; % Offset by 1m in Z for visibility
    quiver3(pos_quad(1), pos_quad(2), pos_quad(3), gimbal_x(1), gimbal_x(2), -gimbal_x(3), 'r', 'LineWidth', 2, 'HandleVisibility','off');
    quiver3(pos_quad(1), pos_quad(2), pos_quad(3), gimbal_y(1), gimbal_y(2), -gimbal_y(3), 'g', 'LineWidth', 2, 'HandleVisibility','off');
    q_gimbal = quiver3(pos_quad(1), pos_quad(2), pos_quad(3), gimbal_z(1), gimbal_z(2), -gimbal_z(3), 'b', 'LineWidth', 2, 'DisplayName', 'Gimbal Frame');

    % Plot vectors
    if ~legend_added
        legend([p1, p2, p_stl, q_gimbal, q_fw_axes], 'Quadrotor Path', 'Fixed-Wing Path', 'Ghost FW aeroplane', 'Gimbal Frame', 'Fixed-Wing Frame');
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

% Gimbal Angles (raw from state)
phi_g_raw = quad_state(:, 14);
theta_g_raw = quad_state(:, 15);
gamma_g_raw = quad_state(:, 16);

% --- Calculate Reference Angles and Errors Post-Simulation ---
phi_g_ref_history = zeros(length(t), 1);
theta_g_ref_history = zeros(length(t), 1);
gamma_g_ref_history = zeros(length(t), 1);

% New history vectors for full orientation tracking
gimbal_global_roll_hist = zeros(length(t), 1);
gimbal_global_pitch_hist = zeros(length(t), 1);
gimbal_global_yaw_hist = zeros(length(t), 1);
fw_global_roll_hist = zeros(length(t), 1);
fw_global_pitch_hist = zeros(length(t), 1);
fw_global_yaw_hist = zeros(length(t), 1);

gimbal_global_roll_raw_hist = zeros(length(t), 1);
gimbal_global_pitch_raw_hist = zeros(length(t), 1);
gimbal_global_yaw_raw_hist = zeros(length(t), 1);
fw_global_roll_raw_hist = zeros(length(t), 1);
fw_global_pitch_raw_hist = zeros(length(t), 1);
fw_global_yaw_raw_hist = zeros(length(t), 1);

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

    % Sanitize matrices before use
    R_gimbal_w = real(R_gimbal_w);
    R_fw_w = real(R_fw_w);

    % Raw angle calculations
    gimbal_global_yaw_raw_hist(i) = atan2(R_gimbal_w(2,1), R_gimbal_w(1,1));
    gimbal_global_pitch_raw_hist(i) = asin(-R_gimbal_w(3,1));
    gimbal_global_roll_raw_hist(i) = atan2(R_gimbal_w(3,2), R_gimbal_w(3,3));
    fw_global_yaw_raw_hist(i) = atan2(R_fw_w(2,1), R_fw_w(1,1));
    fw_global_pitch_raw_hist(i) = asin(-R_fw_w(3,1));
    fw_global_roll_raw_hist(i) = atan2(R_fw_w(3,2), R_fw_w(3,3));

    % --- Calculate Body-Frame Reference Angles (for original plots) ---
    R_gb_ref = R_bw' * R_fw_w;
    
    % Extract target Euler angles from the desired rotation matrix
    % Using a ZYX rotation sequence for the gimbal (yaw, pitch, roll)
    angles_ref = rotm2eul(R_gb_ref, 'ZYX');
    phi_g_ref_history(i) = angles_ref(1);   % Yaw
    theta_g_ref_history(i) = angles_ref(2); % Pitch
    gamma_g_ref_history(i) = angles_ref(3); % Roll
    
    % This old unwrapping code is no longer needed for the 3-axis controller
    %{
    if i == 1
        last_phi_g_ref = phi_g_ref_raw;
        last_theta_g_ref = theta_g_ref_raw;
    end
    jump_threshold = 0.9;
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
    %}
end

% Unwrap angles
gimbal_global_roll_hist = unwrap(gimbal_global_roll_raw_hist);
gimbal_global_pitch_hist = unwrap(gimbal_global_pitch_raw_hist);
gimbal_global_yaw_hist = unwrap(gimbal_global_yaw_raw_hist);
fw_global_roll_hist = unwrap(fw_global_roll_raw_hist);
fw_global_pitch_hist = unwrap(fw_global_pitch_raw_hist);
fw_global_yaw_hist = unwrap(fw_global_yaw_raw_hist);

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
phi_g = unwrap(quad_state(:, 14));
theta_g = unwrap(quad_state(:, 15));
gamma_g = unwrap(quad_state(:, 16));
phi_g_ref_history = unwrap(phi_g_ref_history);
theta_g_ref_history = unwrap(theta_g_ref_history);
gamma_g_ref_history = unwrap(gamma_g_ref_history);


% Plot Actual vs Reference for Yaw Angle
fig3 = figure('Name', 'Gimbal Yaw Angle vs Reference', 'NumberTitle', 'off');
plot(t, phi_g * 180/pi, 'r', t, phi_g_ref_history * 180/pi, 'r--');
grid on;
title('Gimbal Yaw Angle');
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('Actual', 'Reference');
saveas(fig3, 'results/gimbal_yaw_angle.pdf');

% Plot Actual vs Reference for Pitch Angle
fig_pitch = figure('Name', 'Gimbal Pitch Angle vs Reference', 'NumberTitle', 'off');
plot(t, theta_g * 180/pi, 'g', t, theta_g_ref_history * 180/pi, 'g--');
grid on;
title('Gimbal Pitch Angle');
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('Actual', 'Reference');
saveas(fig_pitch, 'results/gimbal_pitch_angle.pdf');

% Plot Actual vs Reference for Roll Angle
fig_roll = figure('Name', 'Gimbal Roll Angle vs Reference', 'NumberTitle', 'off');
plot(t, gamma_g * 180/pi, 'b', t, gamma_g_ref_history * 180/pi, 'b--');
grid on;
title('Gimbal Roll Angle');
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('Actual', 'Reference');
saveas(fig_roll, 'results/gimbal_roll_angle.pdf');

% Gimbal Angle Errors
phi_g_error = phi_g_ref_history - phi_g;
theta_g_error = theta_g_ref_history - theta_g;
gamma_g_error = gamma_g_ref_history - gamma_g;

fig4 = figure('Name', 'Gimbal Angle Errors', 'NumberTitle', 'off');
subplot(3,1,1);
plot(t, phi_g_error * 180/pi, 'r');
grid on;
title('Gimbal Yaw Error');
ylabel('Error (degrees)');
legend('Yaw Error');

subplot(3,1,2);
plot(t, theta_g_error * 180/pi, 'g');
grid on;
title('Gimbal Pitch Error');
ylabel('Error (degrees)');
legend('Pitch Error');

subplot(3,1,3);
plot(t, gamma_g_error * 180/pi, 'b');
grid on;
title('Gimbal Roll Error');
xlabel('Time (s)');
ylabel('Error (degrees)');
legend('Roll Error');
saveas(fig4, 'results/gimbal_angle_errors.pdf');

% --- NEW: Consolidated Gimbal Tracking Performance Plot ---
fig_consolidated = figure('Name', 'Gimbal Tracking Performance', 'NumberTitle', 'off');

% Yaw Angle vs Reference
subplot(3,2,1);
plot(t, phi_g * 180/pi, 'r', t, phi_g_ref_history * 180/pi, 'r--');
grid on;
title('Gimbal Yaw Angle vs Reference');
ylabel('Angle (degrees)');
legend('Actual', 'Reference');

% Yaw Error
subplot(3,2,2);
plot(t, phi_g_error * 180/pi, 'r');
grid on;
title('Gimbal Yaw Error');
ylabel('Error (degrees)');
legend('Yaw Error');

% Pitch Angle vs Reference
subplot(3,2,3);
plot(t, theta_g * 180/pi, 'g', t, theta_g_ref_history * 180/pi, 'g--');
grid on;
title('Gimbal Pitch Angle vs Reference');
ylabel('Angle (degrees)');
legend('Actual', 'Reference');

% Pitch Error
subplot(3,2,4);
plot(t, theta_g_error * 180/pi, 'g');
grid on;
title('Gimbal Pitch Error');
ylabel('Error (degrees)');
legend('Pitch Error');

% Roll Angle vs Reference
subplot(3,2,5);
plot(t, gamma_g * 180/pi, 'b', t, gamma_g_ref_history * 180/pi, 'b--');
grid on;
title('Gimbal Roll Angle vs Reference');
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('Actual', 'Reference');

% Roll Error
subplot(3,2,6);
plot(t, gamma_g_error * 180/pi, 'b');
grid on;
title('Gimbal Roll Error');
xlabel('Time (s)');
ylabel('Error (degrees)');
legend('Roll Error');

saveas(fig_consolidated, 'results/gimbal_tracking_performance.pdf');


% --- NEW: Full Orientation Tracking Comparison Plot ---
fig_full_orientation = figure('Name', 'Absolute Global Orientation Tracking', 'NumberTitle', 'off');

% Roll Comparison
subplot(3,1,1);
plot(t, gimbal_global_roll_hist * 180/pi, 'b', 'LineWidth', 1.5);
hold on;
plot(t, fw_global_roll_hist * 180/pi, 'r--', 'LineWidth', 1.5);
grid on;
title('Global Roll Angle Comparison');
ylabel('Angle (degrees)');
legend('Gimbal', 'Fixed-Wing');

% Pitch Comparison
subplot(3,1,2);
plot(t, gimbal_global_pitch_hist * 180/pi, 'b', 'LineWidth', 1.5);
hold on;
plot(t, fw_global_pitch_hist * 180/pi, 'r--', 'LineWidth', 1.5);
grid on;
title('Global Pitch Angle Comparison');
ylabel('Angle (degrees)');
legend('Gimbal', 'Fixed-Wing');

% Yaw Comparison
subplot(3,1,3);
plot(t, gimbal_global_yaw_hist * 180/pi, 'b', 'LineWidth', 1.5);
hold on;
plot(t, fw_global_yaw_hist * 180/pi, 'r--', 'LineWidth', 1.5);
grid on;
title('Global Yaw Angle Comparison');
xlabel('Time (s)');
ylabel('Angle (degrees)');
legend('Gimbal', 'Fixed-Wing');
saveas(fig_full_orientation, 'results/global_full_orientation_comparison.pdf');


disp('Real-time simulation and plotting complete.');
%}
end
