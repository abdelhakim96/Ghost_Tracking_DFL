function state_dot = unified_dynamics(t, state, fw_params, fw_controls, dfl_gains)
    % This function orchestrates the simulation of a quadrotor tracking a fixed-wing aircraft.

    % Unpack the combined state vector (quadrotor state is now 17 elements)
    quad_state = state(1:17);
    fw_state = state(18:30);

    % --- Fixed-Wing Trajectory Generation ---
    % Use control inputs from the fw_controls struct.
    % Check if the controls are time-varying or constant.
    if isfield(fw_controls, 't_sim') && isvector(fw_controls.thrust)
        % Time-varying controls: interpolate for the current time t.
        thrust = interp1(fw_controls.t_sim, fw_controls.thrust, t, 'linear', 'extrap');
        elevator = interp1(fw_controls.t_sim, fw_controls.elevator, t, 'linear', 'extrap');
        aileron = interp1(fw_controls.t_sim, fw_controls.aileron, t, 'linear', 'extrap');
        rudder = interp1(fw_controls.t_sim, fw_controls.rudder, t, 'linear', 'extrap');
    else
        % Constant controls: use the scalar values directly.
        thrust = fw_controls.thrust;
        elevator = fw_controls.elevator;
        aileron = fw_controls.aileron;
        rudder = fw_controls.rudder;
    end

    % Calculate fixed-wing dynamics using the 6-DOF model.
    [fw_state_dot, ref_acc_ned, ref_jerk_ned, ref_snap_ned] = fw_6dof_quat(t, fw_state, thrust, elevator, aileron, rudder, fw_params);

    % --- Define Reference Trajectory for Quadrotor from Fixed-Wing State ---
    % The reference is the dynamic state of the fixed-wing aircraft, provided in the NED frame.
    
    % Reference Position and Velocity (NED frame)
    ref_pos_ned = fw_state(1:3);
    ref_vel_ned = fw_state_dot(1:3); % This is v_ned from fw_6dof_quat output

    % --- Pass Reference Trajectory to Quadrotor Controller ---
    % The DFL controller uses the full state of the fixed-wing as the reference.
    fw_orientation = fw_state(7:10); % Pass quaternion to the controller

    % Yaw schedule (paper Eq. (16)): psi_M_ref = yaw( qbar_tilt (x) q_A )
    % q_tilt is the minimum-angle quaternion that aligns body z with the
    % desired thrust direction (a_des + g e3).
    global g
    a_des = ref_acc_ned(:);
    e3 = [0; 0; 1];
    % NED convention used here: gravity acts along +z. Required body-z (in world)
    % such that f_thrust_world + gravity = m * a_des, with thrust along -z_body
    % cancelled by writing T R e3 - m g e3 = m a_des  =>  b3_des = (a_des + g e3)/|.|
    n_vec = a_des + g*e3;
    nn = norm(n_vec);
    if nn < 1e-6
        b3_des = e3;
    else
        b3_des = n_vec / nn;
    end
    q_tilt = vec_to_quat(e3, b3_des);
    q_tmp  = quat_mul(quat_conj(q_tilt), fw_orientation(:));
    % Yaw via forward-axis projection: well-defined unless the forward axis is
    % near vertical (loops at pitch = +-90 deg). Standard ZYX atan2 is singular
    % there and feeds noise into the high-gain DFL, so fall back to the last
    % stable value when the horizontal projection is small.
    persistent psid_last
    if isempty(psid_last), psid_last = 0; end
    fwd = quatrotate_v(q_tmp, [1;0;0]);
    horiz = hypot(fwd(1), fwd(2));
    if horiz > 0.1
        psid_scheduled = atan2(fwd(2), fwd(1));
        psid_last = psid_scheduled;
    else
        psid_scheduled = psid_last;
    end

    quad_state_dot = quadrotor_dynamics_realtime(t, quad_state, ref_pos_ned, ref_vel_ned, ref_acc_ned, ref_jerk_ned, ref_snap_ned, psid_scheduled, fw_state, fw_orientation, dfl_gains);

    % --- Combine Derivatives ---
    state_dot = [quad_state_dot; fw_state_dot];
end
