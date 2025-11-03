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
    quad_state_dot = quadrotor_dynamics_realtime(t, quad_state, ref_pos_ned, ref_vel_ned, ref_acc_ned, ref_jerk_ned, ref_snap_ned, 0, fw_state, fw_orientation, dfl_gains);

    % --- Combine Derivatives ---
    state_dot = [quad_state_dot; fw_state_dot];
end
