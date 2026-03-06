function state_dot = unified_dynamics(t, state, fw_params, fw_controls, dfl_gains)
% UNIFIED_DYNAMICS  Combined ODE function for fixed-wing + multicopter simulation.
%
% This function orchestrates the coupled simulation:
%   1. Propagates the fixed-wing aircraft trajectory (the "ghost" target)
%   2. Extracts the reference trajectory (position, velocity, acceleration,
%      jerk, snap) from the fixed-wing state for the DFL controller
%   3. Propagates the multicopter+gimbal dynamics with DFL tracking control
%
% The combined state vector is [quad_state(17); fw_state(13)] = 30 elements.
%
% This implements the framework shown in Fig. 3 of the paper:
%   u_A -> f_A(x_A) -> y_AC -> f_M^{-1}(x_M) -> u_MC -> f_M(x_M) -> y_MC

    % -----------------------------------------------------------------------
    % Split combined state into quadrotor and fixed-wing sub-states
    % -----------------------------------------------------------------------
    quad_state = state(1:17);
    fw_state = state(18:30);

    % -----------------------------------------------------------------------
    % Fixed-wing control inputs (may be time-varying for roll maneuver)
    % -----------------------------------------------------------------------
    if isfield(fw_controls, 't_sim') && isvector(fw_controls.thrust)
        % Time-varying controls: interpolate for the current time t
        thrust = interp1(fw_controls.t_sim, fw_controls.thrust, t, 'linear', 'extrap');
        elevator = interp1(fw_controls.t_sim, fw_controls.elevator, t, 'linear', 'extrap');
        aileron = interp1(fw_controls.t_sim, fw_controls.aileron, t, 'linear', 'extrap');
        rudder = interp1(fw_controls.t_sim, fw_controls.rudder, t, 'linear', 'extrap');
    else
        % Constant controls (e.g., loop maneuver)
        thrust = fw_controls.thrust;
        elevator = fw_controls.elevator;
        aileron = fw_controls.aileron;
        rudder = fw_controls.rudder;
    end

    % -----------------------------------------------------------------------
    % Compute fixed-wing dynamics and reference trajectory
    % -----------------------------------------------------------------------
    % fw_6dof_quat returns the state derivative plus NED-frame acceleration,
    % jerk, and snap for the DFL controller's feedforward terms (v_MC in eq. 20).
    [fw_state_dot, ref_acc_ned, ref_jerk_ned, ref_snap_ned] = ...
        fw_6dof_quat(t, fw_state, thrust, elevator, aileron, rudder, fw_params);

    % Reference position (from current FW state) and velocity (from FW derivative)
    ref_pos_ned = fw_state(1:3);
    ref_vel_ned = fw_state_dot(1:3);

    % -----------------------------------------------------------------------
    % Propagate quadrotor dynamics with DFL tracking control
    % -----------------------------------------------------------------------
    quad_state_dot = quadrotor_dynamics_realtime(t, quad_state, ...
        ref_pos_ned, ref_vel_ned, ref_acc_ned, ref_jerk_ned, ref_snap_ned, ...
        0, fw_state, dfl_gains);

    % -----------------------------------------------------------------------
    % Combine derivatives
    % -----------------------------------------------------------------------
    state_dot = [quad_state_dot; fw_state_dot];
end
