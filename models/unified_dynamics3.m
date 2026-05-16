function state_dot = unified_dynamics3(t, state, fw_params, fw_controls, dfl_gains)
% 18-state quad + 13-state FW = 31 elements combined.
    quad_state = state(1:18);
    fw_state   = state(19:31);

    if isfield(fw_controls, 't_sim') && isvector(fw_controls.thrust)
        thrust   = interp1(fw_controls.t_sim, fw_controls.thrust,   t, 'linear', 'extrap');
        elevator = interp1(fw_controls.t_sim, fw_controls.elevator, t, 'linear', 'extrap');
        aileron  = interp1(fw_controls.t_sim, fw_controls.aileron,  t, 'linear', 'extrap');
        rudder   = interp1(fw_controls.t_sim, fw_controls.rudder,   t, 'linear', 'extrap');
    else
        thrust   = fw_controls.thrust;
        elevator = fw_controls.elevator;
        aileron  = fw_controls.aileron;
        rudder   = fw_controls.rudder;
    end

    [fw_state_dot, ref_acc_ned, ref_jerk_ned, ref_snap_ned] = fw_6dof_quat(t, fw_state, thrust, elevator, aileron, rudder, fw_params);

    ref_pos_ned = fw_state(1:3);
    ref_vel_ned = fw_state_dot(1:3);

    fw_orientation = fw_state(7:10);

    % With 3-axis gimbal, psid is no longer needed (passed as 0).
    quad_state_dot = quadrotor_dynamics_realtime3(t, quad_state, ...
        ref_pos_ned, ref_vel_ned, ref_acc_ned, ref_jerk_ned, ref_snap_ned, ...
        0, fw_state, fw_orientation, dfl_gains);

    state_dot = [quad_state_dot; fw_state_dot];
end
