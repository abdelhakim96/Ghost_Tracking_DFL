function state_dot = unified_dynamics3(t, state, fw_params, fw_controls, dfl_gains)
% 18-state quad + 13-state FW = 31 elements combined.
    quad_state = state(1:18);
    fw_state   = state(19:31);

    % Reference-governor pace factor (default 1). Emulating the maneuver slowed
    % by lambda<=1 maps real time t to FW-time tau = lambda*t; the k-th time
    % derivative of the reference then scales by lambda^k. This keeps the exact
    % DFL inverse inside the feasible set F without changing the controller.
    lam = 1; if isfield(fw_controls,'lambda') && ~isempty(fw_controls.lambda), lam = fw_controls.lambda; end
    tau = lam * t;

    if isfield(fw_controls, 't_sim') && isvector(fw_controls.thrust)
        thrust   = interp1(fw_controls.t_sim, fw_controls.thrust,   tau, 'linear', 'extrap');
        elevator = interp1(fw_controls.t_sim, fw_controls.elevator, tau, 'linear', 'extrap');
        aileron  = interp1(fw_controls.t_sim, fw_controls.aileron,  tau, 'linear', 'extrap');
        rudder   = interp1(fw_controls.t_sim, fw_controls.rudder,   tau, 'linear', 'extrap');
    else
        thrust   = fw_controls.thrust;
        elevator = fw_controls.elevator;
        aileron  = fw_controls.aileron;
        rudder   = fw_controls.rudder;
    end

    % Optional attitude-hold autopilot — same wiring as the FW-only test harness
    if isfield(fw_controls, 'stabilize_after') && tau >= fw_controls.stabilize_after
        target = struct();
        if isfield(fw_controls, 'stabilize_target'), target = fw_controls.stabilize_target; end
        gains  = struct();
        if isfield(fw_controls, 'stabilize_gains'),  gains  = fw_controls.stabilize_gains;  end
        alt_ref = [];
        if isfield(fw_controls, 'stabilize_alt_ref'), alt_ref = fw_controls.stabilize_alt_ref; end
        [del, dai, dru] = fw_attitude_hold(fw_state, target, gains, alt_ref);
        elevator = elevator + del;
        aileron  = aileron  + dai;
        rudder   = rudder   + dru;
    end

    [fw_sd, racc, rjerk, rsnap] = fw_6dof_quat(tau, fw_state, thrust, elevator, aileron, rudder, fw_params);

    % Governed (real-time) reference: y(lambda*t) => k-th derivative * lambda^k.
    % fw_state_dot is slowed by lambda so the FW traces the same path at the
    % governed pace; the camera-pose reference derivatives scale accordingly.
    fw_state_dot = lam * fw_sd;
    ref_pos_ned  = fw_state(1:3);
    ref_vel_ned  = lam   * fw_sd(1:3);
    ref_acc_ned  = lam^2 * racc;
    ref_jerk_ned = lam^3 * rjerk;
    ref_snap_ned = lam^4 * rsnap;

    fw_orientation = fw_state(7:10);
    fw_state_sc = fw_state; fw_state_sc(11:13) = lam * fw_state(11:13);   % real-time body rate

    % With 3-axis gimbal, psid is no longer needed (passed as 0).
    quad_state_dot = quadrotor_dynamics_realtime3(t, quad_state, ...
        ref_pos_ned, ref_vel_ned, ref_acc_ned, ref_jerk_ned, ref_snap_ned, ...
        0, fw_state_sc, fw_orientation, dfl_gains);

    state_dot = [quad_state_dot; fw_state_dot];
end
