function [del, dai, dru] = fw_attitude_hold(fw_state, target, gains, alt_ref)
% fw_attitude_hold  Tiny attitude-hold autopilot for the open-loop FW.
%
%   Engages a PD on (roll, pitch) and a rate-damper on yaw. Outputs
%   *additive* corrections to elevator / aileron / rudder (rad). The
%   scripted maneuver inputs in fw_controls are added on top, so during
%   the scripted phase the autopilot can be disabled and during the
%   stabilisation phase the scripted inputs are usually zero.
%
%   target.phi_ref     — desired roll  (rad)
%   target.theta_ref   — desired pitch (rad)
%   target.r_ref       — desired yaw rate (rad/s)  (use 0 for "hold heading")
%
%   gains.kp_phi, kd_phi      — roll
%   gains.kp_theta, kd_theta  — pitch
%   gains.kd_r                — yaw-rate damping
%   (default gains chosen for the Edge 540 below if any field is missing)

    % Conservative defaults — Edge 540 has very strong control authority
    % (Cl_da = 0.5, Cm_de = -1.8). Big gains cause violent corrections and
    % integrate huge fake body-rotation in the post-maneuver coast.
    % Conservative gains — strong control authority means small gains
    % already give plenty of bandwidth, and high gains were causing the
    % FW to re-flip past inverted in the recovery leg.
    if ~isfield(gains, 'kp_phi'),   gains.kp_phi   = 0.10; end
    if ~isfield(gains, 'kd_phi'),   gains.kd_phi   = 0.12; end
    if ~isfield(gains, 'kp_theta'), gains.kp_theta = 0.18; end
    if ~isfield(gains, 'kd_theta'), gains.kd_theta = 0.15; end
    if ~isfield(gains, 'kd_r'),     gains.kd_r     = 0.05; end
    % Constant elevator trim added on top of PD — Edge 540 has CL0 = 0.4
    % which creates an upward bias the PD can't cancel with zero error.
    if ~isfield(gains, 'elevator_trim'), gains.elevator_trim = +0.010; end
    % Altitude-hold outer loop (optional, only if alt_ref passed in).
    % Output is a pitch reference fed into the inner pitch PD.
    if ~isfield(gains, 'kp_alt'), gains.kp_alt = 0.002; end   % rad pitch per m alt err
    if ~isfield(gains, 'kd_w'),   gains.kd_w   = 0.040; end   % rad pitch per m/s w_world (more damping)
    if ~isfield(gains, 'theta_ref_max'), gains.theta_ref_max = 0.18; end   % stricter pitch ref limit

    if ~isfield(target, 'phi_ref'),   target.phi_ref   = 0; end
    if ~isfield(target, 'theta_ref'), target.theta_ref = 0; end
    if ~isfield(target, 'r_ref'),     target.r_ref     = 0; end

    qa = fw_state(7:10); qa = qa / (norm(qa) + 1e-12);
    p  = fw_state(11);
    q  = fw_state(12);
    r  = fw_state(13);

    [roll, pitch, ~] = quat_to_zyx(qa);

    % Altitude-hold outer loop: pitch reference = -kp*(alt-alt_ref) + Kd*w
    % (positive alt_err -> too high -> nose-down -> negative pitch ref).
    % Wn_world = -z_dot_world; we approximate from R*v_body.
    if nargin >= 4 && ~isempty(alt_ref)
        alt        = -fw_state(3);                          % NED z down -> alt is -z
        % world-frame vertical velocity from body velocity (good enough for nearly level)
        u_b = fw_state(4); v_b = fw_state(5); w_b = fw_state(6);
        R   = quat_to_R_local(qa);
        v_world = R * [u_b; v_b; w_b];
        w_world_up = -v_world(3);                            % +up
        theta_ref = -gains.kp_alt*(alt - alt_ref) - gains.kd_w*w_world_up;
        theta_ref = max(-gains.theta_ref_max, min(gains.theta_ref_max, theta_ref));
    else
        theta_ref = target.theta_ref;
    end

    % Error in roll/pitch (wrap roll to nearest representation of phi_ref)
    e_phi   = wrap(target.phi_ref   - roll);
    e_theta = theta_ref - pitch;            % pitch bounded to +-pi/2, no wrap needed
    e_r     = target.r_ref     - r;

    % Outputs (additive, in rad). Aileron sign: positive aileron -> negative roll.
    %   roll_dynamics: p_dot ~ +Cl_da * delta_a, Cl_da > 0
    %   We want p_dot in direction of -e_phi if PD form is right.
    %   Empirically signs verified for the Edge 540 model used here.
    % Signs derived from the Edge 540 model conventions:
    %   Cl_da = +0.5 -> +aileron causes +roll  -> for damping +roll, use -aileron
    %   Cm_de = -1.8 -> +elevator causes -pitch -> for damping +pitch, use +elevator
    %   Cn_dr = -0.1 -> +rudder   causes -yaw   -> for damping +r, use +rudder
    dai = +gains.kp_phi   * e_phi   - gains.kd_phi   * p;
    del = -gains.kp_theta * e_theta + gains.kd_theta * q + gains.elevator_trim;
    dru = +gains.kd_r     * (r - target.r_ref);

    % Saturate to plausible deflection range
    dai = max(-0.6, min(0.6, dai));
    del = max(-0.4, min(0.4, del));
    dru = max(-0.4, min(0.4, dru));
end

function R = quat_to_R_local(q)
    q = q(:)/max(norm(q),1e-12);
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    R = [q0^2+q1^2-q2^2-q3^2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2);
         2*(q1*q2+q0*q3), q0^2-q1^2+q2^2-q3^2, 2*(q2*q3-q0*q1);
         2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0^2-q1^2-q2^2+q3^2];
end

function [roll, pitch, yaw] = quat_to_zyx(q)
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1^2 + q2^2));
    sinp  = 2*(q0*q2 - q3*q1);
    if abs(sinp) >= 1, pitch = sign(sinp)*pi/2; else, pitch = asin(sinp); end
    yaw   = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2^2 + q3^2));
end

function d = wrap(d)
    d = mod(d + pi, 2*pi) - pi;
end
