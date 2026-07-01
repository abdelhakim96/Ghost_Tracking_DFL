function g = critically_damped_gains(omega, base)
% critically_damped_gains  One-parameter stabilizing gains for the DFL chains.
%
% After exact input-output linearization, each output i satisfies the
% integrator chain  y_i^(r_i) = v_i.  The tracking law
%   v_i = y_ref^(r_i) + sum_j c_{i,j} ( y_ref^(j) - y^(j) )
% makes the error obey  e_i^(r_i) + c_{r_i-1} e^(r_i-1) + ... + c_0 e = 0.
% Placing ALL poles of this certified error ODE at a single location -omega
% (critically damped, monotone decay, no overshoot) FORCES the gains to be the
% binomial coefficients of (s+omega)^{r_i}.  There is therefore no arbitrary
% gain vector -- only the single bandwidth omega, which is in turn bounded by
% the feasible set F (largest omega whose flat-inverse inputs respect actuator
% and gimbal-rate limits along the reference).
%
%   omega : scalar bandwidth [rad/s]  (all error-dynamics poles at -omega)
%   base  : optional existing dfl_gains struct to extend (fields overwritten)
%
% Chains:
%   position p_MC   : relative degree 4  -> (s+w)^4
%   drone yaw q3     : relative degree 2  -> (s+w)^2
%   gimbal phi,th,ps : relative degree 1  -> (s+w)

    if nargin < 2 || isempty(base), base = struct(); end
    g = base;
    w = omega;

    % position chain (r = 4):  e'''' + c3 e''' + c2 e'' + c1 e' + c0 e = 0
    g.c0 = w^4;
    g.c1 = 4*w^3;
    g.c2 = 6*w^2;
    g.c3 = 4*w;

    % drone-yaw chain q3 (r = 2)
    g.c_q3     = w^2;
    g.c_q3_dot = 2*w;

    % gimbal chains phi_g, theta_g, psi_g (r = 1)
    g.c_phi   = w;
    g.c_theta = w;
    g.c_psig  = w;

    % vestigial fields kept for interface compatibility
    if ~isfield(g, 'c4'), g.c4 = 1; end
    if ~isfield(g, 'c5'), g.c5 = 1; end
end
