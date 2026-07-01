function qp = quad_platform_params()
% quad_platform_params  Representative ~5 kg multirotor (cinematography /
% medium-lift class, e.g. Freefly Alta-6 scale).
%
% Single source of truth for the emulation platform, used by run_compare3 so
% every maneuver shares consistent vehicle parameters (replaces the unrealistic
% 0.468 kg micro-quad in the configs).
%
% NOTE: under exact feedback linearization the closed-loop OUTPUT tracking and
% the thrust-to-weight / body-rate demands are mass/inertia invariant (set by
% the reference trajectory). These parameters rescale absolute thrust [N] and
% torque [N*m] into realistic units but do not change tracking or feasibility
% ratios.

    qp.m  = 5.0;      % mass [kg]
    qp.Ix = 0.15;     % moment of inertia about body x [kg m^2]
    qp.Iy = 0.15;     % about body y
    qp.Iz = 0.25;     % about body z
    qp.l  = 0.35;     % arm length [m]
    qp.g  = 9.81;     % gravity [m/s^2]

    % Aerodynamic/drag coefficients: unused by the point-mass translational
    % model and the controller (passed as 0); kept for field compatibility.
    qp.Ax = 0.25; qp.Ay = 0.25; qp.Az = 0.25;
    qp.Ap = 0.022; qp.Aq = 0.022; qp.Ar = 0.022;
    qp.d  = 3e-6; qp.b = 1e-5;
end
