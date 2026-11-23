function apply_flight_schedule(traj, D, opts)
%APPLY_FLIGHT_SCHEDULE  Point the Simulink lookups at the booster trajectory.
%
%   apply_flight_schedule(traj, D)
%   apply_flight_schedule(traj, D, useWind=true)
%
%   mu, lambda, 1/mu, speed, dynamic pressure and the lateral-acceleration
%   gains become functions of flight time. When the engines are off, 1/mu
%   and the acceleration limit are 0 so the gimbal command stays at zero.
%   useWind replaces the zero wind table with the shear plus 1-cosine gust.

arguments
    traj
    D (1,1) struct
    opts.useWind (1,1) logical = false
end

t = traj.t_s(:).';
mu = traj.mu_rps2(:).';
lam = traj.lambda_rps2(:).';
speed = traj.speed_mps(:).';
qdyn = traj.q_Pa(:).';
mass = traj.mass_kg(:).';
thrust = traj.thrust_N(:).';
[t, ia] = unique(t, 'stable');
mu = mu(ia);
lam = lam(ia);
speed = speed(ia);
qdyn = qdyn(ia);
mass = mass(ia);
thrust = thrust(ia);
invMu = zeros(size(mu));
hot = mu > 1e-4;
invMu(hot) = 1 ./ mu(hot);
accThrust = thrust ./ mass;
accAero = qdyn * D.CNa * D.area ./ mass;
if opts.useWind
    wind = load_relief_wind(D, t(:)).';
else
    wind = zeros(size(t));
end

assignin('base', 'schedTime', t);
assignin('base', 'schedMu', mu);
assignin('base', 'schedLambda', lam);
assignin('base', 'schedInvMu', invMu);
assignin('base', 'schedACmdMax', mu * D.deltaMax);
assignin('base', 'schedSpeed', speed);
assignin('base', 'schedQdyn', qdyn);
assignin('base', 'schedAccThrust', accThrust);
assignin('base', 'schedAccAero', accAero);
assignin('base', 'schedWind', wind);
end
