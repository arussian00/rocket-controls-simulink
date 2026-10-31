function apply_flight_schedule(traj, deltaMax)
%APPLY_FLIGHT_SCHEDULE  Point the Simulink lookups at the booster trajectory.
%
%   apply_flight_schedule(traj, deltaMax)
%
%   mu, lambda and 1/mu become functions of flight time. When the engines
%   are off, 1/mu and the acceleration limit are 0 so the gimbal command
%   stays at zero instead of dividing by zero. Breakpoints are clipped
%   outside the table.

t = traj.t_s(:).';
mu = traj.mu_rps2(:).';
lam = traj.lambda_rps2(:).';
[t, ia] = unique(t, 'stable');
mu = mu(ia);
lam = lam(ia);
invMu = zeros(size(mu));
hot = mu > 1e-4;
invMu(hot) = 1 ./ mu(hot);
assignin('base', 'schedTime', t);
assignin('base', 'schedMu', mu);
assignin('base', 'schedLambda', lam);
assignin('base', 'schedInvMu', invMu);
assignin('base', 'schedACmdMax', mu * deltaMax);
end
