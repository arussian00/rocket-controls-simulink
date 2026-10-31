function m = drift_correction_margins(D, thrust, mass, mu, lambda)
%DRIFT_CORRECTION_MARGINS  Outer-loop margins with the lateral PD closed.
%
%   m = drift_correction_margins(D, thrust, mass, mu, lambda)
%
%   The drift PD commands attitude. Breaking the loop at that command,
%   with the correction fully faded in (q below qEngage):
%
%       L = (driftKvy*s + driftKy) * Tcmd(s) * (thrust/mass) / s^2
%
%   Tcmd is the attitude command response, prefilter and sensor delay
%   included. margin uses the exact delay. The stable flag is the Padé test
%   in loop_margins.

flight = struct('name', "return", 'mu', mu, 'lambda', lambda);
P = pitch_plant(flight, D);
Cpi = tf(D.Kp) + tf(D.Ki, [1 0]);
DtermOverCpi = tf([D.Kd, 0, 0], conv([D.Tf, 1], [D.Kp, D.Ki]));
Del = tf(1, 1, 'InputDelay', P.delay);
Fp = tf(1, [D.cmdTau 1]);
M = P.rigid * P.actuator * Cpi;
H = Del * (1 + DtermOverCpi);
Tcmd = Fp * feedback(M, H);
Fm = thrust / mass;
L = tf([D.driftKvy, D.driftKy], 1) * Tcmd * Fm * tf(1, [1 0 0]);
m = loop_margins(L);
m.thrustOverMass = Fm;
m.mass = mass;
m.mu = mu;
m.lambda = lambda;
end
