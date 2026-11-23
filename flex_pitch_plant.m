function P = flex_pitch_plant(mp, slosh, bend, V)
%FLEX_PITCH_PLANT  Pitch plant from a_cmd to measured theta, with slosh and bending.
%
%   P = flex_pitch_plant(mp, slosh, bend, V)
%
%   The rigid piece matches pitch_plant. With slosh and bending left empty
%   and lambda = 0, the loop C(s)*P.sys is the same 64 deg / 19.4 dB plant
%   (the actuator sits on delta_cmd = a_cmd/mu, and mu*delta cancels).
%
%   Slosh coordinates eta [m] are the bob's lateral displacement relative
%   to the body. Specific force is body-fixed (thrust), so the spring does
%   not contain a theta term:
%
%       [ I      , m*xB ] [theta'']   [ lambda*I*theta + mu*I*delta ]
%       [ xB     , 1    ] [eta''  ] = [ -wn^2*eta - 2*zeta*wn*etad   ]
%
%   I already includes the frozen propellant. xB is positive forward of the CG.
%
%   Bending coordinates are mass-normalized. The gimbal side force is
%   T*delta with T = mu*I/arm, applied as -phiG*T*delta on the modal
%   equation (the lateral force sign that also produces +mu*delta on the
%   rigid pitch equation). The IMU reads the local slope:
%
%       theta_meas = theta - dphiImu * eta_bend
%
%   P.sys includes the second-order actuator and the sensor delay.

if nargin < 2 || isempty(slosh)
    slosh = repmat(struct('ms', {}, 'xB', {}, 'wn', {}, 'zeta', {}), 1, 0);
end
if nargin < 3 || isempty(bend)
    bend = repmat(struct('wn', {}, 'zeta', {}, 'phiG', {}, 'dphiImu', {}), 1, 0);
end

ns = numel(slosh);
nb = numel(bend);
n = 2 + 2 * ns + 2 * nb + 2;
A = zeros(n);
B = zeros(n, 1);
C = zeros(1, n);

I = mp.I;
mu = mp.mu;
lam = mp.lambda;
arm = mp.arm;
if ~(isfinite(mu) && mu > 1e-5 && isfinite(I) && I > 0 && isfinite(arm) && arm > 0.5)
    error('flex_pitch_plant:BadPoint', ...
        'Need a finite mu, inertia and gimbal arm to build the flex plant.');
end

nm = 1 + ns;
M = eye(nm);
M(1, 1) = I;
for i = 1:ns
    M(1, 1 + i) = slosh(i).ms * slosh(i).xB;
    M(1 + i, 1) = slosh(i).xB;
end

% rhs = F * state, then [theta''; eta''] = M \ rhs.
% delta is a state, so a_cmd does not feed the mechanics directly.
F = zeros(nm, n);
F(1, 1) = lam * I;
for i = 1:ns
    % State 1 is theta, state 2 is q. Slosh pairs start at 3.
    iEta = 3 + 2 * (i - 1);
    F(1 + i, iEta) = -(slosh(i).wn^2);
    F(1 + i, iEta + 1) = -2 * slosh(i).zeta * slosh(i).wn;
end
iDelta = n - 1;
F(1, iDelta) = mu * I;
acc = M \ F;

A(1, 2) = 1;
A(2, :) = acc(1, :);
for i = 1:ns
    iEta = 3 + 2 * (i - 1);
    A(iEta, iEta + 1) = 1;
    A(iEta + 1, :) = acc(1 + i, :);
end

Tgimbal = mu * I / arm;
for i = 1:nb
    iE = 3 + 2 * ns + 2 * (i - 1);
    A(iE, iE + 1) = 1;
    A(iE + 1, iE) = -(bend(i).wn^2);
    A(iE + 1, iE + 1) = -2 * bend(i).zeta * bend(i).wn;
    A(iE + 1, iDelta) = -bend(i).phiG * Tgimbal;
end

actWn = V.actWn;
A(iDelta, n) = 1;
A(n, iDelta) = -(actWn^2);
A(n, n) = -2 * V.actZeta * actWn;
B(n) = (actWn^2) / mu;

C(1) = 1;
for i = 1:nb
    iE = 3 + 2 * ns + 2 * (i - 1);
    C(iE) = -bend(i).dphiImu;
end

P.sys = ss(A, B, C, 0);
P.sys.InputDelay = V.sensorDelay;
P.sys.InputName = 'a_cmd';
P.sys.OutputName = 'theta_meas';
P.mu = mu;
P.lambda = lam;
P.nSlosh = ns;
P.nBend = nb;
end
