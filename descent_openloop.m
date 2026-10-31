function S = descent_openloop(ret, tq, D)
%DESCENT_OPENLOOP  Allocation, limits and commands along a time grid.
%
%   S = descent_openloop(ret, tq, D)
%
%   tq is increasing. Continuous CSV columns are interpolated. The control
%   phase holds the last sample at or before tq. The mode weight and the
%   mu used as a divisor are first-order filters with the same time
%   constant, so an engine-count step cannot spike a deflection command.
%   Closed-loop states (pitch, integrator, servos) are not in here.

G = ret.G;
tq = tq(:);
tq = min(max(tq, ret.t(1)), ret.t(end));
n = numel(tq);
S.t = tq;

S.muTvc = interp1(ret.t, ret.muTvc, tq, 'linear');
S.muFin = interp1(ret.t, ret.muFin, tq, 'linear');
S.lambdaFin = interp1(ret.t, ret.lambdaFin, tq, 'linear');
S.inertia = interp1(ret.t, ret.inertia, tq, 'linear');
S.q = interp1(ret.t, ret.q, tq, 'linear');
S.nEng = interp1(ret.t, ret.nEng, tq, 'previous');
S.h = interp1(ret.t, ret.h, tq, 'linear');
S.vx = interp1(ret.t, ret.vx, tq, 'linear');
S.vh = interp1(ret.t, ret.vh, tq, 'linear');
S.phaseCode = interp1(ret.t, ret.phaseCode, tq, 'previous');
S.phaseCode = round(S.phaseCode);

S.thetaCmd = theta_on_grid(S.phaseCode, S.h, S.vx, S.vh, G);
S.thetaTrim = atan2(-S.vx, max(abs(S.vh), G.aeroVhFloor));

% Steady targets, then the blend filter. g is the exact discrete pole.
wTgt = zeros(n, 3);
for k = 1:n
    wTgt(k, :) = descent_targets(S.phaseCode(k), S.muTvc(k), S.muFin(k)).';
end
if n >= 2
    dt = median(diff(tq));
else
    dt = G.dt;
end
g = 1 - exp(-dt / G.blendTau);
w = [0, 0, 1];
muAt = 0;
muAf = 0;
S.wTvc = zeros(n, 1);
S.wFin = zeros(n, 1);
S.wRcs = zeros(n, 1);
S.muAllocTvc = zeros(n, 1);
S.muAllocFin = zeros(n, 1);
for k = 1:n
    w = w + g * (wTgt(k, :) - w);
    total = sum(w);
    if total > 1e-9
        w = w / total;
    end
    muAt = muAt + g * (S.muTvc(k) - muAt);
    muAf = muAf + g * (S.muFin(k) - muAf);
    S.wTvc(k) = w(1);
    S.wFin(k) = w(2);
    S.wRcs(k) = w(3);
    S.muAllocTvc(k) = muAt;
    S.muAllocFin(k) = muAf;
end

S.invTvc = zeros(n, 1);
S.invFin = zeros(n, 1);
hotT = S.muAllocTvc > G.muTvcFloor;
hotF = S.muAllocFin > G.muFinFloor;
S.invTvc(hotT) = 1 ./ S.muAllocTvc(hotT);
S.invFin(hotF) = 1 ./ S.muAllocFin(hotF);

S.gateT = double(S.phaseCode == 2 | S.phaseCode == 5 | S.phaseCode == 6 | S.wTvc > 0.02);
S.gateF = double(S.phaseCode == 3 | S.phaseCode == 4 | S.phaseCode == 5 | S.phaseCode == 6);
S.gateR = double(S.phaseCode == 1 | S.wRcs > 0.02);

S.aRcs = G.tauRCS ./ max(S.inertia, 1);
S.aMax = zeros(n, 1);
useT = S.gateT > 0 & S.muTvc > 0;
S.aMax(useT) = S.aMax(useT) + S.muTvc(useT) * sin(D.deltaMax);
S.aMax = S.aMax + S.gateF .* S.muFin .* G.finDeltaMax;
S.aMax = S.aMax + S.gateR .* S.aRcs;

burnLead = D.eLimFloor;
burnRate = D.qLimFloor;
finsOnly = S.gateF > 0 & S.wTvc < 0.2 & S.wRcs < 0.2 ...
    & (S.phaseCode == 3 | S.phaseCode == 4);
S.eFloor = burnLead * ones(n, 1);
S.qFloor = burnRate * ones(n, 1);
S.eFloor(S.phaseCode == 1) = G.flipLead;
S.qFloor(S.phaseCode == 1) = G.flipRate;
S.eFloor(finsOnly) = G.finLeadFloor;
S.qFloor(finsOnly) = G.finRateFloor;
end


function thetaCmd = theta_on_grid(phaseCode, h, vx, vh, G)
thetaCmd = atan2(-vx, max(abs(vh), G.aeroVhFloor));
boost = phaseCode == 1 | phaseCode == 2;
thetaCmd(boost) = -pi/2;
land = phaseCode == 5 | phaseCode == 6;
if any(land)
    hAbove = h(land) - G.hLand;
    fade = min(1, max(hAbove - G.fadeUpright, 0) / G.fadeSpan);
    tiltLim = deg2rad(G.maxTiltDeg) .* fade;
    raw = atan2(-vx(land), max(abs(vh(land)), G.landingVhFloor));
    thetaCmd(land) = min(max(raw, -tiltLim), tiltLim);
end
end
