function R = descent_controller(D, V, ret)
%DESCENT_CONTROLLER  Return pitch law: margins and a fixed-step simulation.
%
%   R = descent_controller(D, V, ret)
%
%   One acceleration command, the same cascade as design_pitch_controller
%   (Kq, Kth, KiTh, back-calculation, command governor). A blend weight
%   splits it across RCS (the flip), TVC and the grid fins. The weight and
%   the divisor mu are filtered with the same time constant, and each
%   surface command is then rate-limited, so a mode change does not step
%   the command or the integrator.
%
%   R.sim is the fixed-step nonlinear loop (no Simulink). R.margins is the
%   linear loop along the return. Stability on the phase anchors uses
%   loop_margins (Padé orders 4 and 6). The along-track curve skips that
%   test, same as flight_margin_schedule.

if nargin < 3 || isempty(ret)
    ret = make_booster_return_trajectory([], V);
end
G = ret.G;

fprintf('Fixed-step return simulation (dt = %.0f ms)...\n', 1e3*G.dt);
R.ret = ret;
R.sim = simulate_return(D, V, ret);
R.margins = return_margins(D, V, ret);
R.G = G;
end


function sim = simulate_return(D, V, ret)
G = ret.G;
N = round((ret.t(end) - ret.t(1)) / G.dt) + 1;
tq = ret.t(1) + (0:N-1).' * G.dt;
S = descent_openloop(ret, tq, D);

theta = ret.theta0;
qpit = 0;
pref = theta;
shaped = theta;
integ = 0;
qf = 0;
trim = 0;
tvAng = 0;
tvRate = 0;
fnAng = 0;
fnRate = 0;
rcs = 0;
cmdT = 0;
cmdF = 0;
cmdR = 0;

nd = max(1, round(G.sensorDelay / G.dt));
bufTh = theta * ones(nd, 1);
bufQ = zeros(nd, 1);
bi = 1;

wnT = V.actWn;
wnF = G.finWn;
Kq = D.Kq;
Kth = D.Kth;
KiTh = D.KiTh;
Kaw = D.kaw;
KawA = Kaw / max(Kq, 1e-9);

dec = 10;
nLog = floor(N / dec) + 2;
sim.t = zeros(nLog, 1);
sim.theta = zeros(nLog, 1);
sim.thetaCmd = zeros(nLog, 1);
sim.shaped = zeros(nLog, 1);
sim.fin = zeros(nLog, 1);
sim.tvc = zeros(nLog, 1);
sim.finCmd = zeros(nLog, 1);
sim.tvcCmd = zeros(nLog, 1);
sim.integ = zeros(nLog, 1);
sim.wTvc = zeros(nLog, 1);
sim.wFin = zeros(nLog, 1);
sim.wRcs = zeros(nLog, 1);
sim.phaseCode = zeros(nLog, 1);
li = 0;

tOnFin = 0;
tOnFinRate = 0;
tOnTvc = 0;
tOnTvcRate = 0;
maxStepT = 0;
maxStepF = 0;
maxStepI = 0;
prevT = 0;
prevF = 0;
prevI = 0;

for k = 1:N
    thM = bufTh(bi);
    qM = bufQ(bi);

    pref = pref + G.dt / D.cmdTau * (S.thetaCmd(k) - pref);
    qf = qf + G.dt / D.Tf * (qM - qf);
    trim = trim + G.dt / D.trimTau * (tvAng - trim);

    dAvail = max(D.deltaMax - abs(trim), G.deltaAvailFrac * D.deltaMax);
    aT = 0;
    if S.muTvc(k) > 0
        aT = S.muTvc(k) * sin(dAvail) * S.gateT(k);
    end
    fAvail = max(G.finDeltaMax - abs(fnAng), G.deltaAvailFrac * G.finDeltaMax);
    aF = S.muFin(k) * fAvail * S.gateF(k);
    aR = S.aRcs(k) * S.gateR(k);
    aAvail = aT + aF + aR;

    eLim = max(D.shapeE * aAvail, S.eFloor(k));
    qLim = max(D.shapeQ * aAvail, S.qFloor(k));

    lead = clip(pref - thM, -eLim, eLim);
    target = thM + lead;
    shaped = shaped + G.dt * clip(D.shapeKs * (target - shaped), -qLim, qLim);

    err = shaped - thM;
    qCmd = Kth * err + integ;
    qUsed = clip(qCmd, -qLim, qLim);
    aRaw = Kq * (qUsed - qf);
    if S.aMax(k) > 0
        aSat = clip(aRaw, -S.aMax(k), S.aMax(k));
    else
        aSat = 0;
    end

    desT = clip(S.wTvc(k) * aSat * S.invTvc(k), -D.deltaMax, D.deltaMax);
    desF = clip(S.wFin(k) * aSat * S.invFin(k), -G.finDeltaMax, G.finDeltaMax);
    desR = clip(S.wRcs(k) * aSat, -S.aRcs(k), S.aRcs(k));

    cmdT = cmdT + G.dt * clip(D.shapeKs * (desT - cmdT), -V.gimbalRate, V.gimbalRate);
    cmdF = cmdF + G.dt * clip(D.shapeKs * (desF - cmdF), -G.finRate, G.finRate);
    rRate = max(aR, 0.2) / G.rcsTau;
    cmdR = cmdR + G.dt * clip(D.shapeKs * (desR - cmdR), -rRate, rRate);

    integ = integ + G.dt * (KiTh * err + Kaw * (qUsed - qCmd) + KawA * (aSat - aRaw));

    [tvAng, tvRate] = step_servo(tvAng, tvRate, cmdT, wnT, V.actZeta, ...
        V.gimbalRate, D.deltaMax, G.dt);
    [fnAng, fnRate] = step_servo(fnAng, fnRate, cmdF, wnF, G.finZeta, ...
        G.finRate, G.finDeltaMax, G.dt);
    rcs = rcs + G.dt / G.rcsTau * (cmdR - rcs);

    alpha = clip(theta - S.thetaTrim(k), -G.alphaCap, G.alphaCap);
    aPlant = S.lambdaFin(k) * alpha + S.muFin(k) * fnAng + rcs;
    if S.muTvc(k) > 0
        aPlant = aPlant + S.muTvc(k) * sin(tvAng);
    end
    qpit = qpit + G.dt * aPlant;
    theta = theta + G.dt * qpit;

    bufTh(bi) = theta;
    bufQ(bi) = qpit;
    bi = bi + 1;
    if bi > nd
        bi = 1;
    end

    if abs(fnAng) >= G.finDeltaMax - deg2rad(0.05)
        tOnFin = tOnFin + G.dt;
    end
    if abs(fnRate) >= G.finRate - deg2rad(0.05)
        tOnFinRate = tOnFinRate + G.dt;
    end
    if abs(tvAng) >= D.deltaMax - deg2rad(0.05)
        tOnTvc = tOnTvc + G.dt;
    end
    if abs(tvRate) >= V.gimbalRate - deg2rad(0.05)
        tOnTvcRate = tOnTvcRate + G.dt;
    end
    maxStepT = max(maxStepT, abs(cmdT - prevT));
    maxStepF = max(maxStepF, abs(cmdF - prevF));
    maxStepI = max(maxStepI, abs(integ - prevI));
    prevT = cmdT;
    prevF = cmdF;
    prevI = integ;

    if mod(k - 1, dec) == 0
        li = li + 1;
        sim.t(li) = tq(k);
        sim.theta(li) = theta;
        sim.thetaCmd(li) = S.thetaCmd(k);
        sim.shaped(li) = shaped;
        sim.fin(li) = fnAng;
        sim.tvc(li) = tvAng;
        sim.finCmd(li) = cmdF;
        sim.tvcCmd(li) = cmdT;
        sim.integ(li) = integ;
        sim.wTvc(li) = S.wTvc(k);
        sim.wFin(li) = S.wFin(k);
        sim.wRcs(li) = S.wRcs(k);
        sim.phaseCode(li) = S.phaseCode(k);
    end
end

sim.t = sim.t(1:li);
sim.theta = sim.theta(1:li);
sim.thetaCmd = sim.thetaCmd(1:li);
sim.shaped = sim.shaped(1:li);
sim.fin = sim.fin(1:li);
sim.tvc = sim.tvc(1:li);
sim.finCmd = sim.finCmd(1:li);
sim.tvcCmd = sim.tvcCmd(1:li);
sim.integ = sim.integ(1:li);
sim.wTvc = sim.wTvc(1:li);
sim.wFin = sim.wFin(1:li);
sim.wRcs = sim.wRcs(1:li);
sim.phaseCode = sim.phaseCode(1:li);

sim.thetaDeg = rad2deg(sim.theta);
sim.thetaCmdDeg = rad2deg(sim.thetaCmd);
sim.shapedDeg = rad2deg(sim.shaped);
sim.finDeg = rad2deg(sim.fin);
sim.tvcDeg = rad2deg(sim.tvc);
sim.finCmdDeg = rad2deg(sim.finCmd);
sim.tvcCmdDeg = rad2deg(sim.tvcCmd);

sim.catchErrDeg = sim.thetaDeg(end) - sim.thetaCmdDeg(end);
sim.peakFinDeg = max(abs(sim.finDeg));
sim.peakTvcDeg = max(abs(sim.tvcDeg));
sim.tOnFin = tOnFin;
sim.tOnFinRate = tOnFinRate;
sim.tOnTvc = tOnTvc;
sim.tOnTvcRate = tOnTvcRate;
sim.maxStepTvcDeg = rad2deg(maxStepT);
sim.maxStepFinDeg = rad2deg(maxStepF);
sim.maxStepInteg = maxStepI;
sim.transitions = transition_metrics(sim, ret.events);
sim.finLimitDeg = G.finDeltaMaxDeg;
sim.tvcLimitDeg = D.deltaMaxDeg;
end


function M = return_margins(D, V, ret)
G = ret.G;
keep = 1;
for i = 2:numel(ret.t)
    if ret.t(i) - ret.t(keep(end)) >= 4
        keep(end + 1) = i; %#ok<AGROW>
    end
end
keep(end + 1) = numel(ret.t);
keep = unique(keep, 'stable');

n = numel(keep);
M.t = ret.t(keep);
M.phase = ret.controlPhase(keep);
M.phaseCode = ret.phaseCode(keep);
M.q = ret.q(keep);
M.muTvc = ret.muTvc(keep);
M.muFin = ret.muFin(keep);
M.nEng = ret.nEng(keep);
M.pm = nan(n, 1);
M.gm = nan(n, 1);
M.wc = nan(n, 1);
M.controlled = false(n, 1);
M.floorGm = 6;
M.floorPm = 30;

fprintf('Return margins at %d samples...\n', n);
for k = 1:n
    i = keep(k);
    [aMax, wt, wf, wr] = steady_authority(ret, i, D, G);
    M.controlled(k) = aMax >= G.authorityMin && (wt + wf + wr) > 0.5;
    if ~M.controlled(k)
        continue
    end
    m = loop_at(D, V, G, ret.lambdaFin(i), wt, wf, wr, false);
    M.pm(k) = m.pm;
    M.gm(k) = m.gmUp;
    M.wc(k) = m.wc;
end
M.below = M.controlled & ~(M.pm >= M.floorPm & M.gm >= M.floorGm);
M.nBelow = nnz(M.below);
M.anchors = phase_anchors(D, V, ret);

if any(M.controlled)
    controlledGm = M.gm;
    controlledGm(~M.controlled) = inf;
    [~, iW] = min(controlledGm);
    fprintf(['  Along the return, worst controlled GM %.1f dB, PM %.1f deg at t = %.0f s (%s). ' ...
        'Samples under 6 dB or 30 deg: %d.\n'], ...
        M.gm(iW), M.pm(iW), M.t(iW), char(M.phase(iW)), M.nBelow);
else
    fprintf('  No return sample had pitch authority.\n');
end
fprintf('  Phase anchors (Padé stability on the order-6 model):\n');
fprintf('  %-22s %8s %8s %8s %8s %8s %8s\n', ...
    'Phase', 't [s]', 'q [kPa]', 'nEng', 'PM', 'GM up', 'stable');
for k = 1:numel(M.anchors)
    a = M.anchors(k);
    if ~a.controlled
        fprintf('  %-22s %8.0f %8.2f %8.0f %s\n', ...
            a.name, a.t, a.q/1e3, a.nEng, 'no pitch authority');
        continue
    end
    stab = 'no';
    if a.stable
        stab = 'yes';
    end
    fprintf('  %-22s %8.0f %8.2f %8.0f %8.1f %8.1f %8s\n', ...
        a.name, a.t, a.q/1e3, a.nEng, a.pm, a.gm, stab);
end
end


function anchors = phase_anchors(D, V, ret)
G = ret.G;
% One Padé check per control phase, plus the peak-q entry sample, the
% first landing sample (13 engines just became 3) and the catch.
wanted = { ...
    1, "flip, RCS"; ...
    2, "boostback, TVC 13"; ...
    3, "coast, above the fins"; ...
    4, "entry, peak dynamic pressure"; ...
    5, "brake, TVC+fins"; ...
    6, "landing, 3 engines"; ...
    6, "catch"};
anchors = struct('name', {}, 't', {}, 'q', {}, 'nEng', {}, 'pm', {}, ...
    'gm', {}, 'wc', {}, 'stable', {}, 'controlled', {}, 'mode', {});
for k = 1:size(wanted, 1)
    code = wanted{k, 1};
    name = wanted{k, 2};
    idx = find(ret.phaseCode == code);
    if isempty(idx)
        continue
    end
    if name == "entry, peak dynamic pressure"
        [~, j] = max(ret.q(idx));
        i = idx(j);
    elseif name == "catch"
        i = idx(end);
    elseif name == "landing, 3 engines"
        i = idx(1);
    else
        i = idx(round(numel(idx) / 2));
    end
    [aMax, wt, wf, wr] = steady_authority(ret, i, D, G);
    controlled = aMax >= G.authorityMin && (wt + wf + wr) > 0.5;
    pm = NaN;
    gm = NaN;
    wc = NaN;
    stable = false;
    if controlled
        m = loop_at(D, V, G, ret.lambdaFin(i), wt, wf, wr, true);
        pm = m.pm;
        gm = m.gmUp;
        wc = m.wc;
        stable = m.stable;
    end
    item = struct( ...
        'name', char(name), ...
        't', ret.t(i), ...
        'q', ret.q(i), ...
        'nEng', ret.nEng(i), ...
        'pm', pm, ...
        'gm', gm, ...
        'wc', wc, ...
        'stable', stable, ...
        'controlled', controlled, ...
        'mode', mode_name(wt, wf, wr));
    if isempty(anchors)
        anchors = item;
    else
        anchors(end + 1) = item; %#ok<AGROW>
    end
end
end


function [aMax, wt, wf, wr] = steady_authority(ret, i, D, G)
w = descent_targets(ret.phaseCode(i), ret.muTvc(i), ret.muFin(i));
wt = w(1);
wf = w(2);
wr = w(3);
code = ret.phaseCode(i);
gateT = code == 2 || code == 5 || code == 6;
gateF = code == 3 || code == 4 || code == 5 || code == 6;
gateR = code == 1;
aMax = 0;
if gateT && ret.muTvc(i) > 0
    aMax = aMax + ret.muTvc(i) * sin(D.deltaMax);
end
if gateF
    aMax = aMax + ret.muFin(i) * G.finDeltaMax;
end
if gateR
    aMax = aMax + G.tauRCS / max(ret.inertia(i), 1);
end
end


function name = mode_name(wt, wf, wr)
if wr > 0.5
    name = 'RCS';
elseif wt > 0.8 && wf < 0.2
    name = 'TVC';
elseif wf > 0.8 && wt < 0.2
    name = 'fins';
else
    name = 'blend';
end
end


function m = loop_at(D, V, G, lambdaFin, wt, wf, wr, doStable)
total = wt + wf + wr;
if total > 0
    wt = wt / total;
    wf = wf / total;
    wr = wr / total;
end
ActT = tf(V.actWn^2, [1, 2*V.actZeta*V.actWn, V.actWn^2]);
ActF = tf(G.finWn^2, [1, 2*G.finZeta*G.finWn, G.finWn^2]);
ActR = tf(1, [G.rcsTau, 1]);
Act = wt * ActT + wf * ActF + wr * ActR;
P = tf(1, [1, 0, -lambdaFin]);
L = D.Ctf * Act * P;
L.InputDelay = V.sensorDelay;
m = loop_margins(L, checkStability=doStable);
end


function T = transition_metrics(sim, events)
T = struct('name', {}, 't', {}, 'maxDTvcDeg', {}, 'maxDFinDeg', {}, ...
    'maxDInteg', {}, 'maxDThetaDeg', {});
for k = 1:numel(events)
    m = abs(sim.t - events(k).t) <= 1;
    idx = find(m);
    T(k).name = events(k).name;
    T(k).t = events(k).t;
    if numel(idx) < 3
        T(k).maxDTvcDeg = NaN;
        T(k).maxDFinDeg = NaN;
        T(k).maxDInteg = NaN;
        T(k).maxDThetaDeg = NaN;
        continue
    end
    T(k).maxDTvcDeg = max(abs(diff(sim.tvcCmdDeg(idx))));
    T(k).maxDFinDeg = max(abs(diff(sim.finCmdDeg(idx))));
    T(k).maxDInteg = max(abs(diff(sim.integ(idx))));
    T(k).maxDThetaDeg = max(abs(diff(sim.thetaDeg(idx))));
end
end


function [ang, rate] = step_servo(ang, rate, cmd, wn, zeta, rateMax, angMax, dt)
acc = wn^2 * (cmd - ang) - 2 * zeta * wn * rate;
rate = clip(rate + dt * acc, -rateMax, rateMax);
angN = ang + dt * rate;
if angN > angMax
    angN = angMax;
    if rate > 0
        rate = 0;
    end
elseif angN < -angMax
    angN = -angMax;
    if rate < 0
        rate = 0;
    end
end
ang = angN;
end


function y = clip(x, lo, hi)
y = min(max(x, lo), hi);
end
