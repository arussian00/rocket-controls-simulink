function r = mc_closed_loop(D, plant, sc)
%MC_CLOSED_LOOP  Fixed-step model of the pitch diagram.
%
%   r = mc_closed_loop(D, plant, sc)
%
%   Same signals as boosterPitchTVC: command prefilter, sensor transport
%   delay, command governor, cascade with back-calculation, second-order
%   gimbal servo with angle and rate stops, and
%   theta'' = mu_true*sin(delta) + lambda*theta + gustAcc*wind.
%   The controller divides by plant.muCmd. The plant accelerates with
%   plant.muTrue. Step size defaults to 2 ms, which reproduces the
%   published nominal 5 deg and 30 deg rows (gimbal, rate, stop dwell,
%   final pitch) within a few tenths.
%
%   sc.stepDeg, sc.tStop, sc.shapeOn, sc.gustSpeed, sc.gustTime, sc.dt,
%   sc.store (logical, default false). When store is true, r.t and r.errDeg
%   are downsampled to about 20 ms. r.errDeg is theta - theta_cmd, noise free.

if ~isfield(sc, 'gustSpeed'), sc.gustSpeed = 0; end
if ~isfield(sc, 'gustTime'), sc.gustTime = D.gustTime; end
if ~isfield(sc, 'shapeOn'), sc.shapeOn = 1; end
if ~isfield(sc, 'dt'), sc.dt = 0.002; end
if ~isfield(sc, 'store'), sc.store = false; end

dt = sc.dt;
n = max(round(sc.tStop / dt), 1);
nd = max(round(plant.delay / dt), 0);
Kq = D.Kq;
Kth = D.Kth;
KiTh = D.KiTh;
Kaw = D.kaw;
KawA = D.kaw / max(Kq, 1e-9);
wn = 2 * pi * plant.actHz;
zeta = plant.actZeta;
rateLim = plant.rate;
deltaMax = D.deltaMax;
muCmd = plant.muCmd;
muTrue = plant.muTrue;
lam = plant.lambda;
gustAcc = plant.gustAcc;

% pref trim shaped integ qf rate delta q theta
x = zeros(9, 1);
if nd > 0
    bufTh = zeros(nd, 1);
    bufQ = zeros(nd, 1);
else
    bufTh = 0;
    bufQ = 0;
end
slot = 1;

peakGim = 0;
peakRate = 0;
tOnAngle = 0;
tOnRate = 0;
overshoot = 0;
tail = 0;
final = 0;
storeN = 0;
if sc.store
    stepOut = max(round(0.02 / dt), 1);
    nStore = ceil(n / stepOut);
    tStore = zeros(nStore, 1);
    eStore = zeros(nStore, 1);
else
    stepOut = 0;
    tStore = [];
    eStore = [];
end

angTol = deg2rad(0.05);
rateTol = deg2rad(0.05);
stepRad = deg2rad(sc.stepDeg);

for k = 1:n
    if nd == 0
        thD = x(9);
        qD = x(8);
    else
        thD = bufTh(slot);
        qD = bufQ(slot);
    end
    t = (k - 1) * dt;
    if t >= 1
        thetaCmd = stepRad;
    else
        thetaCmd = 0;
    end
    if t >= sc.gustTime
        wind = sc.gustSpeed;
    else
        wind = 0;
    end

    pref = x(1); trim = x(2); shaped = x(3); integ = x(4); qf = x(5);
    rate = x(6); delta = x(7); q = x(8); theta = x(9);

    dAvail = deltaMax - abs(trim);
    if dAvail < D.deltaAvailFloor
        dAvail = D.deltaAvailFloor;
    end
    aAvail = muCmd * sin(dAvail);
    qLim = D.shapeQ * aAvail;
    if qLim < D.qLimFloor
        qLim = D.qLimFloor;
    end
    eLim = D.shapeE * aAvail;
    if eLim < D.eLimFloor
        eLim = D.eLimFloor;
    end
    aMax = muCmd * deltaMax;

    lead = clip(pref - thD, eLim);
    sdot = clip(D.shapeKs * ((thD + lead) - shaped), qLim);
    if sc.shapeOn >= 0.5
        thetaRef = shaped;
    else
        thetaRef = pref;
    end
    e = thetaRef - thD;
    qUnsat = Kth * e + integ;
    if sc.shapeOn >= 0.5
        qCmd = clip(qUnsat, qLim);
    else
        qCmd = qUnsat;
    end
    aRaw = Kq * (qCmd - qf);
    aSat = clip(aRaw, aMax);
    deltaCmd = aSat / muCmd;

    acc = wn * wn * (deltaCmd - delta) - 2 * zeta * wn * rate;
    if rate >= rateLim && acc > 0
        acc = 0;
    elseif rate <= -rateLim && acc < 0
        acc = 0;
    end
    dDelta = rate;
    if delta >= deltaMax && dDelta > 0
        dDelta = 0;
    elseif delta <= -deltaMax && dDelta < 0
        dDelta = 0;
    end
    dq = muTrue * sin(delta) + lam * theta + gustAcc * wind;

    x(1) = pref + dt * (thetaCmd - pref) / D.cmdTau;
    x(2) = trim + dt * (delta - trim) / D.trimTau;
    x(3) = shaped + dt * sdot;
    x(4) = integ + dt * (KiTh * e + Kaw * (qCmd - qUnsat) + KawA * (aSat - aRaw));
    x(5) = qf + dt * (qD - qf) / D.Tf;
    x(6) = clip(rate + dt * acc, rateLim);
    x(7) = clip(delta + dt * dDelta, deltaMax);
    x(8) = q + dt * dq;
    x(9) = theta + dt * q;

    if nd > 0
        bufTh(slot) = x(9);
        bufQ(slot) = x(8);
        slot = slot + 1;
        if slot > nd
            slot = 1;
        end
    end

    ag = abs(x(7));
    ar = abs(x(6));
    if ag > peakGim
        peakGim = ag;
    end
    if ar > peakRate
        peakRate = ar;
    end
    t2 = k * dt;
    if ag >= deltaMax - angTol
        tOnAngle = tOnAngle + dt;
    end
    if ar >= rateLim - rateTol
        tOnRate = tOnRate + dt;
    end
    if t2 >= 1
        cmd = stepRad;
    else
        cmd = 0;
    end
    err = x(9) - cmd;
    if t2 >= 1 && err > overshoot
        overshoot = err;
    end
    if t2 >= 10 && abs(err) > tail
        tail = abs(err);
    end
    final = x(9);
    if sc.store && mod(k, stepOut) == 0
        storeN = storeN + 1;
        tStore(storeN) = t2;
        eStore(storeN) = rad2deg(err);
    end
end

r.gimDeg = rad2deg(peakGim);
r.rateDeg = rad2deg(peakRate);
r.tOnAngle = tOnAngle;
r.tOnRate = tOnRate;
r.finalDeg = rad2deg(final);
r.overshootDeg = rad2deg(overshoot);
r.tailDeg = rad2deg(tail);
r.t = tStore(1:storeN);
r.errDeg = eStore(1:storeN);
end


function y = clip(y, lim)
if y > lim
    y = lim;
elseif y < -lim
    y = -lim;
end
end
