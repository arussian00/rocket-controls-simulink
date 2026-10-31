function D = design_pitch_controller(V, opts)
%DESIGN_PITCH_CONTROLLER  Gain-scheduled pitch PID and its margins.
%
%   D = design_pitch_controller(V)
%   D = design_pitch_controller(V, plot=false, verbose=false)
%
%   The controller commands a pitch acceleration. The Simulink model (and
%   the gain schedule) turns that into a gimbal angle with delta = a_cmd/mu,
%   mu = F*arm/I at the flight condition. One set of PID gains is used at
%   every condition; only mu, lambda and the disturbance gain change.
%
%   The Simulink law is a cascade with that loop gain. Derivative on
%   measured rate (no derivative kick):
%
%       q_cmd = Kth*e + KiTh*integral(e)
%       a_cmd = Kq*(q_cmd - s/(Tf*s+1)*theta_meas)
%       Kp = Kq*Kth,  Kd = Kq,  Ki = Kq*KiTh
%
%   Margins use the 1-DOF PID tf(pid(...)), which has the same loop gain.
%   The command prefilter and the command governor sit outside the loop.
%   The governor is inactive on the 5 deg design step.
%
%   Open the same plant interactively with open_pitch_tuner.

arguments
    V (1,1) struct
    opts.plot (1,1) logical = true
    opts.verbose (1,1) logical = true
end

require_control_toolbox();

% Inner rate loop Kq, outer attitude loop Kth, attitude integrator KiTh.
% Overrides V.Kp / V.Ki / V.Kd still win, for a PID Tuner paste-back.
D.Kq = gain_or(V, 'Kq', V.Kq);
D.Kth = gain_or(V, 'Kth', V.attWn^2 / D.Kq);
D.KiTh = gain_or(V, 'KiTh', V.attKiOverWn3 * V.attWn^3 / D.Kq);
D.Kp = gain_or(V, 'Kp', D.Kq * D.Kth);
D.Ki = gain_or(V, 'Ki', D.Kq * D.KiTh);
D.Kd = gain_or(V, 'Kd', D.Kq);
% A pasted PID override wins. Rebuild the cascade so the diagram matches
% the margins: Kd is the rate gain.
if abs(D.Kp - D.Kq * D.Kth) > 1e-9 || abs(D.Ki - D.Kq * D.KiTh) > 1e-9 ...
        || abs(D.Kd - D.Kq) > 1e-9
    D.Kq = D.Kd;
    D.Kth = D.Kp / max(D.Kd, 1e-9);
    D.KiTh = D.Ki / max(D.Kd, 1e-9);
end
D.Tf = gain_or(V, 'Tf', V.attTf);
D.kaw = V.kaw;
D.cmdTau = V.cmdTau;
D.etaAuth = V.etaAuth;
D.shapeKs = V.shapeKs;
D.trimTau = V.trimTau;
D.shapeQ = D.etaAuth / max(D.Kq, 1e-9);       % q_lim = shapeQ * a_avail
D.shapeE = D.etaAuth / max(D.Kp, 1e-6);      % e_lim = shapeE * a_avail
D.qLimFloor = deg2rad(V.thetaStepDeg) / V.cmdTau * 1.25;
D.eLimFloor = deg2rad(V.thetaStepDeg) * 1.2;
D.deltaAvailFloor = V.deltaAvailFrac * V.gimbalMax;
D.CNa = V.CNa;
D.area = V.area;
D.lrBlend = V.lrBlend;
D.qEngage = V.qEngage;
D.qPeak = V.qPeak;
D.lrVfloor = V.lrVfloor;
D.windShear = V.windShear;
D.windShearTime = V.windShearTime;
D.windGust = V.windGust;
D.windGustStart = V.windGustStart;
D.windGustDur = V.windGustDur;
D.loadReliefStop = V.loadReliefStop;
D.pid = pid(D.Kp, D.Ki, D.Kd, D.Tf);
D.Ctf = tf(D.pid);

D.sensorDelay = V.sensorDelay;
D.actHz = V.actHz;
D.actZeta = V.actZeta;
D.actWn = V.actWn;
D.gimbalRate = V.gimbalRate;
D.gimbalRateDeg = V.gimbalRateDeg;
D.deltaMax = V.gimbalMax;
D.deltaMaxDeg = V.gimbalMaxDeg;
D.thetaStepDeg = V.thetaStepDeg;
D.thetaStepTime = V.thetaStepTime;
D.gustSpeed = V.gustSpeed;
D.gustTime = V.gustTime;
D.tStop = V.tStop;
D.vehicleName = V.vehicleName;

n = numel(V.flight);
for k = 1:n
    D.flight(k) = design_one(V.flight(k), V, D); %#ok<AGROW>
end

D.delaySweep = sweep_delay(V, D);
D.actuatorSweep = sweep_actuator(V, D);
D.marginSweep = sweep_static_margin(V, D);

if opts.verbose
    print_report(V, D);
end
if opts.plot
    plot_design(D);
end
end


function flight = design_one(raw, V, D)
flight = raw;
P = pitch_plant(raw, V);
[L, Tcmd, Tdist] = loop_transfers(P, D);

flight.plant = P;
flight.L = L;
flight.Tcmd = Tcmd;
flight.Tdist = Tdist;
flight.margins = loop_margins(L);
flight.dcGain = safe_dcgain(Tcmd);
flight.bandwidth = safe_bandwidth(Tcmd);
[flight.step2, flight.step5] = step_metrics(Tcmd);
end


function [L, Tcmd, Tdist] = loop_transfers(P, D)
% D-on-measurement command response, and the 1-DOF loop gain.
%   L     = Cpid * Actuator * rigid * delay
%   Tcmd  = theta / theta_cmd_raw      (prefilter included, derivative on measurement)
%   Tdist = theta / disturbance acceleration, added after the actuator
%
% H is built as a polynomial so the shared integrator in Cpid and Cpi is
% cancelled explicitly: Cpid/Cpi = 1 + (Kd*s^2)/((Tf*s+1)*(Kp*s+Ki)).
Cpid = D.Ctf;
Cpi = tf(D.Kp) + tf(D.Ki, [1 0]);
DtermOverCpi = tf([D.Kd, 0, 0], conv([D.Tf, 1], [D.Kp, D.Ki]));
Del = tf(1, 1, 'InputDelay', P.delay);
Fp = tf(1, [D.cmdTau 1]);

L = Cpid * P.G;                 % P.G already carries the sensor delay
M = P.rigid * P.actuator * Cpi;
H = Del * (1 + DtermOverCpi);
% feedback() accepts the input delay and stores it as an internal delay.
% step/bode/margin can use that model. isstable, pole and minreal cannot;
% those paths go through rational_delay instead.
Tref = feedback(M, H);          % response to the filtered command
Tcmd = Fp * Tref;
Tdist = feedback(P.rigid, Cpid * P.actuator * Del);

L.Name = "loop " + P.flightName;
Tcmd.Name = "theta/cmd " + P.flightName;
Tdist.Name = "theta/d " + P.flightName;
end


function rows = sweep_delay(V, D)
% How much of the phase margin is the sensor delay, at max-q.
flight = flight_named(V.flight, "max-q");
delays = [0, 0.010, 0.020, 0.050, 0.100];
rows = repmat(empty_row(), 1, numel(delays));
for k = 1:numel(delays)
    P = pitch_plant(flight, V, delay=delays(k));
    m = loop_margins(D.Ctf * P.G);
    rows(k) = pack_row(m, delays(k), P.actHz, flight.staticMarginCal);
end
end


function rows = sweep_actuator(V, D)
flight = flight_named(V.flight, "max-q");
hz = [3, 4, 6, 8, 10];
rows = repmat(empty_row(), 1, numel(hz));
for k = 1:numel(hz)
    P = pitch_plant(flight, V, actHz=hz(k));
    m = loop_margins(D.Ctf * P.G);
    rows(k) = pack_row(m, P.delay, hz(k), flight.staticMarginCal);
end
end


function rows = sweep_static_margin(V, D)
flight = flight_named(V.flight, "max-q");
margins = [0, -0.5, -2, -5, -20, -50];
rows = repmat(empty_row(), 1, numel(margins));
for k = 1:numel(margins)
    lam = aero_lambda(flight, V, margins(k));
    P = pitch_plant(flight, V, lambda=lam);
    m = loop_margins(D.Ctf * P.G);
    rows(k) = pack_row(m, P.delay, P.actHz, margins(k));
    rows(k).unstablePole = sqrt(max(lam, 0));
end
end


function lam = aero_lambda(flight, V, staticMarginCal)
lcp = -staticMarginCal * V.diam;
lam = flight.q * V.CNa * V.area * lcp / flight.inertia;
end


function r = empty_row()
r = struct('pm', NaN, 'wc', NaN, 'gmUp', NaN, 'wGmUp', NaN, ...
    'stable', false, 'delay', NaN, 'actHz', NaN, ...
    'staticMarginCal', NaN, 'unstablePole', NaN);
end


function r = pack_row(m, delay, actHz, sm)
r = empty_row();
r.pm = m.pm;
r.wc = m.wc;
r.gmUp = m.gmUp;
r.wGmUp = m.wGmUp;
r.stable = m.stable;
r.delay = delay;
r.actHz = actHz;
r.staticMarginCal = sm;
end


function wb = safe_bandwidth(Tcmd)
% bandwidth() rejects internal delays. freqresp does not.
try
    wb = bandwidth(Tcmd);
    if ~isempty(wb) && all(isfinite(wb))
        return
    end
catch
end
try
    w = logspace(-2, 2, 2000);
    h = squeeze(freqresp(Tcmd, w));
    mag = abs(h(:));
    dc = mag(1);
    below = find(mag < dc/sqrt(2), 1, 'first');
    if isempty(below) || below == 1
        wb = NaN;
    else
        wb = w(below);
    end
catch
    wb = NaN;
end
end


function dc = safe_dcgain(sys)
% dcgain() is not reliable once feedback() has stored the delay internally.
try
    dc = dcgain(sys);
    if isfinite(dc)
        return
    end
catch
end
try
    h = freqresp(sys, 1e-4);
    dc = real(h(1));
catch
    dc = NaN;
end
end


function [info2, info5] = step_metrics(sys)
% stepinfo() calls isstable on some releases and errors on internal delays.
% step() is the supported simulation. Both thresholds share one response.
% The reference is 1: Tcmd is the response to a unit pitch command.
t = linspace(0, 90, 4501).';
y = step_delay(sys, t);
info5 = metrics_at(t, y, 0.05);
info2 = metrics_at(t, y, 0.02);
end


function info = metrics_at(t, y, thresh)
ref = 1;
info.RiseTime = crossing_time(t, y, 0.9*ref) - crossing_time(t, y, 0.1*ref);
peak = max(y);
info.Overshoot = max(0, (peak - ref) / abs(ref) * 100);
info.Peak = peak;
[~, iPeak] = max(y);
info.PeakTime = t(iPeak);
outside = find(abs(y - ref) > thresh * abs(ref));
if isempty(outside)
    info.SettlingTime = 0;
elseif outside(end) == numel(t)
    info.SettlingTime = NaN;
else
    info.SettlingTime = t(outside(end));
end
end


function tCross = crossing_time(t, y, level)
idx = find(y >= level, 1, 'first');
if isempty(idx)
    tCross = NaN;
elseif idx == 1
    tCross = t(1);
else
    dy = y(idx) - y(idx-1);
    if dy == 0
        tCross = t(idx);
    else
        frac = (level - y(idx-1)) / dy;
        tCross = t(idx-1) + frac * (t(idx) - t(idx-1));
    end
end
end


function flight = flight_named(flights, name)
idx = find(string({flights.name}) == string(name), 1);
if isempty(idx)
    error('design_pitch_controller:Unknown', ...
        'No flight condition named "%s".', name);
end
flight = flights(idx);
end


function print_report(V, D)
fprintf('\nSuper Heavy pitch TVC  (gain-scheduled PID on angular acceleration)\n');
fprintf('Companion target:  wn = %.2f rad/s,  zeta = %.2f\n', V.attWn, V.attZeta);
fprintf('Cascade: Kq = %.3g 1/s, Kth = %.3g 1/s, Ki_th = %.3g 1/s^2\n', ...
    D.Kq, D.Kth, D.KiTh);
fprintf('Equivalent damping zeta = %.2f. Companion PD uses zeta = %.2f.\n', ...
    D.Kd / (2 * sqrt(max(D.Kp, 1e-9))), V.attZeta);
fprintf('Equivalent PID: Kp = %.4g, Ki = %.4g, Kd = %.4g, Tf = %.3f s\n', ...
    D.Kp, D.Ki, D.Kd, D.Tf);
fprintf('Command governor: eta = %.2f, rate floor %.1f deg/s, lead floor %.1f deg\n', ...
    D.etaAuth, rad2deg(D.qLimFloor), rad2deg(D.eLimFloor));
fprintf('Actuator %.1f Hz, zeta %.2f, rate limit %.0f deg/s, angle limit %.0f deg\n', ...
    V.actHz, V.actZeta, V.gimbalRateDeg, V.gimbalMaxDeg);
fprintf('Sensor delay %.0f ms,  command prefilter %.2f s (outside the loop)\n', ...
    1e3*V.sensorDelay, V.cmdTau);
fprintf(['Load relief: lr_blend = %.2f, full authority for q >= %.0f kPa ' ...
    '(ascent peak %.1f kPa).\n' ...
    'The blend edits the attitude command. It does not add loop gain, so the\n' ...
    'margins below are the margins with load relief active.\n'], ...
    D.lrBlend, D.qEngage/1e3, D.qPeak/1e3);
fprintf(['Stability uses a 6th-order Padé model (isstable cannot see internal\n' ...
    'delays). Margins, Bode and step responses use the exact delay.\n\n']);

fprintf('%-10s %8s %8s %8s %10s %8s %8s %8s %8s %8s\n', ...
    'Condition', 'm [t]', 'T/W', 'q [kPa]', 'mu [1/s2]', ...
    'wu', 'PM', 'GM up', 'wc', 'stable');
for k = 1:numel(D.flight)
    f = D.flight(k);
    m = f.margins;
    fprintf('%-10s %8.1f %8.2f %8.2f %10.3f %8.3f %8.1f %8.1f %8.2f %8s\n', ...
        f.name, f.mass/1e3, f.thrustToWeight, f.q/1e3, f.mu, ...
        f.unstablePole, m.pm, m.gmUp, m.wc, string(m.stable));
end

fprintf('\nCommand response to a %.0f deg step (prefilter on, limits off):\n', ...
    V.thetaStepDeg);
fprintf('%-10s %8s %8s %8s %8s %10s %8s\n', ...
    'Condition', 'Tr 10-90', 'OS %', 'Ts 5%', 'Ts 2%', 'bw [rad/s]', 'DC');
for k = 1:numel(D.flight)
    f = D.flight(k);
    fprintf('%-10s %8.2f %8.1f %8.2f %8.2f %10.2f %8.3f\n', ...
        f.name, f.step5.RiseTime, f.step5.Overshoot, ...
        f.step5.SettlingTime, f.step2.SettlingTime, ...
        f.bandwidth, f.dcGain);
end
fprintf(['Ts 5%% is the rigid-body response. Ts 2%% is longer because Ki is\n' ...
    'weak on purpose: it trims a constant moment without adding much overshoot.\n']);

f = flight_named(D.flight, "max-q");
fprintf('\nDownward gain margin (integrator) at max-q: %.1f dB at %.3f rad/s\n', ...
    f.margins.gmDown, f.margins.wGmDown);
fprintf('Extra delay margin at max-q: %.2f s\n', f.margins.delayMargin);
fprintf('MATLAB margin() at max-q: GM %.2f dB at %.2f rad/s, PM %.2f deg at %.2f rad/s\n', ...
    f.margins.matlabGm, f.margins.matlabWcg, f.margins.matlabPm, f.margins.matlabWcp);

fprintf('\nMax-q, sensor delay sweep (same gains):\n');
print_sweep(D.delaySweep, 'delay [ms]', @(r) 1e3*r.delay);
fprintf('\nMax-q, actuator bandwidth sweep:\n');
print_sweep(D.actuatorSweep, 'act [Hz]', @(r) r.actHz);
fprintf('\nMax-q, static-margin sweep (negative = unstable):\n');
print_sweep(D.marginSweep, 'SM [cal]', @(r) r.staticMarginCal);

fprintf(['\nInteractive tuning (normalized plant, a_cmd -> theta):\n' ...
    '  open_pitch_tuner("pid")\n' ...
    '  open_pitch_tuner("designer", "max-q")\n' ...
    'Gains you read in the tuner are Kp, Ki, Kd above. Gimbal [rad] = a_cmd/mu.\n\n']);
end


function print_sweep(rows, label, valueOf)
fprintf('%-12s %8s %8s %8s %8s\n', label, 'PM [deg]', 'wc', 'GM up', 'stable');
for k = 1:numel(rows)
    r = rows(k);
    fprintf('%-12.3g %8.1f %8.2f %8.1f %8s\n', ...
        valueOf(r), r.pm, r.wc, r.gmUp, string(r.stable));
end
end


function plot_design(D)
names = [D.flight.name];
figure('Name', 'Pitch TVC loop gain');
bode_exact(D.flight(1).L, D.flight(2).L, D.flight(3).L, {0.05, 200});
grid on
legend(cellstr(names), 'Location', 'southwest');
title('Loop gain  a_{cmd} back to a_{cmd}');

figure('Name', 'Pitch TVC command step');
tCmd = linspace(0, 20, 500);
hold on
for k = 1:numel(D.flight)
    y = step_delay(D.flight(k).Tcmd, tCmd);
    plot(tCmd, y * D.thetaStepDeg, 'LineWidth', 1.2);
end
grid on
xlabel('Time [s]');
ylabel('Pitch [deg]');
title(sprintf('Closed-loop response to a %.0f deg command', D.thetaStepDeg));
legend(cellstr(names), 'Location', 'southeast');

figure('Name', 'Pitch TVC disturbance');
tDist = linspace(0, 30, 600);
tiledlayout(1, 2);
nexttile
hold on
misDeg = 0.5;
for k = 1:numel(D.flight)
    y = step_delay(D.flight(k).Tdist, tDist);
    plot(tDist, rad2deg(y * D.flight(k).mu * deg2rad(misDeg)), 'LineWidth', 1.2);
end
grid on
xlabel('Time [s]');
ylabel('Pitch [deg]');
title('0.5 deg thrust misalignment');
legend(cellstr(names), 'Location', 'northeast');

nexttile
f = flight_named(D.flight, "max-q");
y = step_delay(f.Tdist, tDist);
plot(tDist, rad2deg(y * f.gustAccPerWind * D.gustSpeed), 'LineWidth', 1.2);
grid on
xlabel('Time [s]');
ylabel('Pitch [deg]');
title(sprintf('%.0f m/s gust at max-q', D.gustSpeed));

figure('Name', 'Open-loop pitch drift at max-q');
plot_open_loop_drift(f, D);
end


function plot_open_loop_drift(flight, D)
% Rigid body only, so the slow unstable pole is obvious. Actuator, delay
% and the prefilter are in the Bode plot and in the Simulink model.
lam = flight.lambda;
Aol = [0, 1; lam, 0];
sysOL = ss(Aol, [0; 1], [1, 0], 0);
Acl = [0, 1, 0; ...
       lam - D.Kp, -D.Kd, D.Ki; ...
       -1, 0, 0];
sysCL = ss(Acl, zeros(3, 1), [1, 0, 0], 0);
t = linspace(0, 90, 400);
[yOL, tOL] = initial(sysOL, [deg2rad(1); 0], t);
[yCL, tCL] = initial(sysCL, [deg2rad(1); 0; 0], t);
plot(tOL, rad2deg(yOL), tCL, rad2deg(yCL), 'LineWidth', 1.2);
grid on
xlabel('Time [s]');
ylabel('Pitch [deg]');
legend('Open loop', 'Closed loop (rigid body)', 'Location', 'northwest');
title(sprintf('Max-q, 1 deg initial pitch, unstable pole %.3f rad/s', ...
    flight.unstablePole));
end


function bode_exact(varargin)
% bode supports input delays. If a release still refuses the model, draw
% the Padé loop and warn rather than dropping the figure.
try
    bode(varargin{:});
catch ME
    warning('design_pitch_controller:Bode', ...
        ['bode() could not use the exact delay (%s). ' ...
         'The figure is a 6th-order Padé approximation.'], ME.message);
    args = varargin;
    for k = 1:numel(args)
        if isa(args{k}, 'DynamicSystem') || isa(args{k}, 'lti')
            args{k} = rational_delay(args{k}, 6);
        end
    end
    bode(args{:});
end
end


function value = gain_or(V, fieldName, fallback)
if isfield(V, fieldName) && ~isempty(V.(fieldName))
    value = V.(fieldName);
else
    value = fallback;
end
end


function require_control_toolbox()
if ~license('test', 'Control_Toolbox')
    error('design_pitch_controller:NeedsCST', ...
        'Control System Toolbox is required (tf, pid, margin, step, feedback).');
end
end
