%% RUN_PITCH_TVC  Design the pitch loop, build the Simulink model, simulate, plot.
%
%   In MATLAB Online, from this folder:
%
%       run_pitch_tvc
%
%   Needs Simulink and Control System Toolbox. Does not need the Aerospace
%   Blockset or Simulink Control Design.
%
%   Prints stability margins, builds boosterPitchTVC.slx, runs the design
%   sims plus a max-q load-relief sweep, and opens one dashboard.
%   The command window still prints the PASS/REVIEW checklist. REVIEW means
%   "look at this number", not that the script failed. The two 30 deg
%   liftoff runs are the same step with the command governor off, then on.
%   The design steps use lr_blend = 0 so the linear overlay stays valid.
%   The load-relief sweep uses the shear plus 1-cosine wind.

clear; close all; clc

V = vehicle_params();
D = design_pitch_controller(V, plot=false);
traj = booster_trajectory();
schedule = flight_margin_schedule(D, V, traj);
mdl = build_pitch_model();

nominal = ["liftoff", "max-q", "landing"];
simNominal = cell(1, numel(nominal));
for k = 1:numel(nominal)
    fprintf('Simulating %s ...\n', nominal(k));
    simNominal{k} = run_case(D, mdl, nominal(k), lrBlend=0, driftCorrOn=0);
end

fprintf('Simulating liftoff, 30 deg step, command governor off ...\n');
simBefore = run_case(D, mdl, "liftoff", ...
    thetaStepDeg=V.largeStepDeg, gustSpeed=0, tStop=V.largeStepStop, ...
    shapeOn=0, lrBlend=0, driftCorrOn=0);
simBefore.label = "30 deg before";

fprintf('Simulating liftoff, 30 deg step, command governor on ...\n');
simAfter = run_case(D, mdl, "liftoff", ...
    thetaStepDeg=V.largeStepDeg, gustSpeed=0, tStop=V.largeStepStop, ...
    shapeOn=1, lrBlend=0, driftCorrOn=0);
simAfter.label = "30 deg after";

fprintf('Simulating max-q, 0.5 deg thrust misalignment ...\n');
simMis = run_case(D, mdl, "max-q", ...
    thetaStepDeg=0, gustSpeed=0, misalignDeg=0.5, misalignTime=2, ...
    tStop=40, lrBlend=0, driftCorrOn=0);

blendGrid = 0:0.1:1;
nBlend = numel(blendGrid);
trade.blend = blendGrid(:);
trade.peakQa = zeros(nBlend, 1);
trade.peakDrift = zeros(nBlend, 1);
trade.finalDrift = zeros(nBlend, 1);
trade.peakGim = zeros(nBlend, 1);
trade.peakQaOff = zeros(nBlend, 1);
trade.peakDriftOff = zeros(nBlend, 1);
trade.finalDriftOff = zeros(nBlend, 1);
trade.current = V.lrBlend;
simLR = cell(1, 3);
simLRoff = cell(1, 3);
nKept = 0;
for k = 1:nBlend
    fprintf('Load relief lr_blend = %.2f, drift correction on ...\n', blendGrid(k));
    result = run_load_relief(D, traj, mdl, blendGrid(k), 1);
    trade.peakQa(k) = result.peakQa;
    trade.peakDrift(k) = result.peakDrift;
    trade.finalDrift(k) = result.finalDrift;
    trade.peakGim(k) = max(abs(result.deltaDeg));
    fprintf('Load relief lr_blend = %.2f, drift correction off ...\n', blendGrid(k));
    bare = run_load_relief(D, traj, mdl, blendGrid(k), 0);
    trade.peakQaOff(k) = bare.peakQa;
    trade.peakDriftOff(k) = bare.peakDrift;
    trade.finalDriftOff(k) = bare.finalDrift;
    if any(abs(blendGrid(k) - [0, 0.5, 1]) < 1e-6)
        nKept = nKept + 1;
        simLR{nKept} = result;
        simLRoff{nKept} = bare;
    end
end
outerMargins = drift_margin_samples(D, traj);

close all
pitchDashboard = pitch_dashboard(D, simNominal, simBefore, simAfter, simMis, ...
    schedule, traj, simLR, trade, simLRoff); %#ok<NASGU>
apply_pitch_condition(D, "max-q", thetaStepDeg=0, gustSpeed=0, ...
    tStop=D.loadReliefStop, shapeOn=1, lrBlend=D.lrBlend);
apply_flight_schedule(traj, D, useWind=true);
print_checks(D, simNominal, simBefore, simAfter, simMis, simLR, outerMargins);


%% ------------------------------------------------------------------------
function result = run_case(D, mdl, name, opts)
arguments
    D (1,1) struct
    mdl (1,1) string
    name (1,1) string
    opts.thetaStepDeg (1,1) double = NaN
    opts.thetaStepTime (1,1) double = NaN
    opts.gustSpeed (1,1) double = NaN
    opts.gustTime (1,1) double = NaN
    opts.misalignDeg (1,1) double = 0
    opts.misalignTime (1,1) double = 2
    opts.tStop (1,1) double = NaN
    opts.shapeOn (1,1) double = NaN
    opts.lrBlend (1,1) double = NaN
    opts.driftCorrOn (1,1) double = NaN
end

args = named_args(opts);
apply_pitch_condition(D, name, args{:});
out = sim(char(mdl), 'ReturnWorkspaceOutputs', 'on');
result = pack_sim(D, name, out);
end


function result = run_load_relief(D, traj, mdl, lrBlend, driftCorrOn)
% Hold the trajectory attitude (no pitch step) through the max-q wind.
apply_pitch_condition(D, "max-q", thetaStepDeg=0, gustSpeed=0, ...
    tStop=D.loadReliefStop, shapeOn=1, lrBlend=lrBlend, driftCorrOn=driftCorrOn);
apply_flight_schedule(traj, D, useWind=true);
out = sim(char(mdl), 'ReturnWorkspaceOutputs', 'on');
result = pack_sim(D, "max-q", out);
result.label = "LR " + sprintf('%.1f', lrBlend);
result.reportLoad = true;
result.marginFrom = "max-q";
result.lrBlend = lrBlend;
end


function result = pack_sim(D, name, out)
[t, theta] = unpack_log(fetch_log(out, 'theta'));
[~, q] = unpack_log(fetch_log(out, 'q'));
[~, delta] = unpack_log(fetch_log(out, 'delta'));
[~, deltaCmd] = unpack_log(fetch_log(out, 'deltaCmd'));
[~, rate] = unpack_log(fetch_log(out, 'gimbalRateMeas'));
[~, thetaCmd] = unpack_log(fetch_log(out, 'thetaCmd'));
[~, integ] = unpack_log(fetch_log(out, 'integ'));
[tA, alpha] = unpack_log(fetch_log(out, 'alpha'));
[tD, drift] = unpack_log(fetch_log(out, 'drift'));
[tQ, qAlpha] = unpack_log(fetch_log(out, 'qAlpha'));
alpha = align_log(t, tA, alpha);
drift = align_log(t, tD, drift);
qAlpha = align_log(t, tQ, qAlpha);

result.name = name;
result.reportLoad = false;
result.marginFrom = "";
result.t = t;
result.thetaDeg = rad2deg(theta);
result.qDeg = rad2deg(q);
result.deltaDeg = rad2deg(delta);
result.deltaCmdDeg = rad2deg(deltaCmd);
result.rateDeg = rad2deg(rate);
result.thetaCmdDeg = rad2deg(thetaCmd);
result.integ = integ(:);
result.integEnd = result.integ(end);
result.alpha = alpha(:);
result.drift = drift(:);
result.qAlpha = qAlpha(:);
result.peakQa = max(abs(result.qAlpha)) * (180 / pi) / 1000;
result.peakDrift = max(abs(result.drift));
result.finalDrift = result.drift(end);
result.tOnAngle = dwell_time(t, abs(result.deltaDeg) >= D.deltaMaxDeg - 0.05);
result.tOnRate = dwell_time(t, abs(result.rateDeg) >= D.gimbalRateDeg - 0.05);
result.shapeOn = base_or('shapeOn', 1, false);
result.thetaStepDeg = base_or('thetaStep', D.thetaStepDeg, true);
result.thetaStepTime = base_or('thetaStepTime', D.thetaStepTime, false);
result.gustSpeed = base_or('gustSpeed', D.gustSpeed, false);
result.gustTime = base_or('gustTime', D.gustTime, false);
result.misalignDeg = rad2deg(base_or('misRad', 0, false));
result.misalignTime = base_or('misalignTime', 2, false);
result.tStop = base_or('tStop', D.tStop, false);
end


function args = named_args(opts)
% Drop NaN overrides so apply_pitch_condition keeps its defaults.
fields = ["thetaStepDeg", "thetaStepTime", "gustSpeed", "gustTime", ...
    "misalignDeg", "misalignTime", "tStop", "shapeOn", "lrBlend", "driftCorrOn"];
args = {};
for k = 1:numel(fields)
    value = opts.(fields(k));
    if ~(isfloat(value) && isscalar(value) && isnan(value))
        args = [args, {fields(k), value}]; %#ok<AGROW>
    end
end
end


function y = align_log(t, tSrc, ySrc)
% Logs share the solver grid. Interpolate only if a block wrote a different time.
if numel(tSrc) == numel(t) && max(abs(tSrc(:) - t(:))) < 1e-8
    y = ySrc(:);
else
    y = interp1(tSrc(:), ySrc(:), t(:), 'linear', 'extrap');
end
end


function value = base_or(varName, fallback, asDegrees)
% fallback is already in the units the caller wants. Base-workspace angles
% that asDegrees converts are stored in radians.
if evalin('base', "exist('" + varName + "', 'var')")
    value = evalin('base', varName);
    if asDegrees
        value = rad2deg(value);
    end
else
    value = fallback;
end
end


function ts = fetch_log(out, name)
try
    ts = out.(name);
catch
    listed = "";
    try
        listed = strjoin(string(out.who), ", ");
    catch
    end
    error('run_pitch_tvc:MissingLog', ...
        'No logged signal "%s". Simulation output has: %s', name, listed);
end
end


function [t, y] = unpack_log(ts)
if isa(ts, 'timeseries')
    t = ts.Time(:);
    y = ts.Data(:);
elseif isstruct(ts) && isfield(ts, 'time')
    t = ts.time(:);
    y = ts.signals.values(:);
else
    error('run_pitch_tvc:LogFormat', ...
        'Expected a timeseries log, got %s.', class(ts));
end
end


function tout = dwell_time(t, on)
% Seconds during which `on` is true. Each solver interval uses its start sample.
if numel(t) < 2
    tout = 0;
    return
end
tout = sum(diff(t(:)) .* double(on(1:end-1)));
end


function print_checks(D, runs, simBefore, simAfter, simMis, simLR, outerMargins)
fprintf('\nChecklist (REVIEW = look at the plot, the run still finished)\n');
for k = 1:numel(runs)
    r = runs{k};
    flight = D.flight([D.flight.name] == r.name);
    err = linear_error(flight, r);
    rmsBeforeGust = err.rmsBeforeGust;
    flag = "PASS";
    if rmsBeforeGust > 0.05
        flag = "REVIEW";
    end
    fprintf('  %s  %-8s  linear vs Simulink RMS to gust   %.4f deg\n', ...
        flag, r.name, rmsBeforeGust);

    peakDelta = max(abs(r.deltaDeg));
    peakRate = max(abs(r.rateDeg));
    flagD = "PASS";
    if peakDelta > D.deltaMaxDeg + 0.05
        flagD = "REVIEW";
    end
    flagR = "PASS";
    if peakRate > 0.98 * D.gimbalRateDeg
        flagR = "REVIEW";
    end
    fprintf('  %s  %-8s  peak |gimbal| %.2f deg (limit %.0f), peak |rate| %.1f deg/s (limit %.0f)\n', ...
        flagD, r.name, peakDelta, D.deltaMaxDeg, peakRate, D.gimbalRateDeg);
    if flagR == "REVIEW"
        fprintf('  REVIEW %-8s  gimbal rate reached the limit on the %.0f deg step\n', ...
            r.name, r.thetaStepDeg);
    end
end

flagB = "PASS";
if simBefore.tOnRate < 0.2
    flagB = "REVIEW";
end
fprintf(['  %s  30 deg before  final %.2f deg, peak |gimbal| %.2f deg, ' ...
    'peak |rate| %.1f deg/s, on angle %.2f s, on rate %.2f s\n'], ...
    flagB, simBefore.thetaDeg(end), max(abs(simBefore.deltaDeg)), ...
    max(abs(simBefore.rateDeg)), simBefore.tOnAngle, simBefore.tOnRate);

flagA = "PASS";
if simAfter.tOnAngle > 0.05 || simAfter.tOnRate > 0.05 ...
        || max(abs(simAfter.deltaDeg)) > D.deltaMaxDeg - 0.5 ...
        || max(abs(simAfter.rateDeg)) > D.gimbalRateDeg - 0.5 ...
        || abs(simAfter.thetaDeg(end) - simAfter.thetaStepDeg) > 2
    flagA = "REVIEW";
end
fprintf(['  %s  30 deg after   final %.2f deg, peak |gimbal| %.2f deg, ' ...
    'peak |rate| %.1f deg/s, on angle %.2f s, on rate %.2f s\n'], ...
    flagA, simAfter.thetaDeg(end), max(abs(simAfter.deltaDeg)), ...
    max(abs(simAfter.rateDeg)), simAfter.tOnAngle, simAfter.tOnRate);

thetaEnd = simMis.thetaDeg(end);
deltaEnd = simMis.deltaDeg(end);
flag = "PASS";
if abs(deltaEnd + simMis.misalignDeg) > 0.15 || abs(thetaEnd) > 0.4
    flag = "REVIEW";
end
fprintf('  %s  misalign final pitch %.3f deg, final gimbal %.3f deg (trim %.2f deg)\n', ...
    flag, thetaEnd, deltaEnd, -simMis.misalignDeg);

for k = 1:numel(simLR)
    r = simLR{k};
    flag = "PASS";
    if r.tOnAngle > 0.05 || r.tOnRate > 0.05 ...
            || max(abs(r.deltaDeg)) > D.deltaMaxDeg - 0.5 ...
            || max(abs(r.rateDeg)) > D.gimbalRateDeg - 0.5
        flag = "REVIEW";
    end
    fprintf(['  %s  %-8s  peak |q alpha| %.1f kPa deg, peak |drift| %.0f m, ' ...
        'final drift %.0f m, peak |gimbal| %.2f deg\n'], ...
        flag, r.label, r.peakQa, r.peakDrift, r.finalDrift, max(abs(r.deltaDeg)));
end
qa = [simLR{1}.peakQa, simLR{2}.peakQa, simLR{3}.peakQa];
finalD = [simLR{1}.finalDrift, simLR{2}.finalDrift, simLR{3}.finalDrift];
flag = "PASS";
% Full weathervane builds more path error than pure tracking. q*alpha should
% still fall as the knob rises, and the correction should finish inside the peak.
if ~(qa(1) > qa(3) && abs(simLR{3}.finalDrift) < simLR{3}.peakDrift)
    flag = "REVIEW";
end
fprintf(['  %s  load relief + drift correction  q alpha %.1f -> %.1f -> %.1f kPa deg, ' ...
    'final drift %.0f -> %.0f -> %.0f m\n'], flag, qa(1), qa(2), qa(3), ...
    finalD(1), finalD(2), finalD(3));
mq = D.flight([D.flight.name] == "max-q");
flag = "PASS";
if ~(mq.margins.pm >= 30 && mq.margins.gmUp >= 6)
    flag = "REVIEW";
end
fprintf(['  %s  attitude loop with load relief active (blend is outside it) ' ...
    'PM %.1f deg, GM %.1f dB\n'], flag, mq.margins.pm, mq.margins.gmUp);
for k = 1:numel(outerMargins)
    m = outerMargins(k).margins;
    flagO = "PASS";
    if ~(m.pm >= 30 && m.gmUp >= 6 && m.stable)
        flagO = "REVIEW";
    end
    fprintf(['  %s  outer drift loop at t = %.0f s, T/m %.1f m/s^2, ' ...
        'PM %.1f deg, GM %.1f dB\n'], ...
        flagO, outerMargins(k).t, m.thrustOverMass, m.pm, m.gmUp);
end

fprintf('\nOpen the model with  open_system(''boosterPitchTVC'')\n');
fprintf('Tune a condition with  open_pitch_tuner("pid", "max-q")\n\n');
end


function rows = drift_margin_samples(D, traj)
want = [110, 140];
rows = struct('t', {}, 'phase', {}, 'margins', {});
for k = 1:numel(want)
    [~, i] = min(abs(traj.t_s - want(k)));
    m = drift_correction_margins(D, traj.thrust_N(i), traj.mass_kg(i), ...
        traj.mu_rps2(i), traj.lambda_rps2(i));
    rows(k).t = traj.t_s(i); %#ok<AGROW>
    rows(k).phase = traj.phase(i);
    rows(k).margins = m;
    fprintf(['Outer drift loop at t = %.0f s (%s), thrust/mass %.1f m/s^2: ' ...
        'PM %.1f deg, GM %.1f dB, stable %s (Padé).\n'], ...
        rows(k).t, rows(k).phase, m.thrustOverMass, m.pm, m.gmUp, string(m.stable));
end
end


function err = linear_error(flight, r)
tLin = (0:0.01:(r.tStop - r.thetaStepTime))';
y = step_delay(flight.Tcmd, tLin);
tCmd = tLin + r.thetaStepTime;
ySim = interp1(tCmd, y * r.thetaStepDeg, r.t, 'linear', 0);
mask = r.t >= r.thetaStepTime & r.t <= r.gustTime;
err.rmsBeforeGust = sqrt(mean((r.thetaDeg(mask) - ySim(mask)).^2));
err.maxAbs = max(abs(r.thetaDeg(mask) - ySim(mask)));
end
