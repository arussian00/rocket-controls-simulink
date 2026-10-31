%% RUN_PITCH_TVC  Design the pitch loop, build the Simulink model, simulate, plot.
%
%   In MATLAB Online, from this folder:
%
%       run_pitch_tvc
%
%   Needs Simulink and Control System Toolbox. Does not need the Aerospace
%   Blockset or Simulink Control Design.
%
%   Prints stability margins, builds boosterPitchTVC.slx, runs six sims
%   and opens one dashboard window. The command window still prints the
%   PASS/REVIEW checklist. REVIEW means "look at this number", not that
%   the script failed. The two 30 deg liftoff runs are the same step with
%   the command governor off, then on.

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
    simNominal{k} = run_case(D, mdl, nominal(k));
end

fprintf('Simulating liftoff, 30 deg step, command governor off ...\n');
simBefore = run_case(D, mdl, "liftoff", ...
    thetaStepDeg=V.largeStepDeg, gustSpeed=0, tStop=V.largeStepStop, shapeOn=0);
simBefore.label = "30 deg before";

fprintf('Simulating liftoff, 30 deg step, command governor on ...\n');
simAfter = run_case(D, mdl, "liftoff", ...
    thetaStepDeg=V.largeStepDeg, gustSpeed=0, tStop=V.largeStepStop, shapeOn=1);
simAfter.label = "30 deg after";

fprintf('Simulating max-q, 0.5 deg thrust misalignment ...\n');
simMis = run_case(D, mdl, "max-q", ...
    thetaStepDeg=0, gustSpeed=0, misalignDeg=0.5, misalignTime=2, tStop=40);

close all
pitchDashboard = pitch_dashboard(D, simNominal, simBefore, simAfter, simMis, schedule, traj); %#ok<NASGU>
apply_flight_schedule(traj, D.deltaMax);
print_checks(D, simNominal, simBefore, simAfter, simMis);


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
end

args = named_args(opts);
apply_pitch_condition(D, name, args{:});
out = sim(char(mdl), 'ReturnWorkspaceOutputs', 'on');

[t, theta] = unpack_log(fetch_log(out, 'theta'));
[~, q] = unpack_log(fetch_log(out, 'q'));
[~, delta] = unpack_log(fetch_log(out, 'delta'));
[~, deltaCmd] = unpack_log(fetch_log(out, 'deltaCmd'));
[~, rate] = unpack_log(fetch_log(out, 'gimbalRateMeas'));
[~, thetaCmd] = unpack_log(fetch_log(out, 'thetaCmd'));
[~, integ] = unpack_log(fetch_log(out, 'integ'));

result.name = name;
result.t = t;
result.thetaDeg = rad2deg(theta);
result.qDeg = rad2deg(q);
result.deltaDeg = rad2deg(delta);
result.deltaCmdDeg = rad2deg(deltaCmd);
result.rateDeg = rad2deg(rate);
result.thetaCmdDeg = rad2deg(thetaCmd);
result.integ = integ(:);
result.integEnd = result.integ(end);
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
    "misalignDeg", "misalignTime", "tStop", "shapeOn"];
args = {};
for k = 1:numel(fields)
    value = opts.(fields(k));
    if ~(isfloat(value) && isscalar(value) && isnan(value))
        args = [args, {fields(k), value}]; %#ok<AGROW>
    end
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


function print_checks(D, runs, simBefore, simAfter, simMis)
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
fprintf('\nOpen the model with  open_system(''boosterPitchTVC'')\n');
fprintf('Tune a condition with  open_pitch_tuner("pid", "max-q")\n\n');
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
