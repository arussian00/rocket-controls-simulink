%% RUN_PITCH_TVC  Design the pitch loop, build the Simulink model, simulate, plot.
%
%   In MATLAB Online, from this folder:
%
%       run_pitch_tvc
%
%   Needs Simulink and Control System Toolbox. Does not need the Aerospace
%   Blockset or Simulink Control Design.
%
%   Prints stability margins, builds boosterPitchTVC.slx, runs five sims
%   and writes a PASS/REVIEW checklist. REVIEW means "look at this number",
%   not that the script failed.

clear; close all; clc

V = vehicle_params();
D = design_pitch_controller(V);
mdl = build_pitch_model();

nominal = ["liftoff", "max-q", "landing"];
simNominal = cell(1, numel(nominal));
for k = 1:numel(nominal)
    fprintf('Simulating %s ...\n', nominal(k));
    simNominal{k} = run_case(D, mdl, nominal(k));
end

fprintf('Simulating liftoff, 30 deg step (rate and angle limits) ...\n');
simLarge = run_case(D, mdl, "liftoff", ...
    thetaStepDeg=V.largeStepDeg, gustSpeed=0, tStop=V.largeStepStop);

fprintf('Simulating max-q, 0.5 deg thrust misalignment ...\n');
simMis = run_case(D, mdl, "max-q", ...
    thetaStepDeg=0, gustSpeed=0, misalignDeg=0.5, misalignTime=2, tStop=40);

plot_nominal(D, simNominal);
plot_overlay(D, simNominal);
plot_large(D, simLarge);
plot_misalign(D, simMis);
print_checks(D, simNominal, simLarge, simMis);


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

result.name = name;
result.t = t;
result.thetaDeg = rad2deg(theta);
result.qDeg = rad2deg(q);
result.deltaDeg = rad2deg(delta);
result.deltaCmdDeg = rad2deg(deltaCmd);
result.rateDeg = rad2deg(rate);
result.thetaCmdDeg = rad2deg(thetaCmd);
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
    "misalignDeg", "misalignTime", "tStop"];
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


function plot_nominal(D, runs)
figure('Name', 'Simulink pitch TVC, 5 deg');
tiledlayout(3, 1);
nexttile
hold on
for k = 1:numel(runs)
    plot(runs{k}.t, runs{k}.thetaDeg, 'LineWidth', 1.2);
end
plot(runs{1}.t, runs{1}.thetaCmdDeg, 'k--', 'LineWidth', 1.0);
grid on
ylabel('Pitch [deg]');
legend({char(runs{1}.name), char(runs{2}.name), char(runs{3}.name), 'command'}, ...
    'Location', 'southeast');
title(sprintf('%.0f deg step, %.0f m/s gust at %.0f s', ...
    D.thetaStepDeg, D.gustSpeed, D.gustTime));

nexttile
hold on
for k = 1:numel(runs)
    plot(runs{k}.t, runs{k}.deltaDeg, 'LineWidth', 1.2);
end
yline(D.deltaMaxDeg, ':');
yline(-D.deltaMaxDeg, ':');
grid on
ylabel('Gimbal [deg]');

nexttile
hold on
for k = 1:numel(runs)
    plot(runs{k}.t, runs{k}.rateDeg, 'LineWidth', 1.2);
end
yline(D.gimbalRateDeg, ':');
yline(-D.gimbalRateDeg, ':');
grid on
xlabel('Time [s]');
ylabel('Gimbal rate [deg/s]');
end


function plot_overlay(D, runs)
figure('Name', 'Linear model vs Simulink');
hold on
colors = lines(numel(runs));
for k = 1:numel(runs)
    r = runs{k};
    flight = D.flight([D.flight.name] == r.name);
    tLin = (0:0.01:(r.tStop - r.thetaStepTime))';
    y = step_delay(flight.Tcmd, tLin);
    tCmd = tLin + r.thetaStepTime;
    plot(r.t, r.thetaDeg, 'Color', colors(k, :), 'LineWidth', 1.3);
    plot(tCmd, y * r.thetaStepDeg, '--', 'Color', colors(k, :), 'LineWidth', 1.1);
end
grid on
xlabel('Time [s]');
ylabel('Pitch [deg]');
title('Solid: Simulink (sin, limits). Dashed: linear Control System Toolbox model');
legend({ ...
    [char(runs{1}.name) ' sim'], [char(runs{1}.name) ' linear'], ...
    [char(runs{2}.name) ' sim'], [char(runs{2}.name) ' linear'], ...
    [char(runs{3}.name) ' sim'], [char(runs{3}.name) ' linear']}, ...
    'Location', 'southeast');
end


function plot_large(D, r)
figure('Name', 'Liftoff 30 deg, actuator limits');
tiledlayout(2, 1);
nexttile
plot(r.t, r.thetaCmdDeg, 'k--', r.t, r.thetaDeg, 'LineWidth', 1.2);
grid on
ylabel('Pitch [deg]');
legend('command', 'pitch', 'Location', 'southeast');
title('30 deg step at liftoff. Gimbal rate and angle limits are active.');
nexttile
plot(r.t, r.deltaCmdDeg, r.t, r.deltaDeg, 'LineWidth', 1.2);
hold on
yline(D.deltaMaxDeg, ':');
yline(-D.deltaMaxDeg, ':');
grid on
xlabel('Time [s]');
ylabel('Gimbal [deg]');
legend('command', 'nozzle', 'limit', 'Location', 'southeast');
end


function plot_misalign(D, r)
figure('Name', 'Thrust misalignment trim');
flight = D.flight([D.flight.name] == r.name);
tLin = (0:0.02:(r.tStop - r.misalignTime))';
y = step_delay(flight.Tdist * flight.mu * deg2rad(r.misalignDeg), tLin);
tiledlayout(2, 1);
nexttile
plot(r.t, r.thetaDeg, 'LineWidth', 1.2);
hold on
plot(tLin + r.misalignTime, rad2deg(y), '--', 'LineWidth', 1.1);
grid on
ylabel('Pitch [deg]');
legend('Simulink', 'linear', 'Location', 'northeast');
title(sprintf('%s, %.1f deg thrust misalignment at %.0f s', ...
    r.name, r.misalignDeg, r.misalignTime));
nexttile
plot(r.t, r.deltaDeg, 'LineWidth', 1.2);
hold on
yline(-r.misalignDeg, ':');
grid on
xlabel('Time [s]');
ylabel('Gimbal [deg]');
legend('nozzle', 'trim (-misalignment)', 'Location', 'northeast');
end


function print_checks(D, runs, simLarge, simMis)
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

peakDelta = max(abs(simLarge.deltaDeg));
peakRate = max(abs(simLarge.rateDeg));
hitAngle = peakDelta > 0.98 * D.deltaMaxDeg;
hitRate = peakRate > 0.98 * D.gimbalRateDeg;
flag = "PASS";
if ~(hitAngle && hitRate)
    flag = "REVIEW";
end
fprintf('  %s  30 deg   peak |gimbal| %.2f deg, peak |rate| %.1f deg/s (both limits should be reached)\n', ...
    flag, peakDelta, peakRate);

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
