function MC = run_monte_carlo(opts)
%RUN_MONTE_CARLO  Dispersed pitch-TVC margins and closed-loop runs.
%
%   MC = run_monte_carlo
%   MC = run_monte_carlo(N=50, seed=42, showFigure=true)
%   MC = run_monte_carlo(N=40, showFigure=false, useSimulink=false)
%
%   Draws the table in mc_dispersions with rng(seed,'twister') (default
%   seed 42, default N 200). The controller keeps the nominal 1/mu
%   schedule. Thrust, mass, inertia, gimbal arm, aero, the servo and the
%   sensor delay hit the plant. That is the truth-versus-guidance split
%   from matlab-rocket-sim.
%
%   Every sample records:
%     peak attitude error on the 5 deg liftoff step (overshoot, and the
%       tail after 10 s)
%     peak gimbal and peak gimbal rate, and time on the angle and rate stops
%     gain and phase margins at liftoff, max-q, landing, and the flight
%       stations from mc_flight_stations
%     pass/fail against the requirements below
%
%   The 5 deg and 30 deg time histories are a 2 ms fixed-step model of the
%   Simulink diagram (governor, anti-windup, stops, delay). It reproduces
%   the published nominal rows. Simulink itself cross-checks the nominal
%   5 deg and 30 deg runs when it is licensed (nSimulink, default 2).
%   Set nSimulink higher to add the worst margin run and the worst 30 deg
%   gimbal run. parsim is used for that batch only when the Parallel
%   Computing Toolbox license is present and nSimulink >= 4; otherwise the
%   sims are serial. There is no Simulink Design Optimization.
%
%   Requirements, all of them, on every draw:
%     phase margin >= 30 deg and upward gain margin >= 6 dB at every
%       design point and flight station
%     closed loop stable on a 6th-order Padé model at liftoff, max-q and
%       landing (isstable does not accept the internal delay)
%     5 deg step, governor on: peak |gimbal| <= 15 deg, peak |rate| <= the
%       run's own rate stop, time on either stop <= 0.05 s, overshoot
%       <= 1 deg, |theta-command| <= 1 deg for t >= 10 s
%     30 deg step, governor on: time on either stop <= 0.05 s, peak
%       |gimbal| <= 14.5 deg, peak |rate| <= rate stop - 0.5 deg/s,
%       |final pitch - 30 deg| <= 2 deg
%
%   In MATLAB Online the default ensemble is about one to three minutes.
%   Each Simulink cross-check adds a few seconds. N=40 is a quick look.
%   The command window prints the pass rate, the worst cases, and how
%   many runs missed each requirement. Margins and the 5 deg step clear
%   the floor on the NumPy cross-check of this same table; the misses are
%   the 30 deg step against a slow dispersed rate stop.

arguments
    opts.N (1,1) double = 200
    opts.seed (1,1) double = 42
    opts.dt (1,1) double = 0.002
    opts.showFigure (1,1) logical = true
    opts.useSimulink (1,1) logical = true
    opts.nSimulink (1,1) double = 2
    opts.V struct = struct()
    opts.tune struct = struct()
end

if ~isfield(opts.V, 'flight')
    V = vehicle_params();
else
    V = opts.V;
end
D = design_pitch_controller(V, plot=false, verbose=false);
spec = mc_dispersions();
mc_print_dispersions(spec);
req = requirement_limits();

fprintf('Nominal plant check (every scale at 1, every offset at 0):\n');
nominalDraw = mc_sample(spec, 0);
nominalReport = nominal_check(D, V, nominalDraw, opts.dt);

fprintf(['Monte Carlo: N = %d, seed = %d, dt = %.0f ms. ' ...
    'Fast closed loop for every draw; Simulink cross-check n = %d.\n'], ...
    opts.N, opts.seed, 1e3 * opts.dt, opts.nSimulink * double(opts.useSimulink));

draws = mc_sample(spec, opts.N, opts.seed);
stations = mc_flight_stations(booster_trajectory());
nS = numel(stations);
designPts = V.flight;

MC = empty_results(opts.N, nS);
MC.seed = opts.seed;
MC.N = opts.N;
MC.req = req;
MC.floorPm = req.pmMin;
MC.floorGm = req.gmMin;
MC.tFlight = [stations.time].';
MC.stationName = string({stations.phase}).';
MC.nominal = nominalReport;
MC.spec = spec;
if isfield(opts.tune, 'tuned')
    MC.tune = opts.tune;
end

t0 = tic;
for i = 1:opts.N
    [pmRun, gmRun, wherePm, stableOk, pmRow, gmRow] = margins_one( ...
        D, V, draws(i), designPts, stations);
    % Liftoff dynamic pressure is zero, so the 15 m/s gust and the wind
    % bias do not move this 5 deg case. They still change max-q margins
    % through lambda, and the wind draw is on plant.knobs.
    sc5 = step_scenario(5, 22, 1, 0, opts.dt, true);
    plant5 = mc_realize(V, draws(i), designPts(1));
    r5 = mc_closed_loop(D, plant5, sc5);
    sc30 = step_scenario(30, D_large_stop(V), 1, 0, opts.dt, false);
    r30 = mc_closed_loop(D, plant5, sc30);

    MC.pmMin(i) = pmRun;
    MC.gmMin(i) = gmRun;
    MC.wherePm(i) = wherePm;
    MC.stable(i) = stableOk;
    MC.pmFlight(i, :) = pmRow;
    MC.gmFlight(i, :) = gmRow;
    MC.peakGimbal5(i) = r5.gimDeg;
    MC.peakRate5(i) = r5.rateDeg;
    MC.tail5(i) = r5.tailDeg;
    MC.over5(i) = r5.overshootDeg;
    MC.tOnAngle5(i) = r5.tOnAngle;
    MC.tOnRate5(i) = r5.tOnRate;
    MC.peakGimbal30(i) = r30.gimDeg;
    MC.peakRate30(i) = r30.rateDeg;
    MC.final30(i) = r30.finalDeg;
    MC.tOnAngle30(i) = r30.tOnAngle;
    MC.tOnRate30(i) = r30.tOnRate;
    MC.rateLimitDeg(i) = plant5.rateDeg;
    if i == 1
        MC.tError = r5.t(:).';
    end
    MC.errDeg(i, 1:numel(r5.errDeg)) = r5.errDeg(:).';
    MC.noiseDeg(i) = draws(i).sensorNoise;

    MC.pass(i) = requirement_pass(MC, i, req);
    if mod(i, 25) == 0 || i == opts.N
        fprintf('  %d/%d   pass so far %d\n', i, opts.N, nnz(MC.pass(1:i)));
    end
end
MC.seconds = toc(t0);
nT = numel(MC.tError);
MC.errDeg = MC.errDeg(:, 1:nT);
MC.errPlot = add_sensor_noise(MC.errDeg, MC.tError, MC.noiseDeg, D.Tf);
MC.reasons = reason_counts(MC, req);
MC.nPass = nnz(MC.pass);
MC.passRate = 100 * MC.nPass / MC.N;
MC.nPassMargins = nnz(margin_and_step5_pass(MC, req));
MC.worst = worst_case(MC, draws);
MC.simulink = simulink_cross_check(D, V, nominalDraw, draws, MC, opts);

print_summary(MC);
if opts.showFigure
    show_monte_carlo(MC);
end
end


function report = nominal_check(D, V, draw, dt)
report = struct();
report.gustSpeed = V.gustSpeed;
fprintf('%-10s %12s %12s %12s %12s\n', 'Condition', 'mu true', 'mu cmd', 'ratio', 'lambda');
for k = 1:numel(V.flight)
    plant = mc_realize(V, draw, V.flight(k));
    fprintf('%-10s %12.4f %12.4f %12.4f %12.4e\n', ...
        char(V.flight(k).name), plant.muTrue, plant.muCmd, plant.ratio, plant.lambda);
    if abs(plant.ratio - 1) > 1e-3 || abs(plant.lambda - V.flight(k).lambda) > 1e-4
        warning('run_monte_carlo:Nominal', ...
            'Nominal realization of %s does not match vehicle_params.', V.flight(k).name);
    end
end
plant = mc_realize(V, draw, V.flight(1));
r5 = mc_closed_loop(D, plant, step_scenario(5, 22, 1, 0, dt, false));
r30 = mc_closed_loop(D, plant, step_scenario(30, D_large_stop(V), 1, 0, dt, false));
mFast = mc_fast_margins(D, 1, 0, V.sensorDelay, V.actHz, V.actZeta);
L = loop_at(D, plant);
mLoop = loop_margins(L, checkStability=true);
fprintf(['Fast 5 deg:  gimbal %.2f deg (published 2.8), rate %.2f deg/s (10.7)\n' ...
    'Fast 30 deg: gimbal %.2f deg (published 6.7), rate %.2f deg/s (18.1), ' ...
    'final %.2f deg (30.6), on angle %.3f s, on rate %.3f s\n'], ...
    r5.gimDeg, r5.rateDeg, r30.gimDeg, r30.rateDeg, r30.finalDeg, r30.tOnAngle, r30.tOnRate);
fprintf(['Liftoff margins: frequency grid PM %.2f deg, GM %.2f dB; ' ...
    'loop_margins PM %.2f deg, GM %.2f dB, stable %d\n'], ...
    mFast.pm, mFast.gm, mLoop.pm, mLoop.gmUp, mLoop.stable);
report.gim5 = r5.gimDeg;
report.rate5 = r5.rateDeg;
report.gim30 = r30.gimDeg;
report.rate30 = r30.rateDeg;
report.final30 = r30.finalDeg;
report.pmFast = mFast.pm;
report.gmFast = mFast.gm;
report.pmLoop = mLoop.pm;
report.gmLoop = mLoop.gmUp;
report.stable = mLoop.stable;
end


function [pmRun, gmRun, wherePm, stableOk, pmRow, gmRow] = margins_one(D, V, draw, designPts, stations)
pmRun = inf;
gmRun = inf;
wherePm = "";
pmRow = nan(1, numel(stations));
gmRow = nan(1, numel(stations));
stableOk = true;
for k = 1:numel(designPts)
    plant = mc_realize(V, draw, designPts(k));
    m = mc_fast_margins(D, plant.ratio, plant.lambda, plant.delay, plant.actHz, plant.actZeta);
    if m.pm < pmRun
        pmRun = m.pm;
        wherePm = string(designPts(k).name) + " (design)";
    end
    gmRun = min(gmRun, m.gm);
    if ~pade_stable_plant(D, plant)
        stableOk = false;
    end
end
for k = 1:numel(stations)
    plant = mc_realize(V, draw, stations(k));
    m = mc_fast_margins(D, plant.ratio, plant.lambda, plant.delay, plant.actHz, plant.actZeta);
    pmRow(k) = m.pm;
    gmRow(k) = m.gm;
    if m.pm < pmRun
        pmRun = m.pm;
        wherePm = string(stations(k).phase) + sprintf(' t=%.0f s', stations(k).time);
    end
    gmRun = min(gmRun, m.gm);
end
end


function ok = pade_stable_plant(D, plant)
% rational_delay (Padé) before isstable. feedback() would hide the delay.
wn = 2 * pi * plant.actHz;
Act = tf(wn^2, [1, 2 * plant.actZeta * wn, wn^2]);
Rigid = tf(plant.ratio, [1, 0, -plant.lambda]);
L = D.Ctf * Act * Rigid;
L.InputDelay = plant.delay;
Tcl = feedback(rational_delay(L, 6), 1);
try
    ok = isstable(Tcl);
catch
    p = pole(Tcl);
    ok = ~isempty(p) && all(real(p) < 1e-6);
end
end


function L = loop_at(D, plant)
wn = 2 * pi * plant.actHz;
Act = tf(wn^2, [1, 2 * plant.actZeta * wn, wn^2]);
Rigid = tf(plant.ratio, [1, 0, -plant.lambda]);
L = D.Ctf * Act * Rigid;
L.InputDelay = plant.delay;
end


function sc = step_scenario(stepDeg, tStop, shapeOn, gustSpeed, dt, store)
sc.stepDeg = stepDeg;
sc.tStop = tStop;
sc.shapeOn = shapeOn;
sc.gustSpeed = gustSpeed;
sc.gustTime = 14;
sc.dt = dt;
sc.store = store;
end


function tStop = D_large_stop(V)
if isfield(V, 'largeStepStop')
    tStop = V.largeStepStop;
else
    tStop = 45;
end
end


function req = requirement_limits()
req.pmMin = 30;
req.gmMin = 6;
req.tailDeg = 1;
req.overshootDeg = 1;
req.stopTime = 0.05;
req.gimbalMaxDeg = 15;
req.gimbalClearDeg = 0.5;
req.finalDeg = 2;
req.command30 = 30;
end


function ok = requirement_pass(MC, i, req)
ok = MC.pmMin(i) >= req.pmMin && MC.gmMin(i) >= req.gmMin && MC.stable(i) ...
    && MC.peakGimbal5(i) <= req.gimbalMaxDeg ...
    && MC.peakRate5(i) <= MC.rateLimitDeg(i) ...
    && MC.tOnAngle5(i) <= req.stopTime && MC.tOnRate5(i) <= req.stopTime ...
    && MC.tail5(i) <= req.tailDeg && MC.over5(i) <= req.overshootDeg ...
    && MC.tOnAngle30(i) <= req.stopTime && MC.tOnRate30(i) <= req.stopTime ...
    && MC.peakGimbal30(i) <= req.gimbalMaxDeg - req.gimbalClearDeg ...
    && MC.peakRate30(i) <= MC.rateLimitDeg(i) - req.gimbalClearDeg ...
    && abs(MC.final30(i) - req.command30) <= req.finalDeg;
end


function mask = margin_and_step5_pass(MC, req)
mask = MC.pmMin >= req.pmMin & MC.gmMin >= req.gmMin & MC.stable ...
    & MC.peakGimbal5 <= req.gimbalMaxDeg ...
    & MC.peakRate5 <= MC.rateLimitDeg ...
    & MC.tOnAngle5 <= req.stopTime & MC.tOnRate5 <= req.stopTime ...
    & MC.tail5 <= req.tailDeg & MC.over5 <= req.overshootDeg;
end


function reasons = reason_counts(MC, req)
reasons.pm = nnz(MC.pmMin < req.pmMin);
reasons.gm = nnz(MC.gmMin < req.gmMin);
reasons.stable = nnz(~MC.stable);
reasons.gim5 = nnz(MC.peakGimbal5 > req.gimbalMaxDeg);
reasons.rate5 = nnz(MC.peakRate5 > MC.rateLimitDeg);
reasons.stop5 = nnz(MC.tOnAngle5 > req.stopTime | MC.tOnRate5 > req.stopTime);
reasons.tail5 = nnz(MC.tail5 > req.tailDeg);
reasons.over5 = nnz(MC.over5 > req.overshootDeg);
reasons.stop30 = nnz(MC.tOnAngle30 > req.stopTime | MC.tOnRate30 > req.stopTime);
reasons.gim30 = nnz(MC.peakGimbal30 > req.gimbalMaxDeg - req.gimbalClearDeg);
reasons.rate30 = nnz(MC.peakRate30 > MC.rateLimitDeg - req.gimbalClearDeg);
reasons.final30 = nnz(abs(MC.final30 - req.command30) > req.finalDeg);
end


function worst = worst_case(MC, draws)
[worst.pm, iPm] = min(MC.pmMin);
worst.gmAtPm = MC.gmMin(iPm);
worst.pmWhere = char(MC.wherePm(iPm));
worst.gm = min(MC.gmMin);
[worst.tail5, ~] = max(MC.tail5);
[worst.peakGimbal5, ~] = max(MC.peakGimbal5);
[worst.peakRate5, ~] = max(MC.peakRate5);
[worst.peakGimbal30, iG] = max(MC.peakGimbal30);
[worst.peakRate30, ~] = max(MC.peakRate30);
[worst.tOnAngle30, ~] = max(MC.tOnAngle30);
[worst.tOnRate30, iR] = max(MC.tOnRate30);
worst.final30Far = max(abs(MC.final30 - 30));
worst.rateLimitAtStop = MC.rateLimitDeg(iR);
worst.z = draws(iPm).z;
worst.actHz = max(6 * draws(iPm).actBandwidth, 1);
worst.delay = max(0.020 * draws(iPm).sensorDelay, 0);
worst.rateDeg = MC.rateLimitDeg(iPm);
worst.gimbalRun = iG;
worst.stopRun = iR;
end


function errPlot = add_sensor_noise(errDeg, t, noiseDeg, tf)
errPlot = errDeg;
if numel(t) < 2
    return
end
dt = t(2) - t(1);
if dt <= 0
    return
end
a = exp(-dt / max(tf, 1e-3));
scale = sqrt((1 + a) / max(1 - a, 1e-6));
for i = 1:size(errDeg, 1)
    sigma = noiseDeg(i);
    if ~(sigma > 0)
        continue
    end
    white = sigma * scale * randn(1, size(errDeg, 2));
    errPlot(i, :) = errDeg(i, :) + filter(1 - a, [1, -a], white);
end
end


function print_summary(MC)
fprintf('\n============== PITCH TVC MONTE CARLO (%d draws, seed %d) ==============\n', ...
    MC.N, MC.seed);
fprintf('Elapsed (ensemble, excluding Simulink): %.0f s\n', MC.seconds);
fprintf('Pass rate, all requirements:  %.1f %%   (%d / %d)\n', ...
    MC.passRate, MC.nPass, MC.N);
fprintf('Pass rate, margins and 5 deg: %.1f %%   (%d / %d)\n', ...
    100 * MC.nPassMargins / MC.N, MC.nPassMargins, MC.N);
fprintf('Worst phase margin:  %.2f deg at %s\n', MC.worst.pm, MC.worst.pmWhere);
fprintf('Worst gain margin:   %.2f dB\n', MC.worst.gm);
fprintf('Worst 5 deg tail |e|: %.3f deg    peak gimbal %.2f deg    peak rate %.2f deg/s\n', ...
    MC.worst.tail5, MC.worst.peakGimbal5, MC.worst.peakRate5);
fprintf(['Worst 30 deg: peak gimbal %.2f deg, peak rate %.2f deg/s, ' ...
    'on angle %.3f s, on rate %.3f s (that run''s rate stop %.1f deg/s)\n'], ...
    MC.worst.peakGimbal30, MC.worst.peakRate30, MC.worst.tOnAngle30, ...
    MC.worst.tOnRate30, MC.worst.rateLimitAtStop);
fprintf('Runs missing each requirement:\n');
fprintf('  margins  PM %d   GM %d   unstable %d\n', ...
    MC.reasons.pm, MC.reasons.gm, MC.reasons.stable);
fprintf('  5 deg    gimbal %d   rate %d   stops %d   tail %d   overshoot %d\n', ...
    MC.reasons.gim5, MC.reasons.rate5, MC.reasons.stop5, MC.reasons.tail5, MC.reasons.over5);
fprintf('  30 deg   stops %d   gimbal %d   rate %d   final %d\n', ...
    MC.reasons.stop30, MC.reasons.gim30, MC.reasons.rate30, MC.reasons.final30);
if MC.reasons.rate30 > 0 && MC.reasons.pm == 0
    fprintf(['The misses are the 30 deg step against a dispersed gimbal-rate stop.\n' ...
        'The governor still asks for about 18 deg/s. A servo drawn near the low\n' ...
        'end of the +/-30%% 3-sigma rate range cannot supply that without the stop.\n']);
end
if isfield(MC, 'simulink') && isfield(MC.simulink, 'compared') && MC.simulink.compared
    s = MC.simulink;
    fprintf(['Simulink nominal: 5 deg gimbal %.2f (fast %.2f), ' ...
        '30 deg gimbal %.2f (fast %.2f), 30 deg rate %.2f (fast %.2f)\n'], ...
        s.gim5, s.fastGim5, s.gim30, s.fastGim30, s.rate30, s.fastRate30);
end
fprintf('Tune without writing gains:  tune_with_systune\n');
fprintf('Open Control System Tuner:   open_pitch_cstuner\n');
fprintf('====================================================================\n\n');
end


function simOut = simulink_cross_check(D, V, nominalDraw, draws, MC, opts)
simOut.compared = false;
if ~opts.useSimulink || opts.nSimulink < 1
    return
end
if ~license('test', 'Simulink')
    fprintf('Simulink is not licensed. Skipping the diagram cross-check.\n');
    return
end
try
    mdl = 'boosterPitchTVC';
    if ~isfile(fullfile(pwd, [mdl '.slx']))
        build_pitch_model();
    elseif ~bdIsLoaded(mdl)
        load_system(mdl);
    end
    jobs = simulink_jobs(D, V, nominalDraw, draws, MC, opts.nSimulink);
    fprintf('Simulink cross-check, %d run(s)', numel(jobs));
    usePar = numel(jobs) >= 4 && exist('parsim', 'file') == 2 ...
        && license('test', 'Distrib_Computing_Toolbox');
    if usePar
        fprintf(', parsim.\n');
    else
        fprintf(', serial.\n');
    end
    outs = simulate_jobs(mdl, D, jobs, usePar);
    simOut = score_simulink(outs, jobs, MC);
    simOut.compared = true;
catch ME
    warning('run_monte_carlo:Simulink', ...
        'Simulink cross-check skipped (%s).', ME.message);
end
end


function jobs = simulink_jobs(D, V, nominalDraw, draws, MC, nSimulink)
plant0 = mc_realize(V, nominalDraw, V.flight(1));
jobs = struct('plant', plant0, 'stepDeg', 5, 'tStop', 22, 'label', "nominal 5 deg");
jobs(2).plant = plant0;
jobs(2).stepDeg = 30;
jobs(2).tStop = D_large_stop(V);
jobs(2).label = "nominal 30 deg";
if nSimulink >= 3
    [~, iPm] = min(MC.pmMin);
    jobs(3).plant = mc_realize(V, draws(iPm), V.flight(1));
    jobs(3).stepDeg = 30;
    jobs(3).tStop = D_large_stop(V);
    jobs(3).label = "worst margin, 30 deg";
end
if nSimulink >= 4
    jobs(4).plant = mc_realize(V, draws(MC.worst.gimbalRun), V.flight(1));
    jobs(4).stepDeg = 30;
    jobs(4).tStop = D_large_stop(V);
    jobs(4).label = "worst gimbal, 30 deg";
end
end


function outs = simulate_jobs(mdl, D, jobs, usePar)
n = numel(jobs);
in(n) = Simulink.SimulationInput(mdl); %#ok<NASGU>
for k = 1:n
    in(k) = simulation_input(mdl, D, jobs(k));
end
if usePar
    try
        outs = parsim(in, 'ShowProgress', 'off', 'TransferBaseWorkspaceVariables', 'off');
        return
    catch ME
        warning('run_monte_carlo:Parsim', ...
            'parsim failed (%s). Running the cross-check serially.', ME.message);
    end
end
for k = 1:n
    outs(k) = sim(in(k)); %#ok<AGROW>
end
end


function in = simulation_input(mdl, D, job)
in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('InitFcn', '');
sc = step_scenario(job.stepDeg, job.tStop, 1, 0, 0.002, false);
pairs = workspace_pairs(D, job.plant, sc);
for k = 1:2:numel(pairs)
    in = in.setVariable(pairs{k}, pairs{k + 1});
end
end


function pairs = workspace_pairs(D, plant, sc)
schedTime = [0 1e6];
pairs = { ...
    'tStop', sc.tStop, ...
    'thetaStep', deg2rad(sc.stepDeg), ...
    'thetaStepTime', 1, ...
    'cmdTau', D.cmdTau, ...
    'sensorDelay', plant.delay, ...
    'Kp', D.Kp, 'Ki', D.Ki, 'Kd', D.Kd, ...
    'Kq', D.Kq, 'Kth', D.Kth, 'KiTh', D.KiTh, 'Tf', D.Tf, ...
    'Kaw', D.kaw, 'KawA', D.kaw / max(D.Kq, 1e-9), ...
    'shapeOn', sc.shapeOn, 'shapeKs', D.shapeKs, ...
    'shapeQ', D.shapeQ, 'shapeE', D.shapeE, ...
    'qLimFloor', D.qLimFloor, 'eLimFloor', D.eLimFloor, ...
    'trimTau', D.trimTau, 'deltaAvailFloor', D.deltaAvailFloor, ...
    'actWn', 2 * pi * plant.actHz, 'actZeta', plant.actZeta, ...
    'gimbalRate', plant.rate, 'deltaMax', D.deltaMax, ...
    'mu', plant.muTrue, 'lambdaAero', plant.lambda, ...
    'aCmdMax', plant.muCmd * D.deltaMax, ...
    'schedTime', schedTime, ...
    'schedMu', plant.muTrue * [1 1], ...
    'schedLambda', plant.lambda * [1 1], ...
    'schedInvMu', (1 / plant.muCmd) * [1 1], ...
    'schedACmdMax', (plant.muCmd * D.deltaMax) * [1 1], ...
    'gustTime', sc.gustTime, 'gustSpeed', sc.gustSpeed, ...
    'gustAccPerWind', plant.gustAcc, ...
    'misalignTime', 12, 'misRad', 0, ...
    'pitchConditionName', 'monte-carlo'};
end


function simOut = score_simulink(outs, jobs, MC)
simOut.compared = false;
simOut.gim5 = NaN;
simOut.gim30 = NaN;
simOut.rate30 = NaN;
simOut.fastGim5 = MC.nominal.gim5;
simOut.fastGim30 = MC.nominal.gim30;
simOut.fastRate30 = MC.nominal.rate30;
for k = 1:numel(jobs)
    r = unpack_sim(outs(k));
    peakGim = max(abs(rad2deg(r.delta)));
    peakRate = max(abs(rad2deg(r.rate)));
    fprintf('  Simulink %-22s peak gimbal %.2f deg, peak rate %.2f deg/s\n', ...
        char(jobs(k).label), peakGim, peakRate);
    if jobs(k).stepDeg == 5 && contains(jobs(k).label, "nominal")
        simOut.gim5 = peakGim;
    elseif jobs(k).stepDeg == 30 && contains(jobs(k).label, "nominal")
        simOut.gim30 = peakGim;
        simOut.rate30 = peakRate;
    end
end
if isfinite(simOut.gim30) && abs(simOut.gim30 - simOut.fastGim30) > 0.6
    warning('run_monte_carlo:SimulinkMatch', ...
        ['Nominal 30 deg gimbal differs by %.2f deg between Simulink and the ' ...
         'fast model. The ensemble numbers are the fast model.'], ...
        simOut.gim30 - simOut.fastGim30);
end
simOut.compared = isfinite(simOut.gim5) || isfinite(simOut.gim30);
end


function r = unpack_sim(out)
[~, r.delta] = unpack_log(fetch_log(out, 'delta'));
[~, r.rate] = unpack_log(fetch_log(out, 'gimbalRateMeas'));
end


function ts = fetch_log(out, name)
ts = out.(name);
end


function [t, y] = unpack_log(ts)
if isa(ts, 'timeseries')
    t = ts.Time(:);
    y = ts.Data(:);
elseif isstruct(ts) && isfield(ts, 'time')
    t = ts.time(:);
    y = ts.signals.values(:);
else
    error('run_monte_carlo:LogFormat', 'Expected a timeseries log, got %s.', class(ts));
end
end


function MC = empty_results(N, nS)
MC.pmMin = nan(N, 1);
MC.gmMin = nan(N, 1);
MC.wherePm = strings(N, 1);
MC.stable = false(N, 1);
MC.pmFlight = nan(N, nS);
MC.gmFlight = nan(N, nS);
MC.peakGimbal5 = nan(N, 1);
MC.peakRate5 = nan(N, 1);
MC.tail5 = nan(N, 1);
MC.over5 = nan(N, 1);
MC.tOnAngle5 = nan(N, 1);
MC.tOnRate5 = nan(N, 1);
MC.peakGimbal30 = nan(N, 1);
MC.peakRate30 = nan(N, 1);
MC.final30 = nan(N, 1);
MC.tOnAngle30 = nan(N, 1);
MC.tOnRate30 = nan(N, 1);
MC.rateLimitDeg = nan(N, 1);
MC.pass = false(N, 1);
MC.errDeg = zeros(N, 1200);
MC.noiseDeg = zeros(N, 1);
MC.tError = zeros(1, 1200);
end


function show_monte_carlo(MC)
try
    figs = findall(0, 'Type', 'figure', 'Name', 'Pitch TVC');
catch
    figs = [];
end
if ~isempty(figs)
    fig = figs(1);
    tg = findall(fig, 'Type', 'uitabgroup');
    if ~isempty(tg)
        old = findall(tg(1), 'Type', 'uitab', 'Title', 'Monte Carlo');
        if ~isempty(old)
            delete(old);
        end
        tab = uitab(tg(1), 'Title', 'Monte Carlo');
        mc_dashboard_tab(tab, MC);
        append_stats_rows(fig, MC);
        return
    end
end
fig = uifigure('Name', 'Pitch TVC', 'Visible', 'off', 'AutoResizeChildren', 'on');
root = uigridlayout(fig, [1 1]);
root.Padding = [0 0 0 0];
root.RowHeight = {'1x'};
root.ColumnWidth = {'1x'};
tabs = uitabgroup(root);
tabMC = uitab(tabs, 'Title', 'Monte Carlo');
mc_dashboard_tab(tabMC, MC);
tabStats = uitab(tabs, 'Title', 'Stats');
build_standalone_stats(tabStats, MC);
fig.Visible = 'on';
try
    fig.WindowState = 'maximized';
catch
end
end


function append_stats_rows(fig, MC)
uits = findall(fig, 'Type', 'uitable');
rows = mc_summary_rows(MC);
for k = 1:numel(uits)
    data = uits(k).Data;
    names = uits(k).ColumnName;
    if ~iscell(data) || numel(names) ~= size(rows, 2)
        continue
    end
    keep = true(size(data, 1), 1);
    for r = 1:size(data, 1)
        label = data{r, 1};
        if ischar(label) && (strcmp(label, 'MC pass rate') || startsWith(label, 'MC worst'))
            keep(r) = false;
        end
    end
    uits(k).Data = [data(keep, :); rows];
end
end


function build_standalone_stats(parent, MC)
g = uigridlayout(parent, [2 1]);
g.RowHeight = {80, '1x'};
g.ColumnWidth = {'1x'};
g.Padding = [12 10 12 10];
note = sprintf(['Monte Carlo seed %d, N = %d. Pass rate %.1f%% (%d/%d). ' ...
    'Margins and the 5 deg step are separate from the 30 deg rate-stop misses. ' ...
    'Re-run pitch_dashboard with this MC struct to put the rows on the full Stats tab.'], ...
    MC.seed, MC.N, MC.passRate, MC.nPass, MC.N);
uilabel(g, 'Text', note, 'WordWrap', 'on', 'VerticalAlignment', 'top');
colnames = {'Condition', 'Stable', 'PM [deg]', 'GM up [dB]', 'GM down [dB]', ...
    'wc [rad/s]', 'Rise [s]', 'Overshoot [%]', 'Ts 5% [s]', 'Ts 2% [s]', ...
    'Peak gimbal [deg]', 'Peak rate [deg/s]', 'On angle [s]', 'On rate [s]', ...
    'Integrator [deg/s]'};
uitable(g, 'Data', mc_summary_rows(MC), 'ColumnName', colnames, ...
    'ColumnSortable', true, 'RowName', {}, 'FontSize', 11);
end
