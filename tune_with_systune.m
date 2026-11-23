function result = tune_with_systune(opts)
%TUNE_WITH_SYSTUNE  Tune the scheduled pitch gains. Do not write them.
%
%   result = tune_with_systune
%   result = tune_with_systune(apply=false, robustCheck=false)
%   result = tune_with_systune(apply=true)     % base workspace only
%   result = tune_with_systune(openApp=true)
%
%   Tunes Kp, Ki and Kd (one acceleration-unit set, scheduled by 1/mu)
%   at liftoff, max-q, landing, boostback and brake. Goals:
%     hard  gain margin >= 6 dB and phase margin >= 30 deg
%     soft  step tracking in about 4 s, gimbal effort, and a softer
%           10 dB / 45 deg margin so the hard floor is not the target
%   The plant at each point uses that point's mu, so the gimbal
%   (delta = a_cmd/mu) is smaller at high mu. Delay inside the tuner is
%   a 4th-order Padé. The comparison below is recomputed with loop_margins
%   on the exact delay.
%
%   systune is used when it is on the path (Control System Toolbox on some
%   releases, Robust Control Toolbox on others). Otherwise a bounded
%   fminsearch does the same trade, starting from the current gains and
%   from pidtune. apply=true copies the tuned gains into the base
%   workspace and does not edit vehicle_params.m. The next
%   vehicle_params() call restores the file values.
%
%   robustCheck=true runs run_monte_carlo on the tuned gains (default
%   N = 40, no Simulink) and prints that pass rate next to a same-size
%   run of the current gains.

arguments
    opts.apply (1,1) logical = false
    opts.robustCheck (1,1) logical = false
    opts.robustN (1,1) double = 40
    opts.seed (1,1) double = 42
    opts.openApp (1,1) logical = false
end

V = vehicle_params();
D = design_pitch_controller(V, plot=false, verbose=false);
points = tune_points(V);
nominal = mc_sample(mc_dispersions(), 0);
x0 = [D.Kp, D.Ki, D.Kd];

[xTuned, method, info] = run_tuner(D, V, points, nominal, x0);
Dcur = with_gains(D, x0(1), x0(2), x0(3));
Dtune = with_gains(D, xTuned(1), xTuned(2), xTuned(3));

result.method = method;
result.info = info;
result.current = pack_gains(Dcur);
result.tuned = pack_gains(Dtune);
result.marginsCurrent = margins_at(Dcur, V, nominal);
result.marginsTuned = margins_at(Dtune, V, nominal);
result.stepCurrent = steps_at(Dcur, V, nominal);
result.stepTuned = steps_at(Dtune, V, nominal);
result.applied = false;
result.points = string({points.name});

print_comparison(result);

if opts.apply
    apply_to_base(result.tuned);
    result.applied = true;
    fprintf(['Tuned gains are in the base workspace (Kp, Ki, Kd, Kq, Kth, KiTh).\n' ...
        'vehicle_params.m was not edited. vehicle_params() still returns the file values.\n' ...
        'Keep a hand tune with V.Kp, V.Ki, V.Kd before design_pitch_controller.\n\n']);
else
    fprintf('Gains were not written. Pass apply=true to copy them into the base workspace.\n\n');
end

if opts.robustCheck
    fprintf('Robust check, N = %d, seed = %d, Simulink off.\n', opts.robustN, opts.seed);
    MCcur = run_monte_carlo(V=V, N=opts.robustN, seed=opts.seed, ...
        showFigure=false, useSimulink=false, nSimulink=0);
    Vt = V;
    Vt.Kp = result.tuned.Kp;
    Vt.Ki = result.tuned.Ki;
    Vt.Kd = result.tuned.Kd;
    MCnew = run_monte_carlo(V=Vt, N=opts.robustN, seed=opts.seed, ...
        showFigure=false, useSimulink=false, nSimulink=0, tune=result);
    result.robustCurrent = MCcur.passRate;
    result.robustTuned = MCnew.passRate;
    fprintf('Pass rate at N = %d: current %.1f%%, tuned %.1f%%.\n\n', ...
        opts.robustN, MCcur.passRate, MCnew.passRate);
end

if opts.openApp
    open_pitch_cstuner(Kp=result.tuned.Kp, Ki=result.tuned.Ki, Kd=result.tuned.Kd);
end
end


function [xBest, method, info] = run_tuner(D, V, points, nominal, x0)
info = struct();
method = "search";
xBest = x0;
xSys = [];
haveSystune = exist('systune', 'file') == 2 && exist('realp', 'file') ~= 0;
if haveSystune
    try
        [xSys, info] = systune_gains(D, points, x0);
        if gains_meet_hard_floor(xSys, D, V, nominal)
            xBest = xSys;
            method = "systune";
            return
        end
        warning('tune_with_systune:HardFloor', ...
            ['systune returned gains that miss 6 dB or 30 deg on the exact-delay ' ...
             'check. The search can still keep them if they have the lower cost.']);
    catch ME
        warning('tune_with_systune:Systune', ...
            'systune did not finish (%s). Using the search.', ME.message);
        xSys = [];
    end
else
    fprintf(['systune is not on the path (it needs Robust Control Toolbox on many\n' ...
        'releases). Tuning with pidtune plus a bounded search on the same goals.\n']);
end
xBest = search_gains(D, V, points, nominal, x0, xSys);
if ~isempty(xSys) && norm(xBest(:) - xSys(:)) < 1e-5
    method = "systune";
else
    method = "search";
end
info.searchCost = gain_cost(xBest, D, V, points, nominal);
end


function x = search_gains(D, V, points, nominal, x0, xAlt)
cost = @(x) gain_cost(x, D, V, points, nominal);
opt = optimset('Display', 'off', 'MaxFunEvals', 80, 'MaxIter', 60);
xA = fminsearch(cost, x0, opt);
xB = xA;
if exist('pidtune', 'file') == 2
    xPid = pidtune_start(D, V);
    xB = fminsearch(cost, xPid, opt);
end
if nargin >= 6 && ~isempty(xAlt) && all(isfinite(xAlt))
    cands = [xA; xB; xAlt(:).'; x0];
else
    cands = [xA; xB; x0];
end
best = inf;
x = x0;
for i = 1:size(cands, 1)
    c = cost(cands(i, :));
    if c < best
        best = c;
        x = cands(i, :);
    end
end
x = box_gains(x);
end


function ok = gains_meet_hard_floor(x, D, V, nominal)
rows = margins_at(with_gains(D, x(1), x(2), x(3)), V, nominal);
ok = all([rows.pm] >= 30 - 0.2) && all([rows.gm] >= 6 - 0.2);
end


function [x, info] = systune_gains(D, points, x0)
% points carry mu, lambda, actHz, actZeta, delay, name.
Kp = realp('Kp', x0(1));
Kp.Minimum = 0.3;
Kp.Maximum = 4;
Ki = realp('Ki', x0(2));
Ki.Minimum = 0;
Ki.Maximum = 0.25;
Kd = realp('Kd', x0(3));
Kd.Minimum = 0.5;
Kd.Maximum = 6;

CL = tunable_closed_loop(points(1), D, Kp, Ki, Kd);
for k = 2:numel(points)
    CL(:, :, k) = tunable_closed_loop(points(k), D, Kp, Ki, Kd);
end

hard = TuningGoal.Margins('a_cmd', 6, 30);
soft = [ ...
    TuningGoal.StepTracking('theta_cmd', 'theta', 4), ...
    TuningGoal.Gain('theta_cmd', 'delta', 3), ...
    TuningGoal.Margins('a_cmd', 10, 45)];
opt = systuneOptions('Display', 'off', 'RandomStart', 0, 'MaxIter', 40);
[CLt, fSoft, gHard, infoTuned] = systune(CL, soft, hard, opt);
x = read_realp(CLt, x0);
info.fSoft = fSoft;
info.gHard = gHard;
info.tuned = infoTuned;
if any(gHard(:) > 1)
    error('tune_with_systune:Goals', ...
        'systune hard goals are not met (worst gHard = %.3g).', max(gHard(:)));
end
end


function x = read_realp(CL, fallback)
x = fallback;
try
    x = [double(getBlockValue(CL, 'Kp')), ...
         double(getBlockValue(CL, 'Ki')), ...
         double(getBlockValue(CL, 'Kd'))];
    return
catch
end
try
    x = [CL.Blocks.Kp.Value, CL.Blocks.Ki.Value, CL.Blocks.Kd.Value];
catch
    x = fallback;
end
end


function CL = tunable_closed_loop(point, D, Kp, Ki, Kd)
% 2-DOF loop, derivative on the delayed measurement, 4th-order Padé.
% Opening the loop at a_cmd is the acceleration-command loop gain.
wn = 2 * pi * point.actHz;
Act = tf(wn^2, [1, 2 * point.actZeta * wn, wn^2]);
Act.InputName = 'delta_cmd';
Act.OutputName = 'delta';
Gth = tf(1, [1, 0, -point.lambda]);
Gth.InputName = 'a_del';
Gth.OutputName = 'theta';
Mu = tf(point.mu);
Mu.InputName = 'delta';
Mu.OutputName = 'a_del';
Inv = tf(1 / point.mu);
Inv.InputName = 'a_cmd';
Inv.OutputName = 'delta_cmd';
Del = pade(tf(1, 'InputDelay', max(point.delay, 0)), 4);
Del.InputName = 'theta';
Del.OutputName = 'theta_d';
Fp = tf(1, [D.cmdTau, 1]);
Fp.InputName = 'theta_cmd';
Fp.OutputName = 'theta_pref';
eSum = sumblk('e = theta_pref - theta_d');
Cpi = Kp + Ki * tf(1, [1, 0]);
Cpi.InputName = 'e';
Cpi.OutputName = 'a_pi';
Cd = Kd * tf([1, 0], [D.Tf, 1]);
Cd.InputName = 'theta_d';
Cd.OutputName = 'a_d';
aSum = sumblk('a_raw = a_pi - a_d');
AP = AnalysisPoint('a_cmd');
AP.InputName = 'a_raw';
AP.OutputName = 'a_cmd';
CL = connect(Gth, Mu, Inv, Act, Del, Fp, eSum, Cpi, Cd, aSum, AP, ...
    {'theta_cmd'}, {'theta', 'delta'});
end


function J = gain_cost(x, D, V, points, nominal)
raw = x;
x = box_gains(x);
pen = 0;
if any(abs(raw - x) > 1e-8)
    pen = 100;
end
D2 = with_gains(D, x(1), x(2), x(3));
J = pen + 0.02 * ((x(1) - 1)^2 + ((x(2) - 0.02) / 0.02)^2 + (x(3) - 2)^2);
for k = 1:numel(points)
    plant = mc_realize(V, nominal, points(k).src);
    m = mc_fast_margins(D2, plant.ratio, plant.lambda, plant.delay, plant.actHz, plant.actZeta);
    J = J + soft_short(m.pm, 50) + soft_short(m.gm, 12);
    J = J + 80 * hard_short(m.pm, 30) + 80 * hard_short(m.gm, 6);
end
plant0 = mc_realize(V, nominal, V.flight(1));
sc.stepDeg = 5;
sc.tStop = 22;
sc.shapeOn = 1;
sc.gustSpeed = 0;
sc.gustTime = 14;
sc.dt = 0.005;
sc.store = true;
r = mc_closed_loop(D2, plant0, sc);
J = J + 4 * mean(r.errDeg .^ 2) + max(0, r.gimDeg - 4)^2 + max(0, r.overshootDeg - 0.5)^2;
end


function y = soft_short(value, target)
if ~isfinite(value)
    y = 0;
    return
end
y = max(0, target - value)^2;
end


function y = hard_short(value, target)
if ~isfinite(value)
    y = 0;
    return
end
y = max(0, target - value)^2;
end


function x = box_gains(x)
x(1) = min(max(x(1), 0.3), 4);
x(2) = min(max(x(2), 0), 0.25);
x(3) = min(max(x(3), 0.5), 6);
end


function x = pidtune_start(D, V)
x = [D.Kp, D.Ki, D.Kd];
try
    P = pitch_plant(V.flight(1), V);
    G = rational_delay(P.G, 4);
    C = pidtune(G, 'PIDF', pidtuneOptions( ...
        'PhaseMargin', 55, 'CrossoverFrequency', 2.1, ...
        'DesignFocus', 'reference-tracking'));
    x = box_gains([C.Kp, C.Ki, C.Kd]);
catch
end
end


function points = tune_points(V)
draw = mc_sample(mc_dispersions(), 0);
stations = mc_flight_stations(booster_trajectory());
src = {V.flight(1), V.flight(2), V.flight(3)};
for k = 1:numel(stations)
    if stations(k).phase == "boostback" || stations(k).phase == "brake"
        src{end + 1} = stations(k); %#ok<AGROW>
    end
end
points = repmat(struct('name', "", 'mu', 0, 'lambda', 0, ...
    'actHz', 0, 'actZeta', 0, 'delay', 0, 'src', struct()), 1, numel(src));
for k = 1:numel(src)
    plant = mc_realize(V, draw, src{k});
    points(k).name = plant.name;
    points(k).mu = plant.muCmd;
    points(k).lambda = plant.lambda;
    points(k).actHz = V.actHz;
    points(k).actZeta = V.actZeta;
    points(k).delay = V.sensorDelay;
    points(k).src = src{k};
end
end


function D2 = with_gains(D, Kp, Ki, Kd)
D2 = D;
D2.Kp = Kp;
D2.Ki = Ki;
D2.Kd = Kd;
D2.Kq = Kd;
D2.Kth = Kp / max(Kd, 1e-9);
D2.KiTh = Ki / max(Kd, 1e-9);
D2.shapeQ = D.etaAuth / max(D2.Kq, 1e-9);
D2.shapeE = D.etaAuth / max(D2.Kp, 1e-6);
D2.pid = pid(Kp, Ki, Kd, D.Tf);
D2.Ctf = tf(D2.pid);
end


function g = pack_gains(D)
g.Kp = D.Kp;
g.Ki = D.Ki;
g.Kd = D.Kd;
g.Tf = D.Tf;
g.Kq = D.Kq;
g.Kth = D.Kth;
g.KiTh = D.KiTh;
end


function rows = margins_at(D, V, nominal)
rows = repmat(struct('name', "", 'pm', NaN, 'gm', NaN, 'stable', false), 1, numel(V.flight));
for k = 1:numel(V.flight)
    plant = mc_realize(V, nominal, V.flight(k));
    wn = 2 * pi * plant.actHz;
    Act = tf(wn^2, [1, 2 * plant.actZeta * wn, wn^2]);
    Rigid = tf(plant.ratio, [1, 0, -plant.lambda]);
    L = D.Ctf * Act * Rigid;
    L.InputDelay = plant.delay;
    m = loop_margins(L);
    rows(k).name = string(V.flight(k).name);
    rows(k).pm = m.pm;
    rows(k).gm = m.gmUp;
    rows(k).stable = m.stable;
end
end


function s = steps_at(D, V, nominal)
plant = mc_realize(V, nominal, V.flight(1));
s5.stepDeg = 5; s5.tStop = 22; s5.shapeOn = 1; s5.gustSpeed = 0;
s5.gustTime = 14; s5.dt = 0.002; s5.store = false;
s30 = s5;
s30.stepDeg = 30;
s30.tStop = 45;
r5 = mc_closed_loop(D, plant, s5);
r30 = mc_closed_loop(D, plant, s30);
s.gim5 = r5.gimDeg;
s.rate5 = r5.rateDeg;
s.over5 = r5.overshootDeg;
s.tail5 = r5.tailDeg;
s.gim30 = r30.gimDeg;
s.rate30 = r30.rateDeg;
s.final30 = r30.finalDeg;
s.tOnRate30 = r30.tOnRate;
end


function print_comparison(result)
fprintf('\nScheduled-gain tune (%s). Exact-delay margins via loop_margins.\n', result.method);
fprintf('%-8s %10s %10s %10s %10s\n', '', 'Kp', 'Ki', 'Kd', 'Kq');
fprintf('%-8s %10.4g %10.4g %10.4g %10.4g\n', 'current', ...
    result.current.Kp, result.current.Ki, result.current.Kd, result.current.Kq);
fprintf('%-8s %10.4g %10.4g %10.4g %10.4g\n', 'tuned', ...
    result.tuned.Kp, result.tuned.Ki, result.tuned.Kd, result.tuned.Kq);
fprintf('\n%-10s %14s %14s %14s %14s\n', 'Condition', 'PM current', 'PM tuned', 'GM current', 'GM tuned');
for k = 1:numel(result.marginsCurrent)
    fprintf('%-10s %14.2f %14.2f %14.2f %14.2f\n', ...
        char(result.marginsCurrent(k).name), ...
        result.marginsCurrent(k).pm, result.marginsTuned(k).pm, ...
        result.marginsCurrent(k).gm, result.marginsTuned(k).gm);
end
fprintf(['Liftoff 5 deg peak gimbal %.2f -> %.2f deg, peak rate %.2f -> %.2f deg/s\n' ...
    'Liftoff 30 deg peak gimbal %.2f -> %.2f deg, peak rate %.2f -> %.2f deg/s, ' ...
    'on rate %.3f -> %.3f s\n\n'], ...
    result.stepCurrent.gim5, result.stepTuned.gim5, ...
    result.stepCurrent.rate5, result.stepTuned.rate5, ...
    result.stepCurrent.gim30, result.stepTuned.gim30, ...
    result.stepCurrent.rate30, result.stepTuned.rate30, ...
    result.stepCurrent.tOnRate30, result.stepTuned.tOnRate30);
end


function apply_to_base(g)
assignin('base', 'Kp', g.Kp);
assignin('base', 'Ki', g.Ki);
assignin('base', 'Kd', g.Kd);
assignin('base', 'Kq', g.Kq);
assignin('base', 'Kth', g.Kth);
assignin('base', 'KiTh', g.KiTh);
assignin('base', 'Tf', g.Tf);
end
