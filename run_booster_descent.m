function R = run_booster_descent(D, V, traj)
%RUN_BOOSTER_DESCENT  Grid-fin return: margins, fixed-step loop, Simulink.
%
%   run_booster_descent
%   R = run_booster_descent(D, V, traj)
%
%   With no inputs, designs the pitch cascade and loads the Super Heavy
%   trajectory. run_pitch_tvc calls it with the design it already has, then
%   adds the Descent tab to the same Pitch TVC window.
%
%   The fixed-step loop is the one on the dashboard. The Simulink model
%   boosterDescentPitch.slx is built and run as a cross-check when Simulink
%   is available. A diagram problem does not discard the fixed-step result.

standalone = nargin == 0;
if nargin < 2 || isempty(V)
    V = vehicle_params();
end
if nargin < 1 || isempty(D)
    D = design_pitch_controller(V, plot=false, verbose=false);
end
if nargin < 3 || isempty(traj)
    traj = booster_trajectory();
end

ret = make_booster_return_trajectory(traj, V);
R = descent_controller(D, V, ret);
print_return(R);

R.simulinkOk = false;
R.simulinkRmsDeg = NaN;
try
    tq = ret.t(1) + (0:round((ret.t(end) - ret.t(1)) / 0.05)).' * 0.05;
    tq = min(tq, ret.t(end));
    S = descent_openloop(ret, tq, D);
    apply_descent_workspace(S, D, V, ret);
    mdl = build_descent_model();
    fprintf('Simulating boosterDescentPitch ...\n');
    out = sim(mdl, 'ReturnWorkspaceOutputs', 'on');
    [tSl, thSl] = unpack_log(out.dTheta);
    R.simulinkOk = true;
    R.simulinkRmsDeg = theta_rms(R.sim.t, R.sim.theta, tSl, thSl);
    flag = "PASS";
    if ~(R.simulinkRmsDeg < 2)
        flag = "REVIEW";
    end
    fprintf('  %s  Simulink vs fixed-step pitch RMS  %.3f deg\n', ...
        flag, R.simulinkRmsDeg);
catch ME
    warning('run_booster_descent:Simulink', ...
        ['boosterDescentPitch was not used (%s). The Descent tab shows ' ...
         'the fixed-step loop.'], ME.message);
end

if standalone && nargout == 0
    descent_dashboard([], R);
end
end


function print_return(R)
sim = R.sim;
fprintf('\nReturn pitch (fixed step)\n');
fprintf('  Catch attitude error     %+.2f deg\n', sim.catchErrDeg);
fprintf('  Peak |fin|               %.2f deg (stop %.0f), on the stop %.2f s, on the rate stop %.2f s\n', ...
    sim.peakFinDeg, sim.finLimitDeg, sim.tOnFin, sim.tOnFinRate);
fprintf('  Peak |TVC|               %.2f deg (stop %.0f), on the stop %.2f s\n', ...
    sim.peakTvcDeg, sim.tvcLimitDeg, sim.tOnTvc);
fprintf('  Largest one-step command %.4f deg TVC, %.4f deg fin, integrator %.3e rad/s\n', ...
    sim.maxStepTvcDeg, sim.maxStepFinDeg, sim.maxStepInteg);
fprintf('  Handoffs, largest change over one 20 ms sample inside ±1 s:\n');
for k = 1:numel(sim.transitions)
    tr = sim.transitions(k);
    fprintf('    %-20s t = %6.1f s   TVC %6.3f deg   fin %6.3f deg   integ %.3e   pitch %.3f deg\n', ...
        tr.name, tr.t, tr.maxDTvcDeg, tr.maxDFinDeg, tr.maxDInteg, tr.maxDThetaDeg);
end

flag = "PASS";
if abs(sim.catchErrDeg) > 3 || sim.peakFinDeg > sim.finLimitDeg + 0.05 ...
        || sim.peakTvcDeg > sim.tvcLimitDeg + 0.05 ...
        || sim.maxStepTvcDeg > 0.05 || sim.maxStepFinDeg > 0.07 ...
        || sim.maxStepInteg > 1e-3 || R.margins.nBelow > 0
    flag = "REVIEW";
end
for k = 1:numel(R.margins.anchors)
    a = R.margins.anchors(k);
    if a.controlled && (~a.stable || a.pm < 30 || a.gm < 6)
        flag = "REVIEW";
    end
end
fprintf('  %s  descent checklist (catch < 3 deg, stops held, no command step, margins)\n\n', flag);
end


function rmsDeg = theta_rms(tRef, thRef, tSl, thSl)
th = interp1(tSl(:), thSl(:), tRef(:), 'linear', 'extrap');
rmsDeg = sqrt(mean((rad2deg(thRef(:)) - rad2deg(th)).^2));
end


function [t, y] = unpack_log(ts)
if isa(ts, 'timeseries')
    t = ts.Time(:);
    y = ts.Data(:);
elseif isstruct(ts) && isfield(ts, 'time')
    t = ts.time(:);
    y = ts.signals.values(:);
else
    error('run_booster_descent:LogFormat', ...
        'Expected a timeseries log, got %s.', class(ts));
end
end