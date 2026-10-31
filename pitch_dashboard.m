function fig = pitch_dashboard(D, simNominal, simLarge, simMis, schedule, traj)
%PITCH_DASHBOARD  One window for the pitch-TVC runs.
%
%   fig = pitch_dashboard(D, simNominal, simLarge, simMis)
%
%   A uifigure with three tabs. MATLAB Online shows it in the browser
%   figure panel. Bode curves are drawn from freqresp so Control System
%   Toolbox does not open its own figure. The exact-delay models are used
%   first; freqresp falls back to rational_delay if a release refuses them.
%   Step overlays go through step_delay, same as the checklist.

colors = [0.000 0.447 0.741
          0.850 0.325 0.098
          0.467 0.674 0.188];

fig = uifigure('Name', 'Pitch TVC', ...
    'Position', [40 40 1180 760], ...
    'Visible', 'off', ...
    'AutoResizeChildren', 'on');
tabs = uitabgroup(fig);

tabResp = uitab(tabs, 'Title', 'Responses');
tabBode = uitab(tabs, 'Title', 'Bode');
tabMore = uitab(tabs, 'Title', 'Limits and disturbances');
tabFlight = uitab(tabs, 'Title', 'Along the flight');

build_responses(tabResp, D, simNominal, colors);
build_bode(tabBode, D, colors);
build_more(tabMore, D, simLarge, simMis, colors);
build_flight(tabFlight, D, schedule, traj);

fig.Visible = 'on';
drawnow
end


function build_responses(parent, D, runs, colors)
g = uigridlayout(parent, [3 2]);
g.RowHeight = {'1x', '1x', 210};
g.ColumnWidth = {'1x', '1x'};
g.Padding = [12 10 12 10];
g.RowSpacing = 8;
g.ColumnSpacing = 10;
g.Scrollable = 'on';

axPitch = new_axes(g, 1, 1);
axGust = new_axes(g, 1, 2);
axGim = new_axes(g, 2, 1);
axRate = new_axes(g, 2, 2);

hSim = gobjects(numel(runs), 1);
for k = 1:numel(runs)
    r = runs{k};
    flight = match_flight(D, r.name);
    [tLin, yLin] = linear_command(flight, r);
    c = colors(k, :);

    hSim(k) = plot(axPitch, r.t, r.thetaDeg, 'Color', c, 'LineWidth', 1.4);
    hold(axPitch, 'on');
    plot(axPitch, tLin, yLin, '--', 'Color', c, 'LineWidth', 1.1, ...
        'HandleVisibility', 'off');

    plot(axGim, r.t, r.deltaDeg, 'Color', c, 'LineWidth', 1.3);
    hold(axGim, 'on');
    plot(axRate, r.t, r.rateDeg, 'Color', c, 'LineWidth', 1.3);
    hold(axRate, 'on');

    [tG, yG] = linear_gust(flight, D.gustSpeed);
    plot(axGust, tG, yG, 'Color', c, 'LineWidth', 1.3);
    hold(axGust, 'on');
end
plot(axPitch, runs{1}.t, runs{1}.thetaCmdDeg, 'k--', 'LineWidth', 0.9, ...
    'HandleVisibility', 'off');

limit_lines(axGim, runs{1}.t, D.deltaMaxDeg);
limit_lines(axRate, runs{1}.t, D.gimbalRateDeg);

style_ax(axPitch);
style_ax(axGust);
style_ax(axGim);
style_ax(axRate);
xlabel(axPitch, 'Time [s]');
ylabel(axPitch, 'Pitch [deg]');
title(axPitch, sprintf('%.0f deg step: solid Simulink, dashed linear, black command', ...
    D.thetaStepDeg));
legend(axPitch, hSim, cellstr(string({runs{1}.name, runs{2}.name, runs{3}.name})), ...
    'Location', 'southeast');

xlabel(axGust, 'Time after gust [s]');
ylabel(axGust, 'Pitch [deg]');
title(axGust, sprintf('Linear response to a %.0f m/s gust (own scale)', D.gustSpeed));
legend(axGust, cellstr(string({runs{1}.name, runs{2}.name, runs{3}.name})), ...
    'Location', 'northeast');

xlabel(axGim, 'Time [s]');
ylabel(axGim, 'Gimbal [deg]');
title(axGim, sprintf('Gimbal angle, stops at \\pm%.0f deg', D.deltaMaxDeg));
legend(axGim, cellstr(string({runs{1}.name, runs{2}.name, runs{3}.name})), ...
    'Location', 'northeast');

xlabel(axRate, 'Time [s]');
ylabel(axRate, 'Gimbal rate [deg/s]');
title(axRate, sprintf('Gimbal rate, stops at \\pm%.0f deg/s', D.gimbalRateDeg));
legend(axRate, cellstr(string({runs{1}.name, runs{2}.name, runs{3}.name})), ...
    'Location', 'northeast');

foot = uigridlayout(g, [2 1]);
foot.Layout.Row = 3;
foot.Layout.Column = [1 2];
foot.RowHeight = {44, '1x'};
foot.Padding = [0 0 0 0];
foot.RowSpacing = 4;
note = ['Solid pitch is the 5 deg Simulink run. Dashed pitch is the linear model ' ...
    'with the exact delay. Rise, overshoot and settling in the table are that ' ...
    'linear command. Peak gimbal and rate are from Simulink. Stable is the ' ...
    'order-6 Padé test (isstable cannot see the internal delay). ' ...
    'GM up is the classical margin; GM down is how far gain can fall.'];
uilabel(foot, 'Text', note, 'WordWrap', 'on', 'VerticalAlignment', 'top');
uitable(foot, 'Data', table2cell(stats_table(D, runs)), ...
    'ColumnName', {'Condition', 'Stable', 'PM [deg]', 'GM up [dB]', 'GM down [dB]', ...
    'wc [rad/s]', 'Rise [s]', 'Overshoot [%]', 'Ts 5% [s]', 'Ts 2% [s]', ...
    'Peak gimbal [deg]', 'Peak rate [deg/s]'}, ...
    'ColumnSortable', true, 'RowName', {}, 'FontSize', 12);
end


function build_bode(parent, D, colors)
g = uigridlayout(parent, [2 1]);
g.RowHeight = {'1x', '1x'};
g.ColumnWidth = {'1x'};
g.Padding = [12 10 12 10];
g.RowSpacing = 8;
axMag = new_axes(g, 1, 1);
axPh = new_axes(g, 2, 1);
names = cell(1, numel(D.flight));
for k = 1:numel(D.flight)
    f = D.flight(k);
    [w, magDb, phaseDeg] = loop_bode(f.L);
    c = colors(k, :);
    m = f.margins;
    names{k} = sprintf('%s   PM %.0f deg   GM %.0f dB', f.name, m.pm, m.gmUp);
    semilogx(axMag, w, magDb, 'Color', c, 'LineWidth', 1.4);
    hold(axMag, 'on');
    semilogx(axPh, w, phaseDeg, 'Color', c, 'LineWidth', 1.4);
    hold(axPh, 'on');
    if isfinite(m.wc) && m.wc > 0
        magC = interp1(w, magDb, m.wc, 'linear', NaN);
        phC = interp1(w, phaseDeg, m.wc, 'linear', NaN);
        if isfinite(magC)
            plot(axMag, m.wc, magC, 'o', 'Color', c, 'MarkerFaceColor', c, ...
                'HandleVisibility', 'off');
        end
        if isfinite(phC)
            plot(axPh, m.wc, phC, 'o', 'Color', c, 'MarkerFaceColor', c, ...
                'HandleVisibility', 'off');
        end
    end
    if isfinite(m.wGmUp) && m.wGmUp > 0
        magG = interp1(w, magDb, m.wGmUp, 'linear', NaN);
        if isfinite(magG)
            plot(axMag, m.wGmUp, magG, 's', 'Color', c, 'MarkerFaceColor', c, ...
                'HandleVisibility', 'off');
        end
    end
end
yline(axMag, 0, 'k:');
yline(axPh, -180, 'k:');
style_ax(axMag);
style_ax(axPh);
axMag.XScale = 'log';
axPh.XScale = 'log';
axMag.XLim = [0.05 200];
axPh.XLim = [0.05 200];
xlabel(axPh, 'Frequency [rad/s]');
ylabel(axMag, 'Magnitude [dB]');
ylabel(axPh, 'Phase [deg]');
title(axMag, 'Loop gain. Circles: gain crossover. Squares: upward gain margin.');
title(axPh, 'Phase. The -180 deg line is the phase-crossover reference.');
legend(axMag, names, 'Location', 'southwest');
legend(axPh, cellstr(string({D.flight.name})), 'Location', 'southwest');
end


function build_more(parent, D, simLarge, simMis, colors)
g = uigridlayout(parent, [3 2]);
g.RowHeight = {'1x', '1x', '1x'};
g.ColumnWidth = {'1x', '1x'};
g.Padding = [12 10 12 10];
g.RowSpacing = 8;
g.ColumnSpacing = 10;
g.Scrollable = 'on';

axLp = new_axes(g, 1, 1);
axLg = new_axes(g, 1, 2);
axMp = new_axes(g, 2, 1);
axMg = new_axes(g, 2, 2);
axDr = new_axes(g, 3, [1 2]);

plot(axLp, simLarge.t, simLarge.thetaCmdDeg, 'k--', simLarge.t, simLarge.thetaDeg, ...
    'LineWidth', 1.2);
style_ax(axLp);
xlabel(axLp, 'Time [s]');
ylabel(axLp, 'Pitch [deg]');
title(axLp, sprintf('Liftoff %.0f deg step (limits active)', simLarge.thetaStepDeg));
legend(axLp, {'command', 'pitch'}, 'Location', 'southeast');

plot(axLg, simLarge.t, simLarge.deltaCmdDeg, simLarge.t, simLarge.deltaDeg, ...
    'LineWidth', 1.2);
hold(axLg, 'on');
limit_lines(axLg, simLarge.t, D.deltaMaxDeg);
style_ax(axLg);
xlabel(axLg, 'Time [s]');
ylabel(axLg, 'Gimbal [deg]');
title(axLg, sprintf('Peak |gimbal| %.1f deg, peak |rate| %.1f deg/s', ...
    max(abs(simLarge.deltaDeg)), max(abs(simLarge.rateDeg))));
legend(axLg, {'command', 'nozzle'}, 'Location', 'northeast');

flight = match_flight(D, simMis.name);
tLin = (0:0.02:(simMis.tStop - simMis.misalignTime))';
y = step_delay(flight.Tdist * flight.mu * deg2rad(simMis.misalignDeg), tLin);
c = colors(2, :);
plot(axMp, simMis.t, simMis.thetaDeg, 'Color', c, 'LineWidth', 1.3);
hold(axMp, 'on');
plot(axMp, tLin + simMis.misalignTime, rad2deg(y), '--', 'Color', c, 'LineWidth', 1.1);
style_ax(axMp);
xlabel(axMp, 'Time [s]');
ylabel(axMp, 'Pitch [deg]');
title(axMp, sprintf('%s, %.1f deg thrust misalignment: solid Simulink, dashed linear', ...
    simMis.name, simMis.misalignDeg));
legend(axMp, {'Simulink', 'linear'}, 'Location', 'northeast');

plot(axMg, simMis.t, simMis.deltaDeg, 'Color', c, 'LineWidth', 1.3);
hold(axMg, 'on');
plot(axMg, [simMis.t(1) simMis.t(end)], ...
    [-simMis.misalignDeg -simMis.misalignDeg], 'k:');
style_ax(axMg);
xlabel(axMg, 'Time [s]');
ylabel(axMg, 'Gimbal [deg]');
title(axMg, 'Gimbal trim. Dotted line is -misalignment.');
legend(axMg, {'nozzle', 'trim'}, 'Location', 'northeast');

flightMq = match_flight(D, "max-q");
plot_drift(axDr, flightMq, D);
style_ax(axDr);
xlabel(axDr, 'Time [s]');
ylabel(axDr, 'Pitch [deg]');
title(axDr, sprintf('Max-q open-loop drift from 1 deg (rigid body, pole %.3f rad/s)', ...
    flightMq.unstablePole));
legend(axDr, {'open loop', 'closed loop'}, 'Location', 'northeast');
end


function build_flight(parent, D, S, traj)
g = uigridlayout(parent, [2 1]);
g.RowHeight = {'1x', '1x'};
g.ColumnWidth = {'1x'};
g.Padding = [12 10 12 10];
g.RowSpacing = 8;
axPm = new_axes(g, 1, 1);
axGm = new_axes(g, 2, 1);
[tMark, pmMark, gmMark] = snapshot_marks(D, traj);

hPm(1) = plot(axPm, S.t, S.pm, 'LineWidth', 1.5);
hold(axPm, 'on');
hPm(2) = plot(axPm, S.t, S.pmFixed, 'LineWidth', 1.3);
hPm(3) = plot(axPm, [S.t(1) S.t(end)], [S.floorPm S.floorPm], 'r--');
hPm(4) = plot(axPm, tMark, pmMark, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
if any(S.belowFixed)
    plot(axPm, S.t(S.belowFixed), S.pmFixed(S.belowFixed), 'rx', ...
        'MarkerSize', 8, 'LineWidth', 1.2, 'HandleVisibility', 'off');
end
style_ax(axPm);
xlabel(axPm, 'Time from liftoff [s]');
ylabel(axPm, 'Phase margin [deg]');
title(axPm, 'Phase margin along the flight. Black dots: liftoff, max-q, landing.');
legend(axPm, hPm, {'scheduled 1/mu', 'fixed liftoff gain', '30 deg floor', ...
    'three design points'}, 'Location', 'southwest');

hGm(1) = plot(axGm, S.t, S.gm, 'LineWidth', 1.5);
hold(axGm, 'on');
hGm(2) = plot(axGm, S.t, S.gmFixed, 'LineWidth', 1.3);
hGm(3) = plot(axGm, [S.t(1) S.t(end)], [S.floorGm S.floorGm], 'r--');
hGm(4) = plot(axGm, tMark, gmMark, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
if any(S.belowFixed)
    plot(axGm, S.t(S.belowFixed), S.gmFixed(S.belowFixed), 'rx', ...
        'MarkerSize', 8, 'LineWidth', 1.2, 'HandleVisibility', 'off');
end
style_ax(axGm);
xlabel(axGm, 'Time from liftoff [s]');
ylabel(axGm, 'Upward gain margin [dB]');
title(axGm, sprintf(['Gain margin. Scheduled floor violations: %d. ' ...
    'Fixed-gain floor violations: %d (marked x).'], S.nBelow, S.nBelowFixed));
legend(axGm, hGm, {'scheduled 1/mu', 'fixed liftoff gain', '6 dB floor', ...
    'three design points'}, 'Location', 'southwest');
end


function [tMark, pmMark, gmMark] = snapshot_marks(D, traj)
qAsc = traj.q_Pa;
qAsc(traj.phase ~= "ascent") = -inf;
[~, iQ] = max(qAsc);
land = find(traj.phase == "landing");
[~, iRel] = min(abs(traj.altitude_m(land) - 1000));
iL = land(iRel);
tMark = [traj.t_s(1), traj.t_s(iQ), traj.t_s(iL)];
pmMark = [D.flight(1).margins.pm, D.flight(2).margins.pm, D.flight(3).margins.pm];
gmMark = [D.flight(1).margins.gmUp, D.flight(2).margins.gmUp, D.flight(3).margins.gmUp];
end


function T = stats_table(D, runs)
n = numel(D.flight);
condition = strings(n, 1);
stable = strings(n, 1);
pm = nan(n, 1);
gmUp = nan(n, 1);
gmDown = nan(n, 1);
wc = nan(n, 1);
rise = nan(n, 1);
overshoot = nan(n, 1);
ts5 = nan(n, 1);
ts2 = nan(n, 1);
peakGim = nan(n, 1);
peakRate = nan(n, 1);
for k = 1:n
    f = D.flight(k);
    r = match_run(runs, f.name);
    condition(k) = string(f.name);
    if f.margins.stable
        stable(k) = "yes";
    else
        stable(k) = "no";
    end
    pm(k) = f.margins.pm;
    gmUp(k) = f.margins.gmUp;
    gmDown(k) = f.margins.gmDown;
    wc(k) = f.margins.wc;
    rise(k) = f.step5.RiseTime;
    overshoot(k) = f.step5.Overshoot;
    ts5(k) = f.step5.SettlingTime;
    ts2(k) = f.step2.SettlingTime;
    peakGim(k) = max(abs(r.deltaDeg));
    peakRate(k) = max(abs(r.rateDeg));
end
T = table(condition, stable, pm, gmUp, gmDown, wc, rise, overshoot, ts5, ts2, ...
    peakGim, peakRate, ...
    'VariableNames', {'Condition', 'Stable', 'PM_deg', 'GM_up_dB', 'GM_down_dB', ...
    'wc_rad_s', 'Rise_s', 'Overshoot_pct', 'Ts5_s', 'Ts2_s', ...
    'PeakGimbal_deg', 'PeakRate_deg_s'});
end


function [w, magDb, phaseDeg] = loop_bode(L)
w = logspace(log10(0.05), log10(200), 600).';
try
    h = freqresp(L, w);
catch ME
    warning('pitch_dashboard:Bode', ...
        ['freqresp could not use the exact delay (%s). ' ...
         'Bode curves use a 6th-order Padé model.'], ME.message);
    h = freqresp(rational_delay(L, 6), w);
end
h = squeeze(h);
h = h(:);
magDb = 20*log10(abs(h));
phaseDeg = rad2deg(unwrap(angle(h)));
end


function [tCmd, yDeg] = linear_command(flight, run)
tLin = (0:0.02:(run.tStop - run.thetaStepTime))';
y = step_delay(flight.Tcmd, tLin);
tCmd = tLin + run.thetaStepTime;
yDeg = y * run.thetaStepDeg;
end


function [t, yDeg] = linear_gust(flight, gustSpeed)
t = (0:0.05:40)';
y = step_delay(flight.Tdist * flight.gustAccPerWind * gustSpeed, t);
yDeg = rad2deg(y);
end


function plot_drift(ax, flight, D)
% Same rigid-body picture as the old standalone figure. No actuator and no
% delay: those are on the Bode tab. Reading the gains, not changing them.
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
plot(ax, tOL, rad2deg(yOL), tCL, rad2deg(yCL), 'LineWidth', 1.2);
end


function ax = new_axes(parent, row, col)
ax = uiaxes(parent);
ax.Layout.Row = row;
ax.Layout.Column = col;
end


function style_ax(ax)
ax.Box = 'on';
ax.FontSize = 11;
grid(ax, 'on');
end


function limit_lines(ax, t, lim)
if isempty(t)
    return
end
plot(ax, [t(1) t(end)], [lim lim], 'k:', 'HandleVisibility', 'off');
plot(ax, [t(1) t(end)], [-lim -lim], 'k:', 'HandleVisibility', 'off');
end


function flight = match_flight(D, name)
idx = find(string({D.flight.name}) == string(name), 1);
if isempty(idx)
    error('pitch_dashboard:MissingFlight', 'No flight named "%s".', name);
end
flight = D.flight(idx);
end


function run = match_run(runs, name)
for k = 1:numel(runs)
    if string(runs{k}.name) == string(name)
        run = runs{k};
        return
    end
end
error('pitch_dashboard:MissingRun', 'No simulation named "%s".', name);
end
