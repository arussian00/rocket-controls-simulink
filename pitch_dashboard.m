function fig = pitch_dashboard(D, simNominal, simBefore, simAfter, simMis, schedule, traj)
%PITCH_DASHBOARD  One window for the pitch-TVC runs.
%
%   fig = pitch_dashboard(D, simNominal, simBefore, simAfter, simMis, schedule, traj)
%
%   A uifigure whose tab group fills the window. MATLAB Online shows it
%   in the browser figure panel. Bode curves are drawn from freqresp so
%   Control System Toolbox does not open its own figure. The exact-delay
%   models are used first; freqresp falls back to rational_delay if a
%   release refuses them. Step overlays go through step_delay, same as
%   the checklist. simBefore / simAfter are the 30 deg liftoff step with
%   the command governor off, then on.

colors = [0.000 0.447 0.741
          0.850 0.325 0.098
          0.467 0.674 0.188];

fig = uifigure('Name', 'Pitch TVC', 'Visible', 'off', 'AutoResizeChildren', 'on');
place_figure(fig);
% A direct uitabgroup on the figure keeps a small pixel position and does
% not grow when the window is resized. The 1x1 grid is the layout manager.
root = uigridlayout(fig, [1 1]);
root.Padding = [0 0 0 0];
root.RowSpacing = 0;
root.ColumnSpacing = 0;
root.RowHeight = {'1x'};
root.ColumnWidth = {'1x'};
tabs = uitabgroup(root);
try
    tabs.Layout.Row = 1;
    tabs.Layout.Column = 1;
catch
    % This release will not lay the tab group out in the grid. Pin it to
    % the figure in normalized units so a resize still stretches it.
    tabs.Parent = fig;
    try
        fig.AutoResizeChildren = 'off';
    catch
    end
    tabs.Units = 'normalized';
    tabs.Position = [0 0 1 1];
end

tabResp = uitab(tabs, 'Title', 'Responses');
tabStats = uitab(tabs, 'Title', 'Stats');
tabBode = uitab(tabs, 'Title', 'Bode');
tabLarge = uitab(tabs, 'Title', '30 deg step');
tabDist = uitab(tabs, 'Title', 'Disturbances');
tabFlight = uitab(tabs, 'Title', 'Along the flight');

build_responses(tabResp, D, simNominal, colors);
build_stats(tabStats, D, simNominal, {simBefore, simAfter});
build_bode(tabBode, D, colors);
build_large(tabLarge, D, simBefore, simAfter, colors);
build_disturbances(tabDist, D, simMis, colors);
build_flight(tabFlight, D, schedule, traj);

fig.Visible = 'on';
try
    fig.WindowState = 'maximized';
catch
end
drawnow
end


function build_responses(parent, D, runs, colors)
tl = make_tiles(parent, 2, 2);
axPitch = next_ax(tl);
axGust = next_ax(tl);
axGim = next_ax(tl);
axRate = next_ax(tl);

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
names = cellstr(string({runs{1}.name, runs{2}.name, runs{3}.name}));
xlabel(axPitch, 'Time [s]');
ylabel(axPitch, 'Pitch [deg]');
title(axPitch, sprintf('%g deg step', D.thetaStepDeg));
small_legend(axPitch, hSim, names);

xlabel(axGust, 'Time after gust [s]');
ylabel(axGust, 'Pitch [deg]');
title(axGust, sprintf('%g m/s gust, linear', D.gustSpeed));
small_legend(axGust, names);

xlabel(axGim, 'Time [s]');
ylabel(axGim, 'Gimbal [deg]');
title(axGim, sprintf('Gimbal, stops at +/-%g deg', D.deltaMaxDeg));
small_legend(axGim, names);

xlabel(axRate, 'Time [s]');
ylabel(axRate, 'Gimbal rate [deg/s]');
title(axRate, sprintf('Gimbal rate, +/-%g deg/s', D.gimbalRateDeg));
small_legend(axRate, names);
end


function build_stats(parent, D, runs, extras)
g = uigridlayout(parent, [2 1]);
g.RowHeight = {100, '1x'};
g.ColumnWidth = {'1x'};
g.Padding = [12 10 12 10];
g.RowSpacing = 6;
g.Scrollable = 'off';
note = ['5 deg rows: solid is Simulink, dashed is the linear command. ' ...
    'Rise and settling are that linear response. Peaks, time on the stops ' ...
    'and the integrator are Simulink. The 30 deg rows reuse the liftoff ' ...
    'margins, so rise and settling are blank. Stable is the order-6 Padé test.'];
lbl = uilabel(g, 'Text', note, 'WordWrap', 'on', 'VerticalAlignment', 'top');
lbl.Layout.Row = 1;
lbl.Layout.Column = 1;
uit = uitable(g, 'Data', cells_for_uitable(stats_table(D, runs, extras)), ...
    'ColumnName', {'Condition', 'Stable', 'PM [deg]', 'GM up [dB]', 'GM down [dB]', ...
    'wc [rad/s]', 'Rise [s]', 'Overshoot [%]', 'Ts 5% [s]', 'Ts 2% [s]', ...
    'Peak gimbal [deg]', 'Peak rate [deg/s]', 'On angle [s]', 'On rate [s]', ...
    'Integrator [deg/s]'}, ...
    'ColumnSortable', true, 'RowName', {}, 'FontSize', 11);
uit.Layout.Row = 2;
uit.Layout.Column = 1;
end


function build_bode(parent, D, colors)
tl = make_tiles(parent, 2, 1);
axMag = next_ax(tl);
axPh = next_ax(tl);
names = cell(1, numel(D.flight));
for k = 1:numel(D.flight)
    f = D.flight(k);
    [w, magDb, phaseDeg] = loop_bode(f.L);
    c = colors(k, :);
    m = f.margins;
    names{k} = sprintf('%s   PM %.0f deg   GM %.0f dB', as_char(f.name), m.pm, m.gmUp);
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
ym = yline(axMag, 0, 'k:');
yp = yline(axPh, -180, 'k:');
try
    ym.HandleVisibility = 'off';
    yp.HandleVisibility = 'off';
catch
end
style_ax(axMag);
style_ax(axPh);
axMag.XScale = 'log';
axPh.XScale = 'log';
axMag.XLim = [0.05 200];
axPh.XLim = [0.05 200];
ylabel(axMag, 'Magnitude [dB]');
ylabel(axPh, 'Phase [deg]');
title(axMag, 'Loop gain');
title(axPh, 'Phase');
small_legend(axMag, names);
small_legend(axPh, cellstr(string({D.flight.name})));
safe_layout_xlabel(tl, 'Frequency [rad/s]', axPh);
end


function build_large(parent, D, simBefore, simAfter, colors)
tl = make_tiles(parent, 2, 2);
axPitch = next_ax(tl);
axInt = next_ax(tl);
axGim = next_ax(tl);
axRate = next_ax(tl);

cBefore = colors(2, :);
cAfter = colors(1, :);
plot(axPitch, simBefore.t, simBefore.thetaCmdDeg, 'k--', 'LineWidth', 1.1);
hold(axPitch, 'on');
plot(axPitch, simBefore.t, simBefore.thetaDeg, 'Color', cBefore, 'LineWidth', 1.2);
plot(axPitch, simAfter.t, simAfter.thetaDeg, 'Color', cAfter, 'LineWidth', 1.3);
style_ax(axPitch);
ylabel(axPitch, 'Pitch [deg]');
title(axPitch, sprintf('Pitch, ends %.1f / %.1f deg', ...
    simBefore.thetaDeg(end), simAfter.thetaDeg(end)));
small_legend(axPitch, {'command', 'before', 'after'});

plot_integ(axInt, simBefore, cBefore, simAfter, cAfter);
style_ax(axInt);
ylabel(axInt, 'Integrator [deg/s]');
title(axInt, sprintf('Integrator, %.2f / %.2f deg/s', ...
    rad2deg(simBefore.integEnd), rad2deg(simAfter.integEnd)));
small_legend(axInt, {'before', 'after'});

plot(axGim, simBefore.t, simBefore.deltaDeg, 'Color', cBefore, 'LineWidth', 1.2);
hold(axGim, 'on');
plot(axGim, simAfter.t, simAfter.deltaDeg, 'Color', cAfter, 'LineWidth', 1.3);
limit_lines(axGim, simBefore.t, D.deltaMaxDeg);
style_ax(axGim);
ylabel(axGim, 'Gimbal [deg]');
title(axGim, sprintf('Gimbal, peak %.1f / %.1f deg', ...
    max(abs(simBefore.deltaDeg)), max(abs(simAfter.deltaDeg))));
small_legend(axGim, {'before', 'after'});

plot(axRate, simBefore.t, simBefore.rateDeg, 'Color', cBefore, 'LineWidth', 1.2);
hold(axRate, 'on');
plot(axRate, simAfter.t, simAfter.rateDeg, 'Color', cAfter, 'LineWidth', 1.3);
limit_lines(axRate, simBefore.t, D.gimbalRateDeg);
style_ax(axRate);
ylabel(axRate, 'Gimbal rate [deg/s]');
title(axRate, sprintf('Rate, peak %.0f / %.0f deg/s', ...
    max(abs(simBefore.rateDeg)), max(abs(simAfter.rateDeg))));
small_legend(axRate, {'before', 'after'});
safe_layout_xlabel(tl, 'Time [s]', [axGim axRate]);
end


function build_disturbances(parent, D, simMis, colors)
tl = make_tiles(parent, 2, 2);
axMp = next_ax(tl);
axMg = next_ax(tl);
axDr = next_ax(tl, [1 2]);
flight = match_flight(D, simMis.name);
tLin = (0:0.02:(simMis.tStop - simMis.misalignTime))';
y = step_delay(flight.Tdist * flight.mu * deg2rad(simMis.misalignDeg), tLin);
c = colors(2, :);
plot(axMp, simMis.t, simMis.thetaDeg, 'Color', c, 'LineWidth', 1.3);
hold(axMp, 'on');
plot(axMp, tLin + simMis.misalignTime, rad2deg(y), '--', 'Color', c, 'LineWidth', 1.1);
style_ax(axMp);
ylabel(axMp, 'Pitch [deg]');
title(axMp, sprintf('Misalignment %.1f deg', simMis.misalignDeg));
small_legend(axMp, {'Simulink', 'linear'});

plot(axMg, simMis.t, simMis.deltaDeg, 'Color', c, 'LineWidth', 1.3);
hold(axMg, 'on');
plot(axMg, [simMis.t(1) simMis.t(end)], ...
    [-simMis.misalignDeg -simMis.misalignDeg], 'k:');
style_ax(axMg);
ylabel(axMg, 'Gimbal [deg]');
title(axMg, 'Gimbal trim');
small_legend(axMg, {'nozzle', 'trim'});

flightMq = match_flight(D, "max-q");
plot_drift(axDr, flightMq, D);
style_ax(axDr);
ylabel(axDr, 'Pitch [deg]');
title(axDr, sprintf('Open-loop drift, pole %.3f /s', flightMq.unstablePole));
small_legend(axDr, {'open loop', 'closed loop'});
safe_layout_xlabel(tl, 'Time [s]', axDr);
end


function build_flight(parent, D, S, traj)
tl = make_tiles(parent, 2, 1);
axPm = next_ax(tl);
axGm = next_ax(tl);
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
ylabel(axPm, 'Phase margin [deg]');
title(axPm, 'Phase margin');
small_legend(axPm, hPm, {'scheduled 1/mu', 'fixed liftoff gain', '30 deg floor', ...
    'design points'});

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
ylabel(axGm, 'Upward gain margin [dB]');
title(axGm, sprintf('Gain margin, %d / %d under the floor', S.nBelow, S.nBelowFixed));
small_legend(axGm, hGm, {'scheduled 1/mu', 'fixed liftoff gain', '6 dB floor', ...
    'design points'});
safe_layout_xlabel(tl, 'Time from liftoff [s]', axGm);
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


function T = stats_table(D, runs, extras)
if nargin < 3
    extras = {};
end
nFlight = numel(D.flight);
n = nFlight + numel(extras);
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
tOnAngle = nan(n, 1);
tOnRate = nan(n, 1);
integEnd = nan(n, 1);
for k = 1:nFlight
    f = D.flight(k);
    r = match_run(runs, f.name);
    [condition(k), stable(k), pm(k), gmUp(k), gmDown(k), wc(k)] = margin_cells(f);
    rise(k) = f.step5.RiseTime;
    overshoot(k) = f.step5.Overshoot;
    ts5(k) = f.step5.SettlingTime;
    ts2(k) = f.step2.SettlingTime;
    [peakGim(k), peakRate(k), tOnAngle(k), tOnRate(k), integEnd(k)] = sim_cells(r);
end
liftoff = D.flight(1);
for k = 1:numel(extras)
    r = extras{k};
    row = nFlight + k;
    [~, stable(row), pm(row), gmUp(row), gmDown(row), wc(row)] = margin_cells(liftoff);
    if isfield(r, 'label') && strlength(string(r.label)) > 0
        condition(row) = string(r.label);
    else
        condition(row) = string(r.name);
    end
    [peakGim(row), peakRate(row), tOnAngle(row), tOnRate(row), integEnd(row)] = sim_cells(r);
end
T = table(condition, stable, pm, gmUp, gmDown, wc, rise, overshoot, ts5, ts2, ...
    peakGim, peakRate, tOnAngle, tOnRate, integEnd, ...
    'VariableNames', {'Condition', 'Stable', 'PM_deg', 'GM_up_dB', 'GM_down_dB', ...
    'wc_rad_s', 'Rise_s', 'Overshoot_pct', 'Ts5_s', 'Ts2_s', ...
    'PeakGimbal_deg', 'PeakRate_deg_s', 'OnAngle_s', 'OnRate_s', 'Integ_deg_s'});
end


function [condition, stable, pm, gmUp, gmDown, wc] = margin_cells(f)
condition = string(f.name);
if f.margins.stable
    stable = "yes";
else
    stable = "no";
end
pm = f.margins.pm;
gmUp = f.margins.gmUp;
gmDown = f.margins.gmDown;
wc = f.margins.wc;
end


function [peakGim, peakRate, tOnAngle, tOnRate, integEnd] = sim_cells(r)
peakGim = max(abs(r.deltaDeg));
peakRate = max(abs(r.rateDeg));
tOnAngle = r.tOnAngle;
tOnRate = r.tOnRate;
integEnd = rad2deg(r.integEnd);
end


function cells = cells_for_uitable(T)
% uitable rejects string, categorical, and nested cells. Every entry is a
% char row or a numeric/logical scalar. Missing numbers become a blank.
raw = table2cell(T);
cells = cell(size(raw));
for k = 1:numel(raw)
    cells{k} = uitable_value(raw{k});
end
end


function value = uitable_value(value)
if isstring(value) || iscategorical(value)
    value = as_char(value);
    return
end
if iscell(value)
    if numel(value) == 1
        value = uitable_value(value{1});
    else
        value = as_char(value);
    end
    return
end
if ischar(value)
    value = reshape(value, 1, []);
    return
end
if islogical(value)
    if ~isscalar(value)
        value = as_char(value);
    end
    return
end
if isnumeric(value)
    if isempty(value) || (isscalar(value) && isnan(value))
        value = '';
        return
    end
    if ~isscalar(value)
        value = value(1);
    end
    return
end
value = as_char(value);
end


function text = as_char(value)
% Row char for titles, legends, and table text. string scalars are not char.
if isstring(value) || iscategorical(value)
    if isempty(value) || (isscalar(value) && all(ismissing(value)))
        text = '';
        return
    end
    value = char(value);
elseif iscell(value)
    parts = cell(1, numel(value));
    for k = 1:numel(value)
        parts{k} = as_char(value{k});
    end
    text = strjoin(parts, ', ');
    return
elseif isnumeric(value) || islogical(value)
    if isempty(value)
        text = '';
        return
    end
    value = char(string(value));
end
if ischar(value)
    text = reshape(value, 1, []);
    return
end
try
    text = reshape(char(string(value)), 1, []);
catch
    text = '';
end
end


function plot_integ(ax, simBefore, cBefore, simAfter, cAfter)
nB = min(numel(simBefore.t), numel(simBefore.integ));
nA = min(numel(simAfter.t), numel(simAfter.integ));
plot(ax, simBefore.t(1:nB), rad2deg(simBefore.integ(1:nB)), ...
    'Color', cBefore, 'LineWidth', 1.2);
hold(ax, 'on');
plot(ax, simAfter.t(1:nA), rad2deg(simAfter.integ(1:nA)), ...
    'Color', cAfter, 'LineWidth', 1.3);
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


function place_figure(fig)
% 90% of the screen, then maximized when this release allows it.
pos = [40 40 1180 760];
try
    scr = get(groot, 'ScreenSize');
    if numel(scr) >= 4 && scr(3) > 400 && scr(4) > 300
        w = min(scr(3), max(960, 0.9 * scr(3)));
        h = min(scr(4), max(640, 0.9 * scr(4)));
        left = scr(1) + max(0, (scr(3) - w) / 2);
        bottom = scr(2) + max(0, (scr(4) - h) / 2);
        pos = [left, bottom, w, h];
    end
catch
end
try
    fig.Position = pos;
catch
end
end


function tl = make_tiles(parent, nRows, nCols)
tl = tiledlayout(parent, nRows, nCols, 'Padding', 'compact', 'TileSpacing', 'compact');
try
    tl.Units = 'normalized';
    tl.OuterPosition = [0 0 1 1];
catch
end
end


function ax = next_ax(tl, span)
if nargin < 2
    ax = nexttile(tl);
else
    ax = nexttile(tl, span);
end
style_ax(ax);
end


function small_legend(ax, varargin)
try
    lg = legend(ax, varargin{:});
    lg.Location = 'best';
    lg.FontSize = 8;
    lg.Box = 'off';
catch
end
end


function safe_layout_xlabel(tl, label, bottomAxes)
% One xlabel for the tiledlayout. If this release has no layout xlabel,
% label the bottom axes instead.
try
    xlabel(tl, label);
catch
    if nargin >= 3
        for k = 1:numel(bottomAxes)
            xlabel(bottomAxes(k), label);
        end
    end
end
end


function style_ax(ax)
ax.Box = 'on';
ax.FontSize = 11;
grid(ax, 'on');
try
    ax.Toolbar.Visible = 'off';
catch
end
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
