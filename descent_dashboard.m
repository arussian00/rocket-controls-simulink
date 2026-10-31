function fig = descent_dashboard(fig, R)
%DESCENT_DASHBOARD  Add the Descent tab, and the catch numbers on Stats.
%
%   fig = descent_dashboard(fig, R)
%   fig = descent_dashboard([], R)
%
%   One window, the same Pitch TVC uifigure. The Descent tab is a 2-by-2
%   layout (four axes). The catch error, peak fin deflection and time on
%   the stops go on the Stats tab. Passing an empty figure opens the
%   window with those two tabs.

if nargin < 1 || isempty(fig) || ~isgraphics(fig)
    fig = new_window();
end
tabs = findobj(fig, 'Type', 'uitabgroup');
if isempty(tabs)
    root = uigridlayout(fig, [1 1]);
    root.Padding = [0 0 0 0];
    root.RowHeight = {'1x'};
    root.ColumnWidth = {'1x'};
    tabs = uitabgroup(root);
else
    tabs = tabs(1);
end

old = findobj(tabs, 'Type', 'uitab', 'Title', 'Descent');
if ~isempty(old)
    delete(old);
end
tab = uitab(tabs, 'Title', 'Descent');
build_descent(tab, R);
try
    add_stats(fig, R);
catch ME
    warning('descent_dashboard:Stats', ...
        'Descent numbers were not added to Stats (%s).', ME.message);
end
drawnow
end


function build_descent(parent, R)
tl = tiledlayout(parent, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
try
    tl.Units = 'normalized';
    tl.OuterPosition = [0 0 1 1];
catch
end
axPitch = next_ax(tl);
axSurf = next_ax(tl);
axHand = next_ax(tl);
axMar = next_ax(tl);

sim = R.sim;
plot(axPitch, sim.t, sim.thetaCmdDeg, 'k--', 'LineWidth', 1.0);
hold(axPitch, 'on');
plot(axPitch, sim.t, sim.shapedDeg, 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0);
plot(axPitch, sim.t, sim.thetaDeg, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.4);
mark_phases(axPitch, R.ret);
style_ax(axPitch);
ylabel(axPitch, 'Pitch [deg]');
title(axPitch, sprintf('Pitch, catch error %+.2f deg', sim.catchErrDeg));
small_legend(axPitch, {'guidance', 'governed', 'pitch'});

plot(axSurf, sim.t, sim.finDeg, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.3);
hold(axSurf, 'on');
plot(axSurf, sim.t, sim.tvcDeg, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.3);
limit_lines(axSurf, sim.t, sim.finLimitDeg, [0.85 0.33 0.10]);
limit_lines(axSurf, sim.t, sim.tvcLimitDeg, [0.00 0.45 0.74]);
mark_phases(axSurf, R.ret);
style_ax(axSurf);
ylabel(axSurf, 'Deflection [deg]');
title(axSurf, sprintf('Fins peak %.1f deg, on stop %.1f s', ...
    sim.peakFinDeg, sim.tOnFin));
small_legend(axSurf, {'grid fin', 'TVC gimbal'});

colors = [0.00 0.45 0.74
          0.85 0.33 0.10
          0.47 0.67 0.19
          0.49 0.18 0.56];
hold(axHand, 'on');
worstT = 0;
worstF = 0;
for k = 1:numel(sim.transitions)
    tr = sim.transitions(k);
    m = abs(sim.t - tr.t) <= 1;
    if ~any(m)
        continue
    end
    c = colors(1 + mod(k - 1, size(colors, 1)), :);
    tt = sim.t(m) - tr.t;
    plot(axHand, tt, sim.tvcCmdDeg(m), '-', 'Color', c, 'LineWidth', 1.3);
    plot(axHand, tt, sim.finCmdDeg(m), '--', 'Color', c, 'LineWidth', 1.1, ...
        'HandleVisibility', 'off');
    worstT = max(worstT, tr.maxDTvcDeg);
    worstF = max(worstF, tr.maxDFinDeg);
end
style_ax(axHand);
xlabel(axHand, 'Time from the handoff [s]');
ylabel(axHand, 'Command [deg]');
title(axHand, sprintf('Bumpless: largest 20 ms step TVC %.3f deg, fin %.3f deg', ...
    worstT, worstF));
names = cell(1, numel(sim.transitions));
for k = 1:numel(sim.transitions)
    names{k} = char(sim.transitions(k).name);
end
small_legend(axHand, names);

M = R.margins;
plot(axMar, M.t, M.pm, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.4);
hold(axMar, 'on');
plot(axMar, M.t, M.gm, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.3);
plot(axMar, [M.t(1) M.t(end)], [M.floorPm M.floorPm], 'b--');
plot(axMar, [M.t(1) M.t(end)], [M.floorGm M.floorGm], 'r--');
for k = 1:numel(M.anchors)
    a = M.anchors(k);
    if a.controlled && isfinite(a.pm)
        plot(axMar, a.t, a.pm, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5, ...
            'HandleVisibility', 'off');
    end
end
style_ax(axMar);
xlabel(axMar, 'Time from liftoff [s]');
ylabel(axMar, 'PM [deg],  GM [dB]');
title(axMar, sprintf('Margins, %d samples under the floor', M.nBelow));
small_legend(axMar, {'phase margin', 'gain margin', '30 deg', '6 dB'});
end


function add_stats(fig, R)
tab = findobj(fig, 'Type', 'uitab', 'Title', 'Stats');
if isempty(tab)
    tabs = findobj(fig, 'Type', 'uitabgroup');
    if isempty(tabs)
        return
    end
    tab = uitab(tabs(1), 'Title', 'Stats');
else
    tab = tab(1);
end
old = findobj(tab, 'Tag', 'descentStats');
if ~isempty(old)
    delete(old);
end
data = stats_cells(R.sim);
gl = findobj(tab, 'Type', 'uigridlayout');
if ~isempty(gl)
    gl = gl(1);
    row = [];
    ud = gl.UserData;
    if isstruct(ud) && isfield(ud, 'descentRow')
        row = ud.descentRow;
    end
    if isempty(row)
        rh = gl.RowHeight;
        if ~iscell(rh)
            rh = {rh};
        end
        rh{end + 1} = 168; %#ok<AGROW>
        gl.RowHeight = rh;
        row = numel(rh);
        if ~isstruct(ud)
            ud = struct();
        end
        ud.descentRow = row;
        gl.UserData = ud;
    end
    uit = uitable(gl, 'Data', data, ...
        'ColumnName', {'Descent metric', 'Value'}, ...
        'RowName', {}, 'ColumnSortable', true, 'FontSize', 11, ...
        'Tag', 'descentStats');
    uit.Layout.Row = row;
    uit.Layout.Column = 1;
    return
end
uit = uitable(tab, 'Data', data, ...
    'ColumnName', {'Descent metric', 'Value'}, ...
    'RowName', {}, 'FontSize', 11, 'Tag', 'descentStats');
uit.Units = 'normalized';
uit.Position = [0.05 0.05 0.9 0.9];
end


function data = stats_cells(sim)
rows = {
    'Catch attitude error [deg]', sim.catchErrDeg
    'Peak fin deflection [deg]', sim.peakFinDeg
    'Time on fin stop [s]', sim.tOnFin
    'Time on fin rate stop [s]', sim.tOnFinRate
    'Peak TVC on the return [deg]', sim.peakTvcDeg
    'Time on gimbal stop [s]', sim.tOnTvc
    };
data = cell(size(rows, 1), 2);
for k = 1:size(rows, 1)
    data{k, 1} = char(rows{k, 1});
    data{k, 2} = numeric_cell(rows{k, 2});
end
end


function value = numeric_cell(value)
if isempty(value) || ~isscalar(value) || ~isnumeric(value) || isnan(value)
    value = '';
    return
end
value = double(value);
end


function mark_phases(ax, ret)
cuts = find(diff(ret.phaseCode) ~= 0) + 1;
for k = 1:numel(cuts)
    try
        xl = xline(ax, ret.t(cuts(k)), ':', 'Color', [0.45 0.45 0.45]);
        xl.HandleVisibility = 'off';
    catch
        plot(ax, [ret.t(cuts(k)) ret.t(cuts(k))], ylim(ax), ':', ...
            'Color', [0.45 0.45 0.45], 'HandleVisibility', 'off');
    end
end
end


function fig = new_window()
fig = uifigure('Name', 'Pitch TVC', 'Visible', 'off', 'AutoResizeChildren', 'on');
try
    scr = get(groot, 'ScreenSize');
    if numel(scr) >= 4 && scr(3) > 400 && scr(4) > 300
        w = min(scr(3), max(960, 0.9 * scr(3)));
        h = min(scr(4), max(640, 0.9 * scr(4)));
        fig.Position = [scr(1) + (scr(3) - w) / 2, scr(2) + (scr(4) - h) / 2, w, h];
    end
catch
end
root = uigridlayout(fig, [1 1]);
root.Padding = [0 0 0 0];
root.RowHeight = {'1x'};
root.ColumnWidth = {'1x'};
uitabgroup(root);
fig.Visible = 'on';
try
    fig.WindowState = 'maximized';
catch
end
end


function ax = next_ax(tl)
ax = nexttile(tl);
style_ax(ax);
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


function small_legend(ax, names)
try
    lg = legend(ax, names);
    lg.Location = 'best';
    lg.FontSize = 8;
    lg.Box = 'off';
catch
end
end


function limit_lines(ax, t, lim, color)
if isempty(t)
    return
end
plot(ax, [t(1) t(end)], [lim lim], ':', 'Color', color, 'HandleVisibility', 'off');
plot(ax, [t(1) t(end)], [-lim -lim], ':', 'Color', color, 'HandleVisibility', 'off');
end
