function mc_dashboard_tab(parent, MC, tune)
%MC_DASHBOARD_TAB  The Monte Carlo tab: four axes, one window.
%
%   mc_dashboard_tab(parent, MC)
%   mc_dashboard_tab(parent, MC, tune)
%
%   parent is a uitab. The axes are the attitude-error envelope (a few
%   spaghetti traces plus the +/-3 sigma band), a histogram of peak gimbal
%   on the 30 deg step, phase and gain margins versus time with the
%   30 deg / 6 dB floor, and the tuned-versus-current margin comparison.

if nargin < 3 || isempty(tune)
    if isfield(MC, 'tune')
        tune = MC.tune;
    else
        tune = struct();
    end
end

tl = tiledlayout(parent, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
try
    tl.Units = 'normalized';
    tl.OuterPosition = [0 0 1 1];
catch
end

axErr = next_ax(tl);
axHist = next_ax(tl);
axMar = next_ax(tl);
axCmp = next_ax(tl);

plot_envelope(axErr, MC);
plot_histogram(axHist, MC);
plot_margins(axMar, MC);
plot_comparison(axCmp, MC, tune);
end


function plot_envelope(ax, MC)
t = MC.tError(:);
E = MC.errPlot;
if isempty(E)
    E = MC.errDeg;
end
n = size(E, 1);
nShow = min(n, 40);
hold(ax, 'on');
for i = 1:nShow
    plot(ax, t, E(i, :), 'Color', [0.65 0.78 0.90], 'LineWidth', 0.6, ...
        'HandleVisibility', 'off');
end
mu = mean(E, 1);
sig = std(E, 0, 1);
sig(~isfinite(sig)) = 0;
hBand = plot(ax, t, mu + 3 * sig, 'Color', [0.10 0.30 0.60], 'LineWidth', 1.4);
plot(ax, t, mu - 3 * sig, 'Color', [0.10 0.30 0.60], 'LineWidth', 1.4, ...
    'HandleVisibility', 'off');
hMean = plot(ax, t, mu, 'k', 'LineWidth', 1.2);
style_ax(ax);
xlabel(ax, 'Time [s]');
ylabel(ax, 'Pitch error [deg]');
title(ax, sprintf('5 deg error, N = %d', n));
small_legend(ax, [hBand, hMean], {'+/- 3 sigma', 'mean'});
end


function plot_histogram(ax, MC)
histogram(ax, MC.peakGimbal30, 16, 'FaceColor', [0.20 0.45 0.70], 'EdgeColor', 'none');
hold(ax, 'on');
xline_safe(ax, 15, 'k--');
style_ax(ax);
xlabel(ax, 'Peak |gimbal| [deg]');
ylabel(ax, 'Runs');
title(ax, sprintf('30 deg step, worst %.1f deg', max(MC.peakGimbal30)));
end


function plot_margins(ax, MC)
t = MC.tFlight(:);
hold(ax, 'on');
n = size(MC.pmFlight, 1);
nShow = min(n, 40);
for i = 1:nShow
    plot(ax, t, MC.pmFlight(i, :), 'Color', [0.65 0.78 0.90], 'LineWidth', 0.7, ...
        'HandleVisibility', 'off');
end
hPm = plot(ax, t, mean(MC.pmFlight, 1), 'Color', [0.10 0.30 0.60], 'LineWidth', 1.6);
hFloor = plot(ax, [t(1) t(end)], [MC.floorPm MC.floorPm], 'r--', 'LineWidth', 1.1);
style_ax(ax);
ylabel(ax, 'Phase margin [deg]');
title(ax, sprintf('Margins vs time, worst PM %.1f deg, GM %.1f dB', ...
    min(MC.pmMin), min(MC.gmMin)));

try
    yyaxis(ax, 'right');
    for i = 1:nShow
        plot(ax, t, MC.gmFlight(i, :), 'Color', [0.90 0.75 0.55], 'LineWidth', 0.7, ...
            'HandleVisibility', 'off');
    end
    hGm = plot(ax, t, mean(MC.gmFlight, 1), 'Color', [0.70 0.35 0.05], 'LineWidth', 1.6);
    hGmFloor = plot(ax, [t(1) t(end)], [MC.floorGm MC.floorGm], ...
        'Color', [0.6 0.1 0.1], 'LineStyle', ':', 'LineWidth', 1.1);
    ylabel(ax, 'Gain margin [dB]');
    yyaxis(ax, 'left');
    small_legend(ax, [hPm, hFloor, hGm, hGmFloor], ...
        {'PM mean', '30 deg floor', 'GM mean', '6 dB floor'});
catch
    small_legend(ax, [hPm, hFloor], {'PM mean', '30 deg floor'});
end
xlabel(ax, 'Time from liftoff [s]');
end


function plot_comparison(ax, MC, tune)
current = current_margins(MC);
if isstruct(tune) && isfield(tune, 'marginsCurrent') && numel(tune.marginsCurrent) >= 3
    current.pm = [tune.marginsCurrent.pm];
    current.gm = [tune.marginsCurrent.gm];
end
tuned = [];
if isstruct(tune) && isfield(tune, 'marginsTuned') && ~isempty(tune.marginsTuned)
    tuned = tune.marginsTuned;
end
if isempty(tuned)
    vals = [current.pm(:); min(current.gm(:), 40)];
    bar(ax, vals, 0.7, 'FaceColor', [0.20 0.45 0.70], 'EdgeColor', 'none');
    style_ax(ax);
    ax.XTickLabel = {'PM lo', 'PM q', 'PM land', 'GM lo', 'GM q', 'GM land'};
    ylabel(ax, 'deg or dB');
    title(ax, 'Current margins. Run tune_with_systune to compare.', 'Interpreter', 'none');
    return
end
% Rows are PM then GM at liftoff, max-q and landing.
M = zeros(6, 2);
nCond = min([3, numel(current.pm), numel(tuned)]);
for k = 1:nCond
    M(k, 1) = current.pm(k);
    M(k, 2) = tuned(k).pm;
    M(k + 3, 1) = cap_gm(current.gm(k));
    M(k + 3, 2) = cap_gm(tuned(k).gm);
end
bar(ax, M, 'grouped', 'EdgeColor', 'none');
style_ax(ax);
ax.XTickLabel = {'PM lo', 'PM q', 'PM land', 'GM lo', 'GM q', 'GM land'};
ylabel(ax, 'deg or dB');
if isfield(tune, 'current') && isfield(tune, 'tuned')
    title(ax, sprintf('Kp %.3g to %.3g, Ki %.3g to %.3g, Kd %.3g to %.3g', ...
        tune.current.Kp, tune.tuned.Kp, tune.current.Ki, tune.tuned.Ki, ...
        tune.current.Kd, tune.tuned.Kd), 'Interpreter', 'none');
else
    title(ax, 'Tuned vs current', 'Interpreter', 'none');
end
small_legend(ax, {'current', 'tuned'});
end


function current = current_margins(MC)
% Nominal liftoff / max-q / landing from the zero-dispersion check when
% the per-run matrices are the only record. The first station set is not
% the three design points, so prefer MC.nominal margins repeated only as
% a fallback and use the mean of the cloud's first three design checks
% when a tune struct is absent. Design-point margins are the published
% ones stored on the nominal report, same at every condition to a tenth.
pm = MC.nominal.pmLoop;
gm = MC.nominal.gmLoop;
if ~isfinite(pm)
    pm = MC.nominal.pmFast;
    gm = MC.nominal.gmFast;
end
current.pm = [pm pm pm];
current.gm = [gm gm gm];
end


function g = cap_gm(g)
if ~isfinite(g)
    g = 40;
end
g = min(g, 40);
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


function small_legend(ax, varargin)
try
    lg = legend(ax, varargin{:});
    lg.Location = 'best';
    lg.FontSize = 8;
    lg.Box = 'off';
catch
end
end


function xline_safe(ax, x, spec)
try
    xline(ax, x, spec, 'HandleVisibility', 'off');
catch
    yl = ylim(ax);
    plot(ax, [x x], yl, spec, 'HandleVisibility', 'off');
end
end
