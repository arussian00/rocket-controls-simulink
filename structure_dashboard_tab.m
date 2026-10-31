function structure_dashboard_tab(parent, S)
%STRUCTURE_DASHBOARD_TAB  The Structure & slosh tab: four axes, same window.
%
%   CG and arm (left) with inertia (right), mode frequencies against the
%   gain crossover, liftoff Bode with and without the notch, and the
%   margins along the flight. No extra figure.

tl = tiledlayout(parent, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
try
    tl.Units = 'normalized';
    tl.OuterPosition = [0 0 1 1];
catch
end

axMass = nexttile(tl);
style_ax(axMass);
yyaxis(axMass, 'left');
hCg = plot(axMass, S.t, S.cg, 'LineWidth', 1.3);
hold(axMass, 'on');
hArm = plot(axMass, S.t, S.arm, 'LineWidth', 1.3);
ylabel(axMass, 'From nose / gimbal arm [m]');
yyaxis(axMass, 'right');
hI = plot(axMass, S.t, S.inertia / 1e6, ...
    'Color', [0.47 0.67 0.19], 'LineWidth', 1.2);
ylabel(axMass, 'Inertia [10^6 kg m^2]');
try
    axMass.YScale = 'log';
catch
end
yyaxis(axMass, 'left');
style_ax(axMass);
title(axMass, 'CG, arm and inertia');
safe_legend(axMass, [hCg, hArm, hI], {'CG from nose', 'gimbal arm', 'inertia'});
mark_stage(axMass, S.tStage);

axF = nexttile(tl);
style_ax(axF);
hold(axF, 'on');
nSlosh = size(S.fSlosh, 2);
for k = 1:nSlosh
    plot(axF, S.t, S.fSlosh(:, k), 'LineWidth', 1.1);
end
plot(axF, S.t, S.fBend(:, 1), 'k', 'LineWidth', 1.4);
if size(S.fBend, 2) >= 2
    plot(axF, S.t, S.fBend(:, 2), 'k--', 'LineWidth', 1.1);
end
plot(axF, S.mt, S.wc / (2 * pi), 'Color', [0.4 0.4 0.4], 'LineWidth', 1.4);
style_ax(axF);
ylabel(axF, 'Frequency [Hz]');
title(axF, 'Slosh, bending and crossover');
names = S.sloshNames;
if isstring(names)
    names = cellstr(names);
end
labels = names;
labels{end + 1} = 'bending 1';
if size(S.fBend, 2) >= 2
    labels{end + 1} = 'bending 2';
end
labels{end + 1} = 'gain crossover';
small_legend(axF, labels);
mark_stage(axF, S.tStage);

axB = nexttile(tl);
style_ax(axB);
p = S.points(1);
hBare = semilogx(axB, S.bode.w, S.bode.magWithout, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
hold(axB, 'on');
hNotch = semilogx(axB, S.bode.w, S.bode.magWith, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.4);
try
    y0 = yline(axB, 0, 'k:');
    y0.HandleVisibility = 'off';
catch
end
for k = 1:numel(S.bode.wnBend)
    try
        xb = xline(axB, S.bode.wnBend(k), 'k:');
        xb.HandleVisibility = 'off';
    catch
    end
end
style_ax(axB);
axB.XScale = 'log';
axB.XLim = [0.05 200];
ylabel(axB, 'Loop gain [dB]');
xlabel(axB, 'Frequency [rad/s]');
title(axB, 'Liftoff, with and without the notch');
safe_legend(axB, [hBare, hNotch], { ...
    sprintf('without notch   PM %.0f   bGM %.0f dB', p.pmBare, p.bendGmBare(1)), ...
    sprintf('with notch   PM %.0f   bGM %.0f dB', p.pm, p.bendGm(1))});

axM = nexttile(tl);
style_ax(axM);
yyaxis(axM, 'left');
hPm = plot(axM, S.mt, S.pm, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.4);
hold(axM, 'on');
hPm0 = plot(axM, S.mt, S.pmBare, ':', 'Color', [0.85 0.33 0.10], 'LineWidth', 1.1);
plot(axM, [S.mt(1) S.mt(end)], [30 30], 'r--', 'HandleVisibility', 'off');
ylabel(axM, 'Phase margin [deg]');
yyaxis(axM, 'right');
hGm = plot(axM, S.mt, S.gm, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.3);
hold(axM, 'on');
hBg = plot(axM, S.mt, S.bendGm(:, 1), 'Color', [0.47 0.67 0.19], 'LineWidth', 1.3);
plot(axM, [S.mt(1) S.mt(end)], [6 6], 'r--', 'HandleVisibility', 'off');
ylabel(axM, 'Gain margin [dB]');
try
    ylim(axM, [-10 30]);
catch
end
yyaxis(axM, 'left');
style_ax(axM);
title(axM, 'Margins, notch on (right axis clipped at 30 dB)');
safe_legend(axM, [hPm, hPm0, hGm, hBg], ...
    {'PM with notch', 'PM without notch', 'GM', 'bending GM'});
mark_stage(axM, S.tStage);
xlabel(axM, 'Time from liftoff [s]');

try
    xlabel(tl, 'Time from liftoff [s]');
catch
end
end


function mark_stage(ax, tStage)
if isempty(tStage) || ~isfinite(tStage)
    return
end
try
    xline(ax, tStage, ':', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
catch
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


function small_legend(ax, labels)
safe_legend(ax, [], labels);
end


function safe_legend(ax, handles, labels)
try
    if isempty(handles)
        lg = legend(ax, labels);
    else
        lg = legend(handles, labels);
    end
    lg.Location = 'best';
    lg.FontSize = 8;
    lg.Box = 'off';
catch
end
end
