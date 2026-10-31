function structure_stats_block(parentGrid, S)
%STRUCTURE_STATS_BLOCK  Key structure numbers on the Stats tab.
%
%   A label plus one uitable. Every cell is a char row or a numeric
%   scalar, which is what uitable accepts on MATLAB Online.

wrap = uigridlayout(parentGrid, [2 1]);
wrap.Layout.Row = 3;
wrap.Layout.Column = 1;
wrap.RowHeight = {48, '1x'};
wrap.ColumnWidth = {'1x'};
wrap.Padding = [0 0 0 0];
wrap.RowSpacing = 2;
lbl = uilabel(wrap, 'Text', worst_text(S), 'WordWrap', 'on', ...
    'VerticalAlignment', 'top');
lbl.Layout.Row = 1;
lbl.Layout.Column = 1;
uit = uitable(wrap, 'Data', stat_cells(S), ...
    'ColumnName', {'Metric', 'Liftoff', 'Max-q', 'Landing'}, ...
    'ColumnSortable', true, 'RowName', {}, 'FontSize', 11);
uit.Layout.Row = 2;
uit.Layout.Column = 1;
end


function text = worst_text(S)
text = sprintf(['Slosh, bending and the scheduled notch. ' ...
    'Worst PM %.1f deg at t = %.0f s (%s). ' ...
    'Worst GM %.1f dB at t = %.0f s (%s). ' ...
    'Worst bending GM %.1f dB at t = %.0f s (%s).'], ...
    S.worstPm.pm, S.worstPm.t, char(S.worstPm.phase), ...
    S.worstGm.gm, S.worstGm.t, char(S.worstGm.phase), ...
    S.worstBend.bendGm, S.worstBend.t, char(S.worstBend.phase));
end


function cells = stat_cells(S)
p = S.points;
n = numel(p);
cg = nan(1, n);
arm = nan(1, n);
inertia = nan(1, n);
mu = nan(1, n);
sm = nan(1, n);
shipLox = nan(1, n);
boostLox = nan(1, n);
boostCh4 = nan(1, n);
f1 = nan(1, n);
f2 = nan(1, n);
pm = nan(1, n);
gm = nan(1, n);
bg = nan(1, n);
for k = 1:n
    cg(k) = p(k).cg;
    arm(k) = p(k).arm;
    inertia(k) = p(k).inertia / 1e6;
    mu(k) = p(k).mu;
    sm(k) = p(k).sm;
    shipLox(k) = p(k).fSlosh(2);
    boostCh4(k) = p(k).fSlosh(3);
    boostLox(k) = p(k).fSlosh(4);
    f1(k) = p(k).fBend(1);
    f2(k) = p(k).fBend(2);
    pm(k) = p(k).pm;
    gm(k) = p(k).gm;
    bg(k) = p(k).bendGm(1);
end
rows = {
    'CG from nose [m]', cg
    'Gimbal arm [m]', arm
    'Inertia [10^6 kg m^2]', inertia
    'mu [1/s^2]', mu
    'Static margin [cal]', sm
    'Ship LOX slosh [Hz]', shipLox
    'Booster CH4 slosh [Hz]', boostCh4
    'Booster LOX slosh [Hz]', boostLox
    'Bending 1 [Hz]', f1
    'Bending 2 [Hz]', f2
    'Phase margin [deg]', pm
    'Gain margin [dB]', gm
    'Bending GM [dB]', bg
    };
cells = cell(size(rows, 1), 1 + n);
for r = 1:size(rows, 1)
    cells{r, 1} = rows{r, 1};
    for k = 1:n
        cells{r, 1 + k} = num_or_blank(rows{r, 2}(k));
    end
end
end


function value = num_or_blank(x)
if isempty(x) || ~isscalar(x) || ~isfinite(x)
    value = '';
else
    value = double(x);
end
end
