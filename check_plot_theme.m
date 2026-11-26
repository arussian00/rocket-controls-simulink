function check_plot_theme()
%CHECK_PLOT_THEME  Headless check that figure text is near-white.
%
%   check_plot_theme
%
%   Builds a classic figure and a small Pitch TVC-style uifigure, applies
%   the shared theme, and checks titles, labels, ticks, colorbar text,
%   annotations, tab and panel titles, labels and tables. Legends keep the
%   colors and font they had. Line colors stay put. No toolbox.

here = fileparts(mfilename('fullpath'));
addpath(here);
theme = plot_theme();

fig = figure('Visible', 'off', 'Name', 'theme check classic', 'Color', theme.bg);
closer = onCleanup(@() close_quietly(fig)); %#ok<NASGU>

tl = tiledlayout(fig, 1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
ax = nexttile(tl);
lineColor = [0.00 0.45 0.74];
hLine = plot(ax, 1:4, [1 2 1.5 2.2], 'Color', lineColor, 'LineWidth', 1.4);
hold(ax, 'on');
hBlack = plot(ax, 1:4, [0.5 0.6 0.4 0.7], 'k', 'LineWidth', 1.2);
grid(ax, 'on');
title(ax, 'Pitch');
xlabel(ax, 'Time [s]');
ylabel(ax, 'Pitch [deg]');
tx = text(ax, 2, 1.2, 'note');
cb = colorbar(ax);
cb.Label.String = 'scale';
lg = legend(ax, [hLine, hBlack], {'curve', 'mean'});
lg.Location = 'best';
lg.FontSize = 8;
lg.Box = 'on';
lg.Color = [1 1 1];
lg.TextColor = [0.06 0.06 0.06];
lg.EdgeColor = [0.75 0.75 0.75];
ann = annotation(fig, 'textbox', [0.15 0.02 0.25 0.04], ...
    'String', 'annotation', 'EdgeColor', 'none');

ax2 = nexttile(tl);
plot(ax2, 1:5, 'LineWidth', 1.2);
yyaxis(ax2, 'right');
plot(ax2, 1:5, 'LineWidth', 1.1);
ylabel(ax2, 'Right');
yyaxis(ax2, 'left');
ylabel(ax2, 'Left');
title(ax2, 'Both sides');
grid(ax2, 'on');
lgDefault = legend(ax2, 'default');
defaultSnap = legend_snap(lgDefault);

sgtitle(tl, 'Overview');
xlabel(tl, 'Shared time [s]');

apply_plot_theme(fig);
assert_classic(fig, ax, ax2, theme, hLine, lineColor, hBlack, lg, tx, cb, ann);
assert_legend_snap(lgDefault, defaultSnap);

title(ax, 'Pitch redrawn');
xlabel(ax, 'Time again [s]');
tx2 = text(ax, 3, 1.4, 'added');
apply_plot_theme(fig);
assert_classic(fig, ax, ax2, theme, hLine, lineColor, hBlack, lg, tx, cb, ann);
must_near('redrawn text', tx2.Color, theme.ink);
assert_legend_snap(lgDefault, defaultSnap);
must_near('styled legend TextColor', lg.TextColor, [0.06 0.06 0.06]);
must_near('styled legend Color', lg.Color, [1 1 1]);
must_near('styled legend EdgeColor', lg.EdgeColor, [0.75 0.75 0.75]);
if lg.FontSize ~= 8
    fail('Styled legend font size changed from 8 to %.3g.', lg.FontSize);
end

ui = uifigure('Name', 'theme check ui', 'Visible', 'off', ...
    'AutoResizeChildren', 'on', 'Color', theme.bg);
closerUi = onCleanup(@() close_quietly(ui)); %#ok<NASGU>
root = uigridlayout(ui, [1 1]);
root.Padding = [0 0 0 0];
root.RowHeight = {'1x'};
root.ColumnWidth = {'1x'};
tabs = uitabgroup(root);
tab = uitab(tabs, 'Title', 'Responses');
inner = uigridlayout(tab, [4 1]);
inner.RowHeight = {24, 90, 36, '1x'};
inner.ColumnWidth = {'1x'};
lbl = uilabel(inner, 'Text', 'Stats note', 'WordWrap', 'on');
uit = uitable(inner, 'Data', {'Catch error', 0.25}, ...
    'ColumnName', {'Metric', 'Value'}, 'RowName', {}, 'FontSize', 11);
panel = uipanel(inner, 'Title', 'Block');
tlUi = tiledlayout(inner, 1, 1, 'Padding', 'compact');
axUi = nexttile(tlUi);
plot(axUi, 1:3, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
title(axUi, 'Tile');
xlabel(axUi, 't');
ylabel(axUi, 'y');
sgtitle(tlUi, 'Panel title');
apply_plot_theme(ui);

must_near('uifigure', ui.Color, theme.bg);
must_near('uitab label', tab.ForegroundColor, theme.ink);
must_dark('uitab background', tab.BackgroundColor, theme);
must_near('uipanel title', panel.ForegroundColor, theme.ink);
must_dark('uipanel background', panel.BackgroundColor, theme);
must_near('uilabel', lbl.FontColor, theme.ink);
must_near('uitable text', uit.ForegroundColor, theme.ink);
must_dark('uitable background', uit.BackgroundColor, theme);
must_near('ui title', axUi.Title.Color, theme.ink);
must_near('ui xlabel', axUi.XLabel.Color, theme.ink);
must_near('ui XColor', axUi.XColor, theme.ink);
assert_light_text(ui, theme);

fprintf('check_plot_theme passed.\n');
end


function assert_classic(fig, ax, ax2, theme, hLine, lineColor, hBlack, lg, tx, cb, ann)
must_near('figure', fig.Color, theme.bg);
must_near('axes', ax.Color, theme.bg);
must_near('XColor', ax.XColor, theme.ink);
must_near('YColor', ax.YColor, theme.ink);
must_near('ZColor', ax.ZColor, theme.ink);
must_near('GridColor', ax.GridColor, theme.grid);
if max(abs(ax.GridColor - theme.ink)) < 0.2
    fail('Grid color is too close to the text color.');
end
must_near('title', ax.Title.Color, theme.ink);
must_near('xlabel', ax.XLabel.Color, theme.ink);
must_near('ylabel', ax.YLabel.Color, theme.ink);
must_near('text', tx.Color, theme.ink);
must_near('annotation', ann.Color, theme.ink);
must_near('colorbar', cb.Color, theme.ink);
must_near('colorbar label', cb.Label.Color, theme.ink);
must_near('line', hLine.Color, lineColor);
must_near('black line', hBlack.Color, [0 0 0]);
must_near('styled legend TextColor', lg.TextColor, [0.06 0.06 0.06]);
must_near('styled legend Color', lg.Color, [1 1 1]);
must_near('styled legend EdgeColor', lg.EdgeColor, [0.75 0.75 0.75]);
assert_rulers(ax, theme);
assert_rulers(ax2, theme);
must_near('right ylabel', ax2.YAxis(end).Label.Color, theme.ink);
assert_light_text(fig, theme);
assert_legend_entries(lg, [0.06 0.06 0.06]);
end


function assert_rulers(ax, theme)
for name = ["XAxis", "YAxis", "ZAxis"]
    rulers = ax.(name);
    for r = 1:numel(rulers)
        must_near(char(name), rulers(r).Color, theme.ink);
        if isprop(rulers(r), 'TickLabelColor')
            must_near([char(name) ' ticks'], rulers(r).TickLabelColor, theme.ink);
        end
    end
end
end


function assert_light_text(fig, theme)
objs = findall(fig);
nText = 0;
for k = 1:numel(objs)
    obj = objs(k);
    if in_legend_check(obj)
        continue
    end
    kind = get(obj, 'Type');
    switch kind
        case {'text', 'textbox', 'textboxshape', 'subplottext'}
            nText = nText + 1;
            must_near([kind ' object'], obj.Color, theme.ink);
        case {'axes', 'polaraxes'}
            must_near('axes XColor', obj.XColor, theme.ink);
            must_near('axes YColor', obj.YColor, theme.ink);
            must_near('axes ZColor', obj.ZColor, theme.ink);
            must_dark('axes background', obj.Color, theme);
        case 'colorbar'
            must_near('colorbar ticks', obj.Color, theme.ink);
            must_near('colorbar label', obj.Label.Color, theme.ink);
        case 'figure'
            must_dark('figure background', obj.Color, theme);
        case 'uilabel'
            must_near('uilabel', obj.FontColor, theme.ink);
        case 'uitable'
            must_near('uitable', obj.ForegroundColor, theme.ink);
        case 'uitab'
            must_near('uitab', obj.ForegroundColor, theme.ink);
            must_dark('uitab background', obj.BackgroundColor, theme);
        case 'uipanel'
            must_near('uipanel', obj.ForegroundColor, theme.ink);
            must_dark('uipanel background', obj.BackgroundColor, theme);
    end
end
if nText < 1
    fail('No text objects were found to check.');
end
end


function assert_legend_entries(lg, ink)
kids = findall(lg, 'Type', 'text');
n = 0;
for k = 1:numel(kids)
    label = string(kids(k).String);
    if strlength(strtrim(label)) == 0
        continue
    end
    n = n + 1;
    must_near('legend entry', kids(k).Color, ink);
end
if n < 1
    fail('Styled legend has no text entries.');
end
end


function snap = legend_snap(lg)
snap = struct();
snap.TextColor = lg.TextColor;
snap.Color = lg.Color;
snap.EdgeColor = lg.EdgeColor;
snap.FontSize = lg.FontSize;
snap.FontName = lg.FontName;
snap.FontWeight = lg.FontWeight;
if isprop(lg, 'FontAngle')
    snap.FontAngle = lg.FontAngle;
end
end


function assert_legend_snap(lg, snap)
must_near('default legend TextColor', lg.TextColor, snap.TextColor);
must_near('default legend Color', lg.Color, snap.Color);
must_near('default legend EdgeColor', lg.EdgeColor, snap.EdgeColor);
if lg.FontSize ~= snap.FontSize
    fail('Default legend FontSize changed.');
end
if ~strcmp(char(string(lg.FontName)), char(string(snap.FontName)))
    fail('Default legend FontName changed.');
end
if ~strcmp(char(string(lg.FontWeight)), char(string(snap.FontWeight)))
    fail('Default legend FontWeight changed.');
end
if isfield(snap, 'FontAngle') && isprop(lg, 'FontAngle')
    if ~strcmp(char(string(lg.FontAngle)), char(string(snap.FontAngle)))
        fail('Default legend FontAngle changed.');
    end
end
end


function tf = in_legend_check(obj)
tf = false;
h = obj;
for step = 1:16 %#ok<NASGU>
    if isempty(h) || ~isgraphics(h)
        return
    end
    try
        if strcmp(get(h, 'Type'), 'legend')
            tf = true;
            return
        end
        h = get(h, 'Parent');
    catch
        return
    end
end
end


function must_near(label, actual, expected)
rgb = as_rgb(actual, label);
want = as_rgb(expected, label);
if max(abs(rgb - want)) > 1/255
    fail('%s is [%0.3f %0.3f %0.3f], expected [%0.3f %0.3f %0.3f].', ...
        label, rgb(1), rgb(2), rgb(3), want(1), want(2), want(3));
end
end


function must_dark(label, actual, theme)
rgb = double(actual);
if numel(rgb) == 3
    rgb = reshape(rgb, 1, 3);
end
if size(rgb, 2) ~= 3
    fail('%s background is not an RGB row.', label);
end
for r = 1:size(rgb, 1)
    if max(abs(rgb(r, :) - theme.bg)) > 1/255
        fail('%s background row %d is not the dark theme color.', label, r);
    end
end
end


function rgb = as_rgb(value, label)
if ischar(value) || isstring(value)
    fail('%s is a named color (%s). Expected an RGB triplet.', label, char(string(value)));
end
rgb = double(value(:))';
if numel(rgb) ~= 3 || any(~isfinite(rgb))
    fail('%s is not a finite RGB triplet.', label);
end
end


function fail(varargin)
error('check_plot_theme:Fail', varargin{:});
end


function close_quietly(fig)
try
    if isgraphics(fig)
        close(fig);
    end
catch
end
end
