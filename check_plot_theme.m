function check_plot_theme()
%CHECK_PLOT_THEME  Headless check that text contrasts with its backdrop.
%
%   check_plot_theme
%
%   Builds a classic figure and a small uifigure, applies the shared theme,
%   and checks WCAG contrast of 4.5 for every text object against the color
%   behind it. Dark axes get near-white text. Light figure chrome, the tab
%   strip, light panels and light tables get near-black text. A dark tab
%   page still keeps a dark tab label, because the strip stays light.
%   Legends keep the colors and font they had. Line colors stay put.

here = fileparts(mfilename('fullpath'));
addpath(here);
theme = plot_theme();
assert_metric(theme);

fig = figure('Visible', 'off', 'Name', 'theme check classic', 'Color', 'w');
closer = onCleanup(@() close_quietly(fig)); %#ok<NASGU>
figColor = color_rows(fig.Color);

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
txBox = text(ax, 3, 1.5, 'boxed', 'BackgroundColor', 'w');
cb = colorbar(ax);
cb.Label.String = 'scale';
lg = legend(ax, [hLine, hBlack], {'curve', 'mean'});
lg.Location = 'best';
lg.FontSize = 8;
lg.Box = 'on';
lg.Color = [1 1 1];
lg.TextColor = [0.06 0.06 0.06];
lg.EdgeColor = [0.75 0.75 0.75];
ann = annotation(fig, 'textbox', [0.15 0.02 0.2 0.04], ...
    'String', 'on figure', 'EdgeColor', 'none');
annDark = annotation(fig, 'textbox', [0.4 0.02 0.2 0.04], ...
    'String', 'on dark box', 'BackgroundColor', theme.bg, 'EdgeColor', 'none');

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
assert_classic(fig, figColor, ax, ax2, theme, hLine, lineColor, hBlack, ...
    lg, tx, txBox, cb, ann, annDark);
assert_legend_snap(lgDefault, defaultSnap);
assert_painted(fig);

title(ax, 'Pitch redrawn');
xlabel(ax, 'Time again [s]');
tx2 = text(ax, 3, 1.0, 'added');
apply_plot_theme(fig);
assert_classic(fig, figColor, ax, ax2, theme, hLine, lineColor, hBlack, ...
    lg, tx, txBox, cb, ann, annDark);
must_ink('redrawn text', tx2.Color, text_ink(text_backdrop(tx2)));
must_contrast('redrawn text', tx2.Color, text_backdrop(tx2));
assert_legend_snap(lgDefault, defaultSnap);
assert_painted(fig);
if lg.FontSize ~= 8
    fail('Styled legend font size changed from 8 to %.3g.', lg.FontSize);
end

ui = uifigure('Name', 'theme check ui', 'Visible', 'off', ...
    'AutoResizeChildren', 'on', 'Color', theme.fallback);
closerUi = onCleanup(@() close_quietly(ui)); %#ok<NASGU>
root = uigridlayout(ui, [1 1]);
root.Padding = [0 0 0 0];
root.RowHeight = {'1x'};
root.ColumnWidth = {'1x'};
tabs = uitabgroup(root);
tabLight = uitab(tabs, 'Title', 'Light');
lightPage = color_rows(tabLight.BackgroundColor);
if isempty(lightPage)
    lightPage = theme.fallback;
end
if wcag_contrast(theme.lightInk, lightPage(1, :)) >= theme.minContrast
    fail('The default tab page is not a light background.');
end
tabChrome = uitab(tabs, 'Title', 'Chrome');
tabDark = uitab(tabs, 'Title', 'Dark', 'BackgroundColor', theme.bg);

tlL = tiledlayout(tabLight, 1, 1, 'Padding', 'compact');
axL = nexttile(tlL);
plot(axL, 1:3, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
title(axL, 'Light page');
xlabel(axL, 't');
panelL = uipanel(tabChrome, 'Title', 'Light block', 'Position', [8 8 120 40]);
panelD = uipanel(tabChrome, 'Title', 'Dark block', 'Position', [140 8 120 40], ...
    'BackgroundColor', theme.bg);
uitL = uitable(tabChrome, 'Data', {'A', 1}, 'ColumnName', {'n', 'v'}, ...
    'Position', [8 56 250 80]);
uitD = uitable(tabChrome, 'Data', {'B', 2}, 'ColumnName', {'n', 'v'}, ...
    'Position', [8 146 250 80], 'BackgroundColor', [theme.bg; theme.bg]);
lblL = uilabel(tabChrome, 'Text', 'light label', 'Position', [8 230 120 22]);

tlD = tiledlayout(tabDark, 1, 1, 'Padding', 'compact');
axD = nexttile(tlD);
plot(axD, 1:3, 'LineWidth', 1.2);
title(axD, 'Dark page');
xlabel(axD, 't');
text(axD, 2, 0.5, 'inside');
lblD = uilabel(tabDark, 'Text', 'dark label', 'Position', [8 8 120 22]);

apply_plot_theme(ui);
assert_ui(ui, theme, tabLight, tabChrome, tabDark, lightPage, axL, axD, ...
    panelL, panelD, uitL, uitD, lblL, lblD);
assert_painted(ui);

fprintf('check_plot_theme passed.\n');
end


function assert_metric(theme)
if wcag_contrast(theme.lightInk, [1 1 1]) >= theme.minContrast
    fail('Near-white on white should miss the %.1f contrast bar.', theme.minContrast);
end
if wcag_contrast(theme.lightInk, theme.tabStrip) >= theme.minContrast
    fail('Near-white on the tab strip should miss the contrast bar.');
end
if wcag_contrast(theme.darkInk, theme.bg) >= theme.minContrast
    fail('Near-black on the dark axes should miss the contrast bar.');
end
if wcag_contrast(theme.lightInk, theme.bg) < theme.minContrast
    fail('Near-white on the dark axes should clear the contrast bar.');
end
if wcag_contrast(theme.darkInk, [1 1 1]) < theme.minContrast
    fail('Near-black on white should clear the contrast bar.');
end
if wcag_contrast(theme.darkInk, theme.tabStrip) < theme.minContrast
    fail('Near-black on the tab strip should clear the contrast bar.');
end
end


function assert_classic(fig, figColor, ax, ax2, theme, hLine, lineColor, ...
    hBlack, lg, tx, txBox, cb, ann, annDark)
must_near('figure', fig.Color, figColor);
must_near('axes', ax.Color, theme.bg);
must_near('grid', ax.GridColor, theme.grid);
must_ink('ticks', ax.XColor, theme.darkInk);
must_ink('title', ax.Title.Color, theme.darkInk);
must_ink('xlabel', ax.XLabel.Color, theme.darkInk);
must_ink('ylabel', ax.YLabel.Color, theme.darkInk);
must_contrast('title', ax.Title.Color, figColor);
must_contrast('ticks', ax.XColor, figColor);
must_ink('text on axes', tx.Color, theme.lightInk);
must_contrast('text on axes', tx.Color, theme.bg);
must_ink('boxed text', txBox.Color, theme.darkInk);
must_contrast('boxed text', txBox.Color, [1 1 1]);
must_ink('annotation', ann.Color, theme.darkInk);
must_contrast('annotation', ann.Color, figColor);
must_ink('dark annotation', annDark.Color, theme.lightInk);
must_contrast('dark annotation', annDark.Color, theme.bg);
must_ink('colorbar', cb.Color, theme.darkInk);
must_contrast('colorbar', cb.Color, figColor);
must_ink('colorbar label', cb.Label.Color, theme.darkInk);
must_near('line', hLine.Color, lineColor);
must_near('black line', hBlack.Color, [0 0 0]);
must_near('styled legend TextColor', lg.TextColor, [0.06 0.06 0.06]);
must_near('styled legend Color', lg.Color, [1 1 1]);
must_near('styled legend EdgeColor', lg.EdgeColor, [0.75 0.75 0.75]);
must_contrast('styled legend', lg.TextColor, lg.Color);
must_ink('right ticks', ax2.YColor, theme.darkInk);
must_contrast('right ylabel', ax2.YAxis(end).Label.Color, figColor);
end


function assert_ui(ui, theme, tabLight, tabChrome, tabDark, lightPage, axL, axD, ...
    panelL, panelD, uitL, uitD, lblL, lblD)
must_near('uifigure', ui.Color, theme.fallback);
if ~isempty(color_rows(tabLight.BackgroundColor))
    must_near('light tab page', tabLight.BackgroundColor, lightPage);
end
must_near('dark tab page', tabDark.BackgroundColor, theme.bg);

must_ink('light tab label', tabLight.ForegroundColor, theme.darkInk);
must_contrast('light tab label', tabLight.ForegroundColor, theme.tabStrip);
must_ink('chrome tab label', tabChrome.ForegroundColor, theme.darkInk);
must_contrast('chrome tab label', tabChrome.ForegroundColor, theme.tabStrip);
must_ink('dark tab label', tabDark.ForegroundColor, theme.darkInk);
must_contrast('dark tab label', tabDark.ForegroundColor, theme.tabStrip);

must_ink('light page title', axL.Title.Color, theme.darkInk);
must_contrast('light page title', axL.Title.Color, lightPage);
must_ink('light page ticks', axL.XColor, theme.darkInk);
must_ink('dark page title', axD.Title.Color, theme.lightInk);
must_contrast('dark page title', axD.Title.Color, theme.bg);
must_ink('dark page ticks', axD.XColor, theme.lightInk);
must_contrast('dark page ticks', axD.XColor, theme.bg);

lightPanel = color_rows(panelL.BackgroundColor);
if isempty(lightPanel) || wcag_contrast(theme.lightInk, lightPanel(1, :)) >= theme.minContrast
    fail('The default panel is not a light background.');
end
must_ink('light panel', panelL.ForegroundColor, theme.darkInk);
must_contrast('light panel', panelL.ForegroundColor, lightPanel);
must_ink('dark panel', panelD.ForegroundColor, theme.lightInk);
must_contrast('dark panel', panelD.ForegroundColor, theme.bg);

lightRows = color_rows(uitL.BackgroundColor);
if isempty(lightRows)
    lightRows = [1 1 1; theme.fallback];
end
if wcag_contrast(theme.lightInk, lightRows(1, :)) >= theme.minContrast
    fail('The default table is not a light background.');
end
must_ink('light table', uitL.ForegroundColor, theme.darkInk);
must_contrast('light table', uitL.ForegroundColor, lightRows);
must_ink('dark table', uitD.ForegroundColor, theme.lightInk);
must_contrast('dark table', uitD.ForegroundColor, uitD.BackgroundColor);

chromePage = color_rows(tabChrome.BackgroundColor);
if isempty(chromePage)
    chromePage = lightPage;
end
assert_label(lblL, theme, chromePage);
assert_label(lblD, theme, theme.bg);
end


function assert_label(lbl, theme, page)
own = color_rows(lbl.BackgroundColor);
if isempty(own)
    bg = page;
else
    bg = own;
end
must_ink('uilabel', lbl.FontColor, text_ink(bg));
must_contrast('uilabel', lbl.FontColor, bg);
if wcag_contrast(theme.lightInk, bg(1, :)) < theme.minContrast
    must_ink('uilabel on light', lbl.FontColor, theme.darkInk);
elseif wcag_contrast(theme.darkInk, bg(1, :)) < theme.minContrast
    must_ink('uilabel on dark', lbl.FontColor, theme.lightInk);
end
end


function assert_painted(fig)
theme = plot_theme();
objs = findall(fig);
nText = 0;
for k = 1:numel(objs)
    obj = objs(k);
    if ~isgraphics(obj)
        continue
    end
    kind = get(obj, 'Type');
    inLegend = legend_child(obj);
    switch kind
        case {'text', 'textbox', 'textboxshape', 'subplottext'}
            nText = nText + 1;
            bg = text_backdrop(obj);
            must_contrast(kind, obj.Color, bg);
            if ~inLegend
                must_ink(kind, obj.Color, text_ink(bg));
            end
        case {'textarrow', 'textarrowshape'}
            if ~isprop(obj, 'TextColor')
                continue
            end
            bg = text_backdrop(obj);
            must_contrast(kind, obj.TextColor, bg);
            must_ink(kind, obj.TextColor, text_ink(bg));
        case {'axes', 'polaraxes'}
            if inLegend
                continue
            end
            bg = text_backdrop(obj);
            must_ink('XColor', obj.XColor, text_ink(bg));
            must_contrast('XColor', obj.XColor, bg);
            must_ink('YColor', obj.YColor, text_ink(bg));
            must_contrast('YColor', obj.YColor, bg);
            must_ink('ZColor', obj.ZColor, text_ink(bg));
            must_contrast('ZColor', obj.ZColor, bg);
            must_near('axes interior', obj.Color, theme.bg);
            assert_rulers(obj, bg);
        case 'colorbar'
            bg = text_backdrop(obj);
            must_ink('colorbar', obj.Color, text_ink(bg));
            must_contrast('colorbar', obj.Color, bg);
            must_ink('colorbar label', obj.Label.Color, text_ink(text_backdrop(obj.Label)));
            must_contrast('colorbar label', obj.Label.Color, text_backdrop(obj.Label));
        case 'uitab'
            bg = text_backdrop(obj);
            must_ink('tab label', obj.ForegroundColor, theme.darkInk);
            must_ink('tab label', obj.ForegroundColor, text_ink(bg));
            must_contrast('tab label', obj.ForegroundColor, theme.tabStrip);
        case 'uipanel'
            bg = text_backdrop(obj);
            must_ink('panel title', obj.ForegroundColor, text_ink(bg));
            must_contrast('panel title', obj.ForegroundColor, bg);
        case 'uilabel'
            bg = text_backdrop(obj);
            must_ink('uilabel', obj.FontColor, text_ink(bg));
            must_contrast('uilabel', obj.FontColor, bg);
        case 'uitable'
            bg = text_backdrop(obj);
            must_ink('uitable', obj.ForegroundColor, text_ink(bg));
            must_contrast('uitable', obj.ForegroundColor, bg);
    end
end
if nText < 1
    fail('No text objects were found to check.');
end
end


function assert_rulers(ax, bg)
for name = ["XAxis", "YAxis", "ZAxis"]
    rulers = ax.(name);
    for r = 1:numel(rulers)
        must_ink(char(name), rulers(r).Color, text_ink(bg));
        must_contrast(char(name), rulers(r).Color, bg);
        if isprop(rulers(r), 'TickLabelColor')
            must_ink([char(name) ' ticks'], rulers(r).TickLabelColor, text_ink(bg));
            must_contrast([char(name) ' ticks'], rulers(r).TickLabelColor, bg);
        end
    end
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
must_contrast('default legend', lg.TextColor, lg.Color);
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


function tf = legend_child(obj)
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
        h = h.Parent;
    catch
        return
    end
end
end


function must_ink(label, actual, expected)
must_near(label, actual, expected);
end


function must_contrast(label, actual, bg)
rows = color_rows(bg);
fg = color_rows(actual);
if isempty(fg) || isempty(rows)
    fail('%s has no opaque color to compare.', label);
end
need = plot_theme().minContrast;
for r = 1:size(rows, 1)
    ratio = wcag_contrast(fg(1, :), rows(r, :));
    if ~(ratio >= need)
        fail(['%s contrast %.2f is below %.1f. Text [%0.3f %0.3f %0.3f], ' ...
            'background [%0.3f %0.3f %0.3f].'], label, ratio, need, ...
            fg(1), fg(2), fg(3), rows(r, 1), rows(r, 2), rows(r, 3));
    end
end
end


function must_near(label, actual, expected)
rgb = color_rows(actual);
want = color_rows(expected);
if isempty(rgb) || isempty(want) || size(want, 1) ~= 1
    fail('%s is not a single opaque color.', label);
end
if max(abs(rgb(1, :) - want(1, :))) > 1/255
    fail('%s is [%0.3f %0.3f %0.3f], expected [%0.3f %0.3f %0.3f].', ...
        label, rgb(1), rgb(2), rgb(3), want(1), want(2), want(3));
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
