function apply_plot_theme(target)
%APPLY_PLOT_THEME  Near-white text on the dark figure background.
%
%   apply_plot_theme(fig)
%   apply_plot_theme(ax)
%
%   Walks the figure that owns target. Axes stay dark, grid lines stay
%   grey, and line colors are left alone. Legend objects and their
%   children are skipped. Legend text, box, edge and font are put back if
%   an axes color change moved them. Call this after building a figure and
%   again after any redraw that adds titles, labels, tabs or tables.

if nargin < 1 || isempty(target) || ~isgraphics(target)
    return
end
fig = ancestor_figure(target);
theme = plot_theme();
legs = legend_objects(fig);
snap = snapshot_legends(legs);

objs = findall(fig);
for k = 1:numel(objs)
    obj = objs(k);
    if ~isgraphics(obj) || in_legend(obj)
        continue
    end
    paint_one(obj, theme);
end
restore_legends(legs, snap);
end


function fig = ancestor_figure(target)
fig = target;
if strcmp(get(target, 'Type'), 'figure')
    return
end
try
    parent = ancestor(target, 'figure');
catch
    parent = [];
end
if ~isempty(parent) && isgraphics(parent)
    fig = parent;
end
end


function legs = legend_objects(fig)
try
    legs = findall(fig, 'Type', 'legend');
catch
    legs = gobjects(0);
end
end


function snap = snapshot_legends(legs)
props = {'TextColor', 'Color', 'EdgeColor', 'FontSize', 'FontName', ...
    'FontWeight', 'FontAngle'};
snap = cell(numel(legs), 1);
for i = 1:numel(legs)
    s = struct();
    s.props = struct();
    if ~isgraphics(legs(i))
        s.kids = gobjects(0);
        s.colors = {};
        snap{i} = s;
        continue
    end
    for p = 1:numel(props)
        name = props{p};
        try
            if isprop(legs(i), name)
                s.props.(name) = legs(i).(name);
            end
        catch
        end
    end
    try
        kids = findall(legs(i), 'Type', 'text');
    catch
        kids = gobjects(0);
    end
    colors = cell(numel(kids), 1);
    for c = 1:numel(kids)
        try
            colors{c} = kids(c).Color;
        catch
            colors{c} = [];
        end
    end
    s.kids = kids;
    s.colors = colors;
    snap{i} = s;
end
end


function restore_legends(legs, snap)
for i = 1:min(numel(legs), numel(snap))
    if ~isgraphics(legs(i))
        continue
    end
    s = snap{i};
    names = fieldnames(s.props);
    for p = 1:numel(names)
        restore_prop(legs(i), names{p}, s.props.(names{p}));
    end
    for c = 1:numel(s.kids)
        if c <= numel(s.colors) && isgraphics(s.kids(c)) && ~isempty(s.colors{c})
            restore_prop(s.kids(c), 'Color', s.colors{c});
        end
    end
end
end


function restore_prop(obj, name, value)
try
    if isequaln(obj.(name), value)
        return
    end
    obj.(name) = value;
catch
end
end


function tf = in_legend(obj)
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
        if isprop(h, 'Tag') && strcmp(char(string(get(h, 'Tag'))), 'legend')
            tf = true;
            return
        end
        h = get(h, 'Parent');
    catch
        return
    end
end
end


function paint_one(obj, theme)
try
    kind = get(obj, 'Type');
catch
    return
end
switch kind
    case 'figure'
        setp(obj, 'Color', theme.bg);
    case {'axes', 'polaraxes'}
        paint_axes(obj, theme);
    case 'colorbar'
        setp(obj, 'Color', theme.ink);
        try
            if isprop(obj, 'Label')
                setp(obj.Label, 'Color', theme.ink);
            end
        catch
        end
    case {'text', 'textbox', 'textboxshape', 'subplottext'}
        setp(obj, 'Color', theme.ink);
    case {'textarrow', 'textarrowshape'}
        setp(obj, 'TextColor', theme.ink);
    case 'tiledlayout'
        paint_layout(obj, theme);
    case {'uitab', 'uipanel'}
        setp(obj, 'ForegroundColor', theme.ink);
        setp(obj, 'BackgroundColor', theme.bg);
    case 'uitabgroup'
        setp(obj, 'ForegroundColor', theme.ink);
    case 'uigridlayout'
        setp(obj, 'BackgroundColor', theme.bg);
    case 'uilabel'
        setp(obj, 'FontColor', theme.ink);
        setp(obj, 'BackgroundColor', theme.bg);
    case 'uitable'
        paint_table(obj, theme);
end
end


function paint_axes(ax, theme)
setp(ax, 'Color', theme.bg);
setp(ax, 'XColor', theme.ink);
setp(ax, 'YColor', theme.ink);
setp(ax, 'ZColor', theme.ink);
% Axis color also drives an automatic grid. Put the grey back afterwards.
setp(ax, 'GridColor', theme.grid);
setp(ax, 'MinorGridColor', theme.grid);
setp(ax, 'GridAlpha', theme.gridAlpha);
for name = ["XAxis", "YAxis", "ZAxis"]
    if ~isprop(ax, name)
        continue
    end
    try
        rulers = ax.(name);
    catch
        continue
    end
    for r = 1:numel(rulers)
        setp(rulers(r), 'Color', theme.ink);
        setp(rulers(r), 'TickLabelColor', theme.ink);
        try
            if isprop(rulers(r), 'Label')
                setp(rulers(r).Label, 'Color', theme.ink);
            end
        catch
        end
    end
end
for name = ["Title", "Subtitle", "XLabel", "YLabel", "ZLabel"]
    if ~isprop(ax, name)
        continue
    end
    try
        setp(ax.(name), 'Color', theme.ink);
    catch
    end
end
end


function paint_layout(obj, theme)
for name = ["Title", "Subtitle", "XLabel", "YLabel"]
    if ~isprop(obj, name)
        continue
    end
    try
        setp(obj.(name), 'Color', theme.ink);
    catch
    end
end
end


function paint_table(obj, theme)
setp(obj, 'ForegroundColor', theme.ink);
if ~assign_ok(obj, 'BackgroundColor', [theme.bg; theme.bg])
    setp(obj, 'BackgroundColor', theme.bg);
end
end


function ok = assign_ok(obj, name, value)
ok = false;
try
    if isempty(obj) || ~isprop(obj, name)
        return
    end
    obj.(name) = value;
    ok = true;
catch
    ok = false;
end
end


function setp(obj, name, value)
if isempty(obj)
    return
end
try
    if isgraphics(obj) && isprop(obj, name)
        obj.(name) = value;
        return
    end
catch
end
try
    set(obj, name, value);
catch
end
end
