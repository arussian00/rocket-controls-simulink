function apply_plot_theme(target)
%APPLY_PLOT_THEME  Text color chosen for contrast with its backdrop.
%
%   apply_plot_theme(fig)
%   apply_plot_theme(ax)
%
%   Walks the figure that owns target. The axes interior stays dark and
%   the grid stays grey. Line colors are left alone. Each text color is
%   near-white or near-black, whichever has the higher WCAG ratio against
%   the color behind that text (see text_backdrop and text_ink). Tab
%   labels use the light tab strip, which has no color property. Legend
%   objects are skipped, and legend text, box, edge and font are restored
%   if an axes color change moved them. Call this after building a figure
%   and again after any redraw that adds titles, labels, tabs or tables.

if nargin < 1 || isempty(target) || ~isgraphics(target)
    return
end
fig = ancestor_figure(target);
theme = plot_theme();
legs = legend_objects(fig);
snap = snapshot_legends(legs);

objs = findall(fig);
for pass = 1:2
    for k = 1:numel(objs)
        obj = objs(k);
        if ~isgraphics(obj) || in_legend(obj)
            continue
        end
        if pass == 1
            paint_interior(obj, theme);
        else
            paint_text(obj);
        end
    end
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


function paint_interior(obj, theme)
try
    kind = get(obj, 'Type');
catch
    return
end
if any(strcmp(kind, {'axes', 'polaraxes'}))
    setp(obj, 'Color', theme.bg);
end
end


function paint_text(obj)
try
    kind = get(obj, 'Type');
catch
    return
end
switch kind
    case {'axes', 'polaraxes'}
        paint_axes_text(obj);
    case 'colorbar'
        ink = text_ink(text_backdrop(obj));
        setp(obj, 'Color', ink);
        try
            if isprop(obj, 'Label')
                setp(obj.Label, 'Color', ink);
            end
        catch
        end
    case {'text', 'textbox', 'textboxshape', 'subplottext'}
        setp(obj, 'Color', text_ink(text_backdrop(obj)));
    case {'textarrow', 'textarrowshape'}
        setp(obj, 'TextColor', text_ink(text_backdrop(obj)));
    case 'tiledlayout'
        paint_layout(obj);
    case {'uitab', 'uipanel'}
        setp(obj, 'ForegroundColor', text_ink(text_backdrop(obj)));
    case {'uilabel', 'uibutton', 'uicheckbox', 'uiradiobutton', ...
            'uidropdown', 'uieditfield', 'uispinner', 'uitextarea', 'uilistbox'}
        setp(obj, 'FontColor', text_ink(text_backdrop(obj)));
    case 'uitable'
        setp(obj, 'ForegroundColor', text_ink(text_backdrop(obj)));
end
end


function paint_axes_text(ax)
ink = text_ink(text_backdrop(ax));
setp(ax, 'XColor', ink);
setp(ax, 'YColor', ink);
setp(ax, 'ZColor', ink);
% Axis color also drives an automatic grid. Put the grey back afterwards.
theme = plot_theme();
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
        setp(rulers(r), 'Color', ink);
        setp(rulers(r), 'TickLabelColor', ink);
        try
            if isprop(rulers(r), 'Label')
                setp(rulers(r).Label, 'Color', text_ink(text_backdrop(rulers(r).Label)));
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
        label = ax.(name);
    catch
        continue
    end
    if isgraphics(label)
        setp(label, 'Color', text_ink(text_backdrop(label)));
    end
end
end


function paint_layout(obj)
for name = ["Title", "Subtitle", "XLabel", "YLabel"]
    if ~isprop(obj, name)
        continue
    end
    try
        label = obj.(name);
    catch
        continue
    end
    if isgraphics(label)
        setp(label, 'Color', text_ink(text_backdrop(label)));
    end
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
