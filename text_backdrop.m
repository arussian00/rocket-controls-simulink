function bg = text_backdrop(obj)
%TEXT_BACKDROP  Color actually behind this object's text.
%
%   bg = text_backdrop(obj)
%
%   Titles, tick labels and axis labels sit in the margin, on the parent
%   container, not on the dark axes interior. text() drawn in the axes
%   sits on the axes color unless it has its own box. Tab labels sit on
%   the tab strip, which MATLAB does not expose and which stays light.
%   A table returns one row per stripe. Legends return their own box.

theme = plot_theme();
bg = theme.fallback;
if isempty(obj) || ~isgraphics(obj)
    return
end
leg = legend_of(obj);
if ~isempty(leg)
    bg = opaque_or(leg, 'Color', [1 1 1]);
    return
end
try
    kind = get(obj, 'Type');
catch
    return
end
switch kind
    case 'uitab'
        bg = theme.tabStrip;
    case {'uilabel', 'uibutton', 'uicheckbox', 'uiradiobutton', ...
            'uidropdown', 'uieditfield', 'uispinner', 'uitextarea', 'uilistbox'}
        own = prop_color(obj, 'BackgroundColor');
        if isempty(own)
            bg = container_from(parent_of(obj));
        else
            bg = own(1, :);
        end
    case 'uitable'
        rows = prop_color(obj, 'BackgroundColor');
        if isempty(rows)
            bg = [1 1 1; theme.fallback];
        else
            bg = rows;
        end
    case {'axes', 'polaraxes'}
        bg = container_from(parent_of(obj));
    case 'colorbar'
        bg = container_from(parent_of(obj));
    case {'text', 'textbox', 'textboxshape', 'subplottext'}
        bg = text_object_backdrop(obj);
    case {'textarrow', 'textarrowshape'}
        own = prop_color(obj, 'TextBackgroundColor');
        if isempty(own)
            own = prop_color(obj, 'BackgroundColor');
        end
        if isempty(own)
            bg = container_from(parent_of(obj));
        else
            bg = own(1, :);
        end
    otherwise
        bg = container_from(obj);
end
end


function bg = text_object_backdrop(obj)
own = prop_color(obj, 'BackgroundColor');
if ~isempty(own)
    bg = own(1, :);
    return
end
if is_margin_text(obj)
    bg = container_from(parent_of(obj));
    return
end
ax = [];
try
    ax = ancestor(obj, 'axes');
catch
end
if ~isempty(ax) && isgraphics(ax)
    interior = prop_color(ax, 'Color');
    if isempty(interior)
        bg = container_from(parent_of(ax));
    else
        bg = interior(1, :);
    end
    return
end
bg = container_from(parent_of(obj));
end


function tf = is_margin_text(tx)
tf = false;
try
    if strcmp(get(tx, 'Type'), 'subplottext')
        tf = true;
        return
    end
    parent = tx.Parent;
catch
    return
end
if ~isgraphics(parent)
    return
end
pk = get(parent, 'Type');
if any(strcmp(pk, {'tiledlayout', 'colorbar'}))
    tf = true;
    return
end
ax = [];
if any(strcmp(pk, {'axes', 'polaraxes'}))
    ax = parent;
else
    try
        ax = ancestor(tx, 'axes');
    catch
    end
end
if isempty(ax) || ~isgraphics(ax)
    return
end
candidates = gobjects(0);
for name = ["Title", "Subtitle", "XLabel", "YLabel", "ZLabel"]
    if ~isprop(ax, name)
        continue
    end
    try
        h = ax.(name);
    catch
        h = [];
    end
    if ~isempty(h) && isgraphics(h)
        candidates(end + 1) = h; %#ok<AGROW>
    end
end
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
        try
            h = rulers(r).Label;
        catch
            h = [];
        end
        if ~isempty(h) && isgraphics(h)
            candidates(end + 1) = h; %#ok<AGROW>
        end
    end
end
if isempty(candidates)
    return
end
tf = any(candidates == tx);
end


function bg = container_from(h)
theme = plot_theme();
bg = theme.fallback;
for step = 1:20 %#ok<NASGU>
    if isempty(h) || ~isgraphics(h)
        return
    end
    try
        kind = get(h, 'Type');
    catch
        return
    end
    if strcmp(kind, 'root')
        return
    end
    if ~any(strcmp(kind, {'axes', 'polaraxes', 'legend', 'colorbar', ...
            'text', 'textbox', 'tiledlayout'}))
        rows = fill_of(h);
        if ~isempty(rows)
            bg = rows(1, :);
            return
        end
    end
    h = parent_of(h);
end
end


function rows = fill_of(h)
rows = [];
try
    kind = get(h, 'Type');
catch
    return
end
switch kind
    case 'figure'
        rows = prop_color(h, 'Color');
    case {'uitab', 'uipanel', 'uibuttongroup', 'uigridlayout', ...
            'uiflowcontainer'}
        rows = prop_color(h, 'BackgroundColor');
end
end


function rows = prop_color(obj, name)
rows = [];
try
    if isprop(obj, name)
        rows = color_rows(obj.(name));
    end
catch
    rows = [];
end
end


function bg = opaque_or(obj, name, fallback)
rows = prop_color(obj, name);
if isempty(rows)
    bg = fallback;
else
    bg = rows(1, :);
end
end


function parent = parent_of(obj)
parent = [];
try
    parent = obj.Parent;
catch
    parent = [];
end
end


function leg = legend_of(obj)
leg = [];
h = obj;
for step = 1:16 %#ok<NASGU>
    if isempty(h) || ~isgraphics(h)
        return
    end
    try
        if strcmp(get(h, 'Type'), 'legend')
            leg = h;
            return
        end
        if isprop(h, 'Tag') && strcmp(char(string(get(h, 'Tag'))), 'legend')
            leg = h;
            return
        end
        h = h.Parent;
    catch
        return
    end
end
end
