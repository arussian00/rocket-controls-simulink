function rgb = color_rows(value)
%COLOR_ROWS  Opaque RGB rows, or [] when the color is transparent.
%
%   rgb = color_rows(value)
%
%   Accepts an RGB triplet, an N-by-3 list, a color name, or a hex code.
%   'none' and empty values return [].

rgb = [];
if isempty(value)
    return
end
if isstring(value) || ischar(value) || iscellstr(value)
    rgb = named_color(char(string(value)));
    return
end
if ~isnumeric(value)
    return
end
value = double(value);
if isvector(value) && numel(value) == 3
    rgb = reshape(value, 1, 3);
elseif size(value, 2) == 3
    rgb = value;
elseif size(value, 1) == 3
    rgb = value.';
else
    return
end
if any(~isfinite(rgb(:)))
    rgb = [];
    return
end
rgb = min(max(rgb, 0), 1);
end


function rgb = named_color(name)
key = lower(strtrim(name));
if isempty(key) || strcmp(key, 'none')
    rgb = [];
    return
end
if startsWith(key, '#')
    rgb = hex_color(key);
    return
end
switch key
    case {'w', 'white'}
        rgb = [1 1 1];
    case {'k', 'black'}
        rgb = [0 0 0];
    case {'r', 'red'}
        rgb = [1 0 0];
    case {'g', 'green'}
        rgb = [0 1 0];
    case {'b', 'blue'}
        rgb = [0 0 1];
    case {'c', 'cyan'}
        rgb = [0 1 1];
    case {'m', 'magenta'}
        rgb = [1 0 1];
    case {'y', 'yellow'}
        rgb = [1 1 0];
    otherwise
        rgb = [];
end
end


function rgb = hex_color(key)
digits = key(2:end);
if numel(digits) == 3
    digits = [digits(1) digits(1) digits(2) digits(2) digits(3) digits(3)];
end
if numel(digits) ~= 6
    rgb = [];
    return
end
nums = sscanf(digits, '%2x');
if numel(nums) ~= 3
    rgb = [];
    return
end
rgb = (nums(:).' / 255);
end
