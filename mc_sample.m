function draws = mc_sample(spec, N, seed)
%MC_SAMPLE  Draw the dispersion table.
%
%   draws = mc_sample(spec, N, seed)
%   draw0 = mc_sample(spec, 0)
%
%   N = 0 returns one nominal draw (every z = 0). Otherwise rng(seed,
%   'twister') is set and N independent draws are returned. z is clipped
%   to +/-3, then each row's minimum and maximum are applied. Disabled
%   rows stay at their nominal value. draws(i).z keeps the clipped z so a
%   worst run can be traced back to a parameter.

if nargin < 1 || isempty(spec)
    spec = mc_dispersions();
end
if nargin < 2 || isempty(N)
    N = 200;
end
if nargin < 3 || isempty(seed)
    seed = 42;
end

if N == 0
    draws = one_draw(spec, zeros(1, numel(spec)));
    return
end

rng(seed, 'twister');
zAll = randn(N, numel(spec));
zAll = min(max(zAll, -3), 3);
draws = repmat(one_draw(spec, zAll(1, :)), N, 1);
for i = 1:N
    draws(i) = one_draw(spec, zAll(i, :));
end
end


function draw = one_draw(spec, zRow)
draw = struct();
draw.z = struct();
for k = 1:numel(spec)
    name = char(spec(k).name);
    draw.(name) = apply_row(spec(k), zRow(k));
    draw.z.(name) = zRow(k);
end
end


function value = apply_row(row, z)
if ~row.enabled
    value = row.nominal;
    return
end
switch row.mode
    case "scale"
        value = row.nominal * (1 + row.sigma * z);
    case "offset"
        value = row.nominal + row.sigma * z;
    case "noise"
        value = row.sigma;
    otherwise
        value = row.nominal;
end
value = min(max(value, row.minimum), row.maximum);
end
