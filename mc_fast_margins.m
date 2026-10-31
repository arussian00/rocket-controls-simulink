function m = mc_fast_margins(D, ratio, lambda, delay, actHz, actZeta)
%MC_FAST_MARGINS  Gain and phase margins on a fixed frequency grid.
%
%   m = mc_fast_margins(D, ratio, lambda, delay, actHz, actZeta)
%
%   L(s) = ratio * C(s) * Actuator(s) * 1/(s^2 - lambda) * exp(-s*delay),
%   with C the acceleration PID. The grid is the same test that reproduces
%   the published 64 deg / 19.4 dB nominal margins. loop_margins remains
%   the design-point check; this one is for the flight stations, where
%   calling allmargin on every Monte Carlo sample is the slow part.
%
%   Phase margin is folded into (-180, 180] degrees. Upward gain margin is
%   the smallest positive margin, in dB. Empty crossings come back as NaN
%   or Inf the same way a loop that never crosses does.

if nargin < 6 || isempty(actZeta)
    actZeta = D.actZeta;
end
w = margin_grid();
s = 1j * w;
wn = 2 * pi * actHz;
C = D.Kp + D.Ki ./ s + (D.Kd .* s) ./ (D.Tf .* s + 1);
Act = wn^2 ./ (s.^2 + 2 * actZeta * wn .* s + wn^2);
Rigid = 1 ./ (s.^2 - lambda);
L = ratio .* C .* Act .* Rigid .* exp(-s * delay);
mag = abs(L);
ph = rad2deg(unwrap(angle(L)));

pmList = [];
wcList = [];
for i = 1:numel(w) - 1
    if (mag(i) - 1) * (mag(i + 1) - 1) > 0
        continue
    end
    denom = mag(i + 1) - mag(i);
    if denom == 0
        f = 0;
    else
        f = (1 - mag(i)) / denom;
    end
    if f < 0 || f > 1
        continue
    end
    phc = ph(i) + f * (ph(i + 1) - ph(i));
    wc = w(i) * (w(i + 1) / w(i))^f;
    pmList(end + 1) = fold_pm(phc + 180); %#ok<AGROW>
    wcList(end + 1) = wc; %#ok<AGROW>
end
if isempty(pmList)
    m.pm = NaN;
    m.wc = NaN;
else
    [m.pm, idx] = min(pmList);
    m.wc = wcList(idx);
end

gmList = [];
lo = floor(min(ph) / 360) - 1;
hi = ceil(max(ph) / 360) + 1;
for tgt = lo:hi
    target = -180 + 360 * tgt;
    for i = 1:numel(w) - 1
        if (ph(i) - target) * (ph(i + 1) - target) > 0
            continue
        end
        denom = ph(i + 1) - ph(i);
        if denom == 0
            continue
        end
        f = (target - ph(i)) / denom;
        if f < -0.02 || f > 1.02
            continue
        end
        f = min(max(f, 0), 1);
        magc = mag(i) + f * (mag(i + 1) - mag(i));
        gmList(end + 1) = -20 * log10(max(magc, 1e-15)); %#ok<AGROW>
    end
end
pos = gmList(gmList > 0.5);
if isempty(pos)
    m.gm = Inf;
else
    m.gm = min(pos);
end
m.ratio = ratio;
m.lambda = lambda;
m.delay = delay;
m.actHz = actHz;
end


function w = margin_grid()
persistent grid
if isempty(grid)
    grid = logspace(-3, 2.5, 5000).';
end
w = grid;
end


function pm = fold_pm(pm)
pm = mod(pm + 180, 360) - 180;
end
