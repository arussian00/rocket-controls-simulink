function m = loop_margins(L, opts)
%LOOP_MARGINS  Exact-delay margins and a Padé closed-loop stability test.
%
%   m = loop_margins(L)
%   m = loop_margins(L, checkStability=false)
%
%   allmargin and margin keep the delay in L. isstable cannot see the
%   internal delay that feedback() would create, so the stability flag uses
%   rational_delay (Padé orders 4 and 6). Skip that flag on a long schedule
%   sweep; the 6 dB / 30 deg test does not need it.

arguments
    L
    opts.checkStability (1,1) logical = true
end

am = margins_exact(L);
m.all = am;
if opts.checkStability
    m.stable = closed_loop_stable(L);
else
    m.stable = false;
end

if isempty(am.PhaseMargin)
    m.pm = NaN;
    m.wc = NaN;
    m.delayMargin = NaN;
else
    [m.pm, idx] = min(am.PhaseMargin(:));
    freqs = am.PMFrequency(:);
    m.wc = freqs(idx);
    if isfield(am, 'DelayMargin') && numel(am.DelayMargin) >= idx
        delays = am.DelayMargin(:);
        m.delayMargin = delays(idx);
    elseif m.wc > 0
        m.delayMargin = deg2rad(m.pm) / m.wc;
    else
        m.delayMargin = NaN;
    end
end

gm = 20*log10(am.GainMargin(:));
freq = am.GMFrequency(:);
pos = find(gm > 0 & isfinite(gm));
neg = find(gm < 0 & isfinite(gm));
if isempty(pos)
    m.gmUp = Inf;
    m.wGmUp = NaN;
else
    [m.gmUp, k] = min(gm(pos));
    m.wGmUp = freq(pos(k));
end
if isempty(neg)
    m.gmDown = Inf;
    m.wGmDown = NaN;
else
    [nearest, k] = max(gm(neg));
    m.gmDown = -nearest;
    m.wGmDown = freq(neg(k));
end

try
    [Gm, Pm, Wcg, Wcp] = margin(L);
    m.matlabGm = 20*log10(Gm);
    m.matlabPm = Pm;
    m.matlabWcg = Wcg;
    m.matlabWcp = Wcp;
catch ME
    warning('loop_margins:Margin', ...
        ['margin() could not use the exact delay (%s). ' ...
         'Reporting the allmargin result instead.'], ME.message);
    m.matlabGm = m.gmUp;
    m.matlabPm = m.pm;
    m.matlabWcg = m.wGmUp;
    m.matlabWcp = m.wc;
end
end


function am = margins_exact(L)
try
    am = allmargin(L);
catch ME
    warning('loop_margins:Allmargin', ...
        ['allmargin() could not use the exact delay (%s). ' ...
         'Margins below are for a 6th-order Padé approximation.'], ME.message);
    am = allmargin(rational_delay(L, 6));
end
if ~isfield(am, 'GainMargin') || isempty(am.GainMargin)
    am.GainMargin = NaN;
    am.GMFrequency = NaN;
end
if ~isfield(am, 'PhaseMargin')
    am.PhaseMargin = [];
    am.PMFrequency = [];
end
end


function stable = closed_loop_stable(L)
s4 = pade_is_stable(L, 4);
s6 = pade_is_stable(L, 6);
if s4 ~= s6
    warning('loop_margins:PadeDisagree', ...
        ['Padé orders 4 and 6 disagree on closed-loop stability ' ...
         '(order 4 stable = %d, order 6 stable = %d). ' ...
         'Reporting the order-6 result.'], double(s4), double(s6));
end
stable = s6;
end


function stable = pade_is_stable(L, n)
Tcl = feedback(rational_delay(L, n), 1);
try
    stable = isstable(Tcl);
catch
    p = pole(rational_delay(Tcl, n));
    stable = ~isempty(p) && all(real(p) < 1e-6);
end
end
