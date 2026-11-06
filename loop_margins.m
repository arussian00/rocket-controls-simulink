function m = loop_margins(L, opts)
%LOOP_MARGINS  Exact-delay margins and a Padé closed-loop stability test.
%
%   m = loop_margins(L)
%   m = loop_margins(L, checkStability=false)
%
%   allmargin keeps the exact delay for the gain and phase margins.
%   The stability flag is the closed-loop poles of Padé orders 4, 6, 8
%   and 10. They must agree, and every pole must lie in the open left
%   half-plane. margin() is not called on a model that already has an
%   internal delay: that call warns that the closed loop is unstable when
%   it cannot certify the interconnection. It is called on the order-6
%   Padé model instead, the same model the pole test uses. Skip the pole
%   test on a long schedule sweep; the 6 dB / 30 deg test does not need it.

arguments
    L
    opts.checkStability (1,1) logical = true
end

am = margins_exact(L);
m.all = am;
m.delayStable = delay_stable_flag(am);
if opts.checkStability
    [m.stable, m.stability, m.rightmost, m.padeOrders] = pade_consensus(L);
else
    m.stable = false;
    m.stability = "unchecked";
    m.rightmost = NaN;
    m.padeOrders = [];
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

% margin() on a model that already contains an internal delay warns
% "The closed-loop system is unstable" when it will not certify that
% interconnection, including when the closed-loop poles are in the left
% half-plane. Ask margin() about the same order-6 Padé model whose poles
% were just tested, so a warning and the stability flag cannot disagree.
% Input-delay attitude loops stay on the exact delay: margin() accepts
% those and does not warn.
LforMargin = L;
if internal_delay(L)
    LforMargin = rational_delay(L, 6);
end
try
    [Gm, Pm, Wcg, Wcp] = margin(LforMargin);
    m.matlabGm = 20*log10(Gm);
    m.matlabPm = Pm;
    m.matlabWcg = Wcg;
    m.matlabWcp = Wcp;
catch ME
    warning('loop_margins:Margin', ...
        ['margin() could not use the model it was given (%s). ' ...
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


function [stable, status, rightmost, orders] = pade_consensus(L)
% Closed-loop poles of several Padé orders. One order is not a proof:
% they have to agree, and every pole has to sit in the open left half-plane.
% pole() on the unreduced feedback model keeps hidden modes that a
% cancelled input-output map would drop.
orders = [4 6 8 10];
rights = nan(size(orders));
flags = false(size(orders));
failed = false(size(orders));
for k = 1:numel(orders)
    [flags(k), rights(k), failed(k)] = pade_poles_stable(L, orders(k));
end
rightmost = max(rights);
if any(failed) || any(isnan(rights))
    stable = false;
    status = "indeterminate";
    warning('loop_margins:PadeDisagree', ...
        ['Padé orders %s did not all return closed-loop poles ' ...
         '(rightmost Re by order: %s). The stability flag is not true.'], ...
        sprintf('%d ', orders), sprintf('%.4g ', rights));
elseif all(flags)
    stable = true;
    status = "stable";
elseif ~any(flags)
    stable = false;
    status = "unstable";
    warning('loop_margins:Unstable', ...
        ['Closed-loop poles of Padé orders %s are not in the open left ' ...
         'half-plane (rightmost Re = %.4g).'], ...
        sprintf('%d ', orders), rightmost);
else
    stable = false;
    status = "indeterminate";
    warning('loop_margins:PadeDisagree', ...
        ['Padé orders %s disagree on closed-loop stability ' ...
         '(rightmost Re by order: %s). The stability flag is not true.'], ...
        sprintf('%d ', orders), sprintf('%.4g ', rights));
end
end


function [stable, rightmost, failed] = pade_poles_stable(L, n)
failed = false;
stable = false;
rightmost = NaN;
try
    Tcl = feedback(rational_delay(L, n), 1);
    p = pole(Tcl);
catch
    failed = true;
    return
end
if isempty(p)
    failed = true;
    return
end
rightmost = max(real(p));
% +1e-8 keeps a roundoff pole on the axis from flipping the flag, and
% still rejects a slow unstable mode.
stable = all(real(p) < 1e-8);
end


function tf = internal_delay(sys)
tf = false;
try
    d = sys.InternalDelay;
    tf = ~isempty(d) && any(d(:) > 0);
catch
    tf = false;
end
end


function flag = delay_stable_flag(am)
% allmargin.Stable is 1, 0, or NaN (cannot assess). Empty when this
% release does not return the field. It is recorded, not used as the
% only verdict: on an internal delay it can be 0 while the closed-loop
% poles are in the left half-plane.
flag = NaN;
if isstruct(am) && isfield(am, 'Stable') && ~isempty(am.Stable)
    flag = am.Stable(1);
end
end
