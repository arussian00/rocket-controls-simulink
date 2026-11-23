function m = loop_margins(L, opts)
%LOOP_MARGINS  Exact-delay margins and a Padé closed-loop stability test.
%
%   m = loop_margins(L)
%   m = loop_margins(L, checkStability=false)
%
%   allmargin keeps the exact delay for the gain and phase margins.
%   The stability flag is not one Padé polynomial. Orders 4, 6, 8 and 10
%   are built as state-space models (Padé on the delay states only),
%   reduced with minreal and balreal, and then the poles are the
%   eigenvalues of A. A single high order that jumps into the right
%   half-plane while the lower orders agree is dropped as a badly scaled
%   delay polynomial. The exact-delay frequency response has to stay off
%   the point -1 as well (modulus margin). "stable" means the retained
%   rightmost real part is left of -1e-3 1/s and that modulus margin is
%   at least 0.2. margin() is not called on a model that already has an
%   internal delay: that call warns that the closed loop is unstable when
%   it cannot certify the interconnection. It is called on the order-6
%   Padé model instead. Skip the pole test on a long schedule sweep; the
%   6 dB / 30 deg test does not need it.

arguments
    L
    opts.checkStability (1,1) logical = true
end

am = margins_exact(L);
m.all = am;
m.delayStable = delay_stable_flag(am);
m.modulusMargin = exact_modulus_margin(L);
if opts.checkStability
    [m.stable, m.stability, m.rightmost, m.padeOrders, detail] = pade_consensus(L);
    m.rightmostByOrder = detail.rights;
    m.padeArtifact = detail.artifact;
    m.padeFailed = detail.failed;
else
    m.stable = false;
    m.stability = "unchecked";
    m.rightmost = NaN;
    m.padeOrders = [];
    m.rightmostByOrder = [];
    m.padeArtifact = [];
    m.padeFailed = [];
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


function [stable, status, rightmost, orders, detail] = pade_consensus(L)
% Closed-loop poles of several Padé orders, in state space.
% One order is not a proof. A high order whose rightmost pole sits more
% than 1 rad/s to the right of the median of the others, and in the right
% half-plane, is a badly scaled delay polynomial (the order-10 companion
% matrix of an unreduced tf does this). It is reported and then dropped.
% "stable" also requires a clear abscissa (-1e-3 1/s, not just roundoff)
% and an exact-delay modulus margin of at least 0.2.
orders = [4 6 8 10];
rights = nan(size(orders));
failed = false(size(orders));
artifact = false(size(orders));
for k = 1:numel(orders)
    [rights(k), failed(k)] = pade_rightmost(L, orders(k));
end
use = ~failed & isfinite(rights);
if nnz(use) >= 3
    for k = 1:numel(orders)
        others = use;
        others(k) = false;
        if ~use(k) || nnz(others) < 2
            continue
        end
        med = median(rights(others));
        if rights(k) > 0.5 && rights(k) > med + 1
            artifact(k) = true;
        end
    end
end
kept = use & ~artifact;
detail.rights = rights;
detail.failed = failed;
detail.artifact = artifact;
detail.orders = orders;
if any(kept)
    rightmost = max(rights(kept));
else
    rightmost = NaN;
end

% -1e-3 1/s is about a 1000 s time constant. The attitude integrator
% (near -Ki/Kp = -0.02) is on the stable side of this line. A pole that
% is only a roundoff left of the axis is "marginal", not "stable".
abscissaLimit = -1e-3;
modulusFloor = 0.2;
modm = exact_modulus_margin(L);
if nnz(kept) < 2 || ~isfinite(rightmost)
    stable = false;
    status = "indeterminate";
    warning('loop_margins:PadeDisagree', ...
        ['Padé orders %s did not leave two usable closed-loop models ' ...
         '(rightmost Re by order: %s, dropped as artifact: %s). ' ...
         'The stability flag is not true.'], ...
        sprintf('%d ', orders), sprintf('%.4g ', rights), ...
        sprintf('%d ', orders(artifact)));
elseif all(rights(kept) < abscissaLimit) && isfinite(modm) && modm >= modulusFloor
    stable = true;
    status = "stable";
    if any(artifact)
        warning('loop_margins:PadeArtifact', ...
            ['Padé order(s) %s returned a rightmost Re of %s, which does not ' ...
             'agree with orders %s (median %.4g). Those orders were dropped ' ...
             'as a badly scaled delay polynomial. Retained rightmost Re = %.4g, ' ...
             'exact-delay modulus margin %.3f.'], ...
            sprintf('%d ', orders(artifact)), sprintf('%.4g ', rights(artifact)), ...
            sprintf('%d ', orders(kept)), median(rights(kept)), rightmost, modm);
    end
elseif all(rights(kept) < 1e-7)
    stable = false;
    status = "marginal";
    warning('loop_margins:Marginal', ...
        ['Retained Padé orders %s are in the left half-plane but not by a ' ...
         'clear margin (rightmost Re = %.4g, modulus margin %.3f).'], ...
        sprintf('%d ', orders(kept)), rightmost, modm);
else
    stable = false;
    status = "unstable";
    warning('loop_margins:Unstable', ...
        ['Closed-loop poles of the retained Padé orders %s are not left of ' ...
         '%.4g 1/s (rightmost Re = %.4g, by order: %s).'], ...
        sprintf('%d ', orders(kept)), abscissaLimit, rightmost, sprintf('%.4g ', rights));
end
end


function [rightmost, failed] = pade_rightmost(L, n)
% Eigenvalues of a balanced, reduced closed loop. pole() on the transfer
% function would call roots() on a denominator whose coefficients span
% many decades once the Padé order reaches 10, and that companion matrix
% invents large positive poles. Staying in state space, and approximating
% only the delay, avoids that.
failed = false;
rightmost = NaN;
try
    Ld = rational_delay(L, n);
    Tcl = feedback(Ld, 1);
    Tcl = reduce_realization(Tcl);
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
end


function sys = reduce_realization(sys)
% minreal drops cancelled hidden modes. balreal then drops states whose
% Hankel singular values are numerical dust, which is where a high-order
% Padé leaves its spurious modes.
try
    sys = minreal(ss(sys), 1e-5);
catch
    sys = ss(sys);
end
if exist('balreal', 'file') ~= 2 && exist('balreal', 'file') ~= 6
    return
end
try
    [sysb, hsv] = balreal(sys);
    hsv = hsv(:);
    if isempty(hsv) || ~all(isfinite(hsv))
        sys = sysb;
        return
    end
    floorH = max(max(hsv) * 1e-8, 1e-8);
    drop = find(hsv < floorH);
    if ~isempty(drop) && numel(drop) < numel(hsv) && exist('modred', 'file')
        sys = modred(sysb, drop, 'Truncate');
    else
        sys = sysb;
    end
catch
    % balreal is not available, or this realization has no gramian.
end
end


function mgn = exact_modulus_margin(L)
% min |1 + L(jw)| with the delay left exact. freqresp keeps it.
% About 0.8 at a 47 deg phase margin. This is a distance, not a proof
% when the open loop itself is unstable; the pole test covers that case.
mgn = NaN;
w = logspace(-3, 3, 2500);
try
    h = freqresp(L, w);
catch
    try
        h = freqresp(rational_delay(L, 6), w);
    catch
        return
    end
end
h = h(:);
if isempty(h)
    return
end
mgn = min(abs(1 + h));
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
