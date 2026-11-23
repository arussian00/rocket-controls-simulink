function sysR = rational_delay(sys, n)
%RATIONAL_DELAY  Replace delays with a Padé approximation, if there are any.
%
%   sysR = rational_delay(sys)
%   sysR = rational_delay(sys, n)
%
%   isstable, pole and minreal reject internal delays, and feedback() turns
%   an input or output delay into one. Frequency-domain tools (margin,
%   allmargin, bode, step) keep the exact delay. Use this only where a
%   rational model is required. Default order is 6, which is accurate well
%   past the pitch-loop crossover for the delays in this project (0 to 0.1 s).
%
%   The Padé polynomial is applied to the delay block, and the result is
%   kept as a state-space model. pade() of a high-order transfer function
%   builds one denominator whose coefficients span tens of decades; roots()
%   of that polynomial invents large positive poles (order 10 has done
%   this on the drift loop). A delay state-space block does not.

if nargin < 2
    n = 6;
end
if ~hasdelay(sys)
    sysR = sys;
    return
end

[inD, outD, ioD, intD] = read_delays(sys);
sisoIn = isscalar(inD) && inD > 0 && delay_total(outD) == 0 ...
    && delay_total(ioD) == 0 && delay_total(intD) == 0;
if sisoIn
    sys0 = sys;
    sys0.InputDelay = 0;
    sysR = series_delay(sys0, inD, n, true);
    return
end
sisoOut = isscalar(outD) && outD > 0 && delay_total(inD) == 0 ...
    && delay_total(ioD) == 0 && delay_total(intD) == 0;
if sisoOut
    sys0 = sys;
    sys0.OutputDelay = 0;
    sysR = series_delay(sys0, outD, n, false);
    return
end

% Internal delay (feedback of a delayed loop). pade() on the state-space
% model augments only the delay states. Convert first so the result is not
% a single ill-conditioned transfer-function polynomial.
try
    sysR = pade(ss(sys), n);
catch
    sysR = pade(sys, n);
end
end


function sysR = series_delay(sys0, delay, n, atInput)
[num, den] = pade(delay, n);
blk = ss(tf(num, den));
try
    [blk, ~] = balreal(blk);
catch
end
if atInput
    sysR = series(blk, ss(sys0));
else
    sysR = series(ss(sys0), blk);
end
end


function [inD, outD, ioD, intD] = read_delays(sys)
inD = 0;
outD = 0;
ioD = 0;
intD = 0;
try
    inD = sys.InputDelay;
catch
end
try
    outD = sys.OutputDelay;
catch
end
try
    ioD = sys.ioDelay;
catch
end
try
    intD = sys.InternalDelay;
catch
end
end


function total = delay_total(d)
if isempty(d)
    total = 0;
else
    total = sum(d(:));
end
end
