function sysR = rational_delay(sys, n)
%RATIONAL_DELAY  Replace delays with a Padé approximation, if there are any.
%
%   sysR = rational_delay(sys)
%   sysR = rational_delay(sys, n)
%
%   isstable, pole and minreal reject internal delays, and feedback() turns
%   an input/output delay into one. Frequency-domain tools (margin,
%   allmargin, bode, step) keep the exact delay. Use this only where a
%   rational model is required. Default order is 6, which is accurate well
%   past the pitch-loop crossover for the delays in this project (0 to 0.1 s).

if nargin < 2
    n = 6;
end
if hasdelay(sys)
    sysR = pade(sys, n);
else
    sysR = sys;
end
end
