function y = step_delay(sys, t)
%STEP_DELAY  Unit-step response sampled on t, for plants that may have delays.
%
%   y = step_delay(sys, t)
%
%   step() is the supported way to simulate a model with an internal delay
%   (feedback of a plant that has an input delay). If this MATLAB release
%   still refuses that model, the response is recomputed from a 6th-order
%   Padé approximation and a warning is issued. y is a column, one sample
%   per entry of t.

t = t(:);
try
    [y, tOut] = step(sys, t);
catch ME
    warning('step_delay:Pade', ...
        ['step() could not use the exact delay (%s). ' ...
         'Using a 6th-order Padé approximation.'], ME.message);
    [y, tOut] = step(rational_delay(sys, 6), t);
end
y = squeeze(y);
y = y(:);
tOut = tOut(:);
if numel(y) ~= numel(t)
    y = interp1(tOut, y, t, 'linear', 'extrap');
end
end
