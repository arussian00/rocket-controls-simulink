function [num, den] = notch_for_flight(flight)
%NOTCH_FOR_FLIGHT  Sensor-notch numerator and denominator for one condition.
%
%   [num, den] = notch_for_flight(flight)
%
%   Reads flight.bendWn (set in vehicle_params from bending_model) and
%   returns the same coefficients as notch_tf, as row vectors for the
%   Simulink Transfer Fcn. A flight without bendWn is a unit gain so an
%   older workspace still simulates.

if ~isfield(flight, 'bendWn') || isempty(flight.bendWn)
    num = 1;
    den = 1;
    return
end
[num, den] = tfdata(notch_tf(flight.bendWn(:).'), 'v');
num = num(:).';
den = den(:).';
end
