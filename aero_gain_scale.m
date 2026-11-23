function k = aero_gain_scale(lambda)
%AERO_GAIN_SCALE  Uniform scale on Kp, Ki and Kd when aero outruns the loop.
%
%   k = aero_gain_scale(lambda)
%
%   The nominal acceleration PID (Kp = 1, Ki = 0.02, Kd = 2) crosses over
%   near 2.1 rad/s and holds about 64 deg / 19 dB while sqrt(lambda) is
%   well below that. With the moving centre of gravity the brake reaches
%   lambda about 2 /s^2 (unstable pole about 1.4 rad/s) and those fixed
%   gains go unstable. Scaling all three gains by k moves the crossover
%   out ahead of the unstable pole. k is 1 at liftoff and max-q, so the
%   published margins there do not move. The cap is 3.2: past that the
%   2 Hz grid-fin servo, at the peak-q fins-only sample, drops under 6 dB.
%
%   k = max(1, min(3.2, (sqrt(lambda) / 0.88)^2)). Tuned so the brake
%   (lambda about 2, thrust vectoring) and the fins-only sample just
%   before it (net lambda about 1.5, after the fin restoring moment)
%   both clear 30 deg and 6 dB upward.

p = sqrt(max(lambda, 0));
k = (p ./ 0.88) .^ 2;
k = max(k, 1);
k = min(k, 3.2);
end
