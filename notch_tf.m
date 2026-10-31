function N = notch_tf(wn)
%NOTCH_TF  Sensor notch at each bending frequency in wn [rad/s].
%
%   Second order per mode. Zero damping 0.015, pole damping 0.40, depth
%   20*log10(0.015/0.40) dB. An empty wn is unity.
c = structure_constants();
N = tf(1, 1);
wn = wn(:).';
for k = 1:numel(wn)
    if ~(isfinite(wn(k)) && wn(k) > 0)
        continue
    end
    N = N * tf([1, 2 * c.notchZetaZ * wn(k), wn(k)^2], ...
        [1, 2 * c.notchZetaP * wn(k), wn(k)^2]);
end
end
