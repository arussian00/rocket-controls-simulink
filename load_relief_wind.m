function w = load_relief_wind(V, t)
%LOAD_RELIEF_WIND  Crosswind through max-q: shear plus a 1-cosine gust.
%
%   w = load_relief_wind(V, t)
%
%   The shear grows linearly from 0 at liftoff to V.windShear at
%   V.windShearTime (the max-q time) and then holds. The discrete gust is
%   (V.windGust/2) * (1 - cos(2*pi*x)) on x in [0, 1], with x measured from
%   V.windGustStart over V.windGustDur. Its peak is V.windGust, on top of
%   the shear, at the middle of that window (T+46 s with the defaults).

t = t(:);
shear = V.windShear * min(max(t, 0) / V.windShearTime, 1);
x = (t - V.windGustStart) / V.windGustDur;
gust = zeros(size(t));
on = x >= 0 & x <= 1;
gust(on) = V.windGust * 0.5 .* (1 - cos(2 * pi * x(on)));
w = shear + gust;
end
