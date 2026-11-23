function modes = slosh_model(mp)
%SLOSH_MODEL  First-mode pendulum slosh for each LOX and CH4 tank.
%
%   modes = slosh_model(mp)
%
%   mp is one vehicle_mass_properties result. The equivalent pendulum is
%   the NASA SP-106 / Abramson model for a circular cylinder:
%
%       lambda1 = 1.841   (first root of J1')
%       wn^2    = (a_x / R) * lambda1 * tanh(lambda1 * h / R)
%       m_s/m   = 2*tanh(lambda1*h/R) / (lambda1*(lambda1^2-1)*(h/R))
%
%   a_x is the axial specific force T/m (what an accelerometer reads), not
%   T/m - g. The hinge is at the free surface and the bob hangs aft of it,
%   which is where the thrust acceleration puts the liquid. Shallow tanks
%   whose pendulum would be longer than the fill put the bob at the liquid
%   CG instead. Baffle damping is light and constant: zeta = 0.05.
%
%   xB is the bob's body-forward coordinate from the CG (positive toward
%   the nose). A forward bob and an aft bob couple into pitch with
%   opposite signs.

c = slosh_constants();
modes = repmat(empty_slosh(), 1, 0);
if mp.ax <= 0.05 || mp.radius <= 0
    return
end
R = mp.radius;
for i = 1:numel(mp.tanks)
    col = mp.tanks(i);
    h = col.h;
    if col.m <= 0 || h < c.minH
        continue
    end
    hr = h / R;
    th = tanh(c.lambda1 * hr);
    if th < 1e-8
        continue
    end
    ms = col.m * (2 * th) / (c.lambda1 * (c.lambda1^2 - 1) * hr);
    lp = R / (c.lambda1 * th);
    if lp > h
        xBob = col.xCg;
    else
        xBob = col.xFs + lp;
    end
    s = empty_slosh();
    s.name = string(col.name);
    s.ms = ms;
    s.h = h;
    s.lp = lp;
    s.wn = sqrt(mp.ax * c.lambda1 * th / R);
    s.hz = s.wn / (2 * pi);
    s.zeta = c.zeta;
    s.xBob = xBob;
    s.xB = mp.cg - xBob;
    modes(end + 1) = s; %#ok<AGROW>
end
end


function c = slosh_constants()
s = structure_constants();
c.zeta = s.sloshZeta;
c.lambda1 = s.sloshLambda1;
c.minH = s.sloshMinH;
end


function s = empty_slosh()
s = struct('name', "", 'ms', 0, 'h', 0, 'lp', 0, ...
    'wn', 0, 'hz', 0, 'zeta', 0, 'xBob', 0, 'xB', 0);
end
