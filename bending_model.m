function modes = bending_model(mp, V)
%BENDING_MODEL  First two free-free body bending modes.
%
%   modes = bending_model(mp, V)
%
%   matlab-rocket-sim has no structural model, so the frequency is a
%   sizing assumption: the full stack at liftoff is 3.5 Hz in the first
%   mode. That sits between the ~0.3 Hz attitude crossover and the 6 Hz
%   gimbal servo, which is where an un-notched loop loses its gain margin.
%   A uniform beam then scales the frequency as
%
%       wn ~ 1/sqrt(m * L^3)
%
%   because wn = (beta*L)^2 * sqrt(EI / (mu * L^4)) and mu = m/L, with EI
%   held fixed. The second mode is the next free-free eigenvalue,
%   (7.853/4.730)^2 times the first. Damping is 1%, a light structural value.
%
%   Mode shapes are the analytic free-free beam, mass-normalized with a
%   uniform mass density (shapes, not the inertia, stay uniform). The
%   gimbal force is applied 1.5 m forward of the aft end. The IMU, which
%   reads the local rotation, is at 8% of the length from the nose
%   (forward avionics on the stack, the interstage bay on the booster).
%
%   phiG is the mode displacement at the gimbal. dphiImu is d(phi)/d(xi)
%   at the IMU with xi measured aft from the nose. The sensor equation is
%   theta_meas = theta - dphiImu * eta, with eta the mass-normalized
%   modal coordinate. Free-free modes are orthogonal to rigid pitch, so
%   they are forced only by the gimbal side force.

c = bending_constants();
betas = [4.730040744862704, 7.853204624095838];
mRef = V.mDryBooster + V.library.superheavy.mProp ...
    + V.library.starship.mDry + V.library.starship.mProp + V.library.starship.mPay;
Lref = V.stackLength;
L = mp.length;
mass = max(mp.mass, 1);
w1 = 2 * pi * c.f1RefHz * sqrt((mRef * Lref^3) / (mass * L^3));

xi = linspace(0, L, 2001);
mu = mass / L;
xG = L - c.gimbalInset;
xImu = c.imuFrac * L;
modes = repmat(empty_mode(), 1, c.nModes);
for i = 1:c.nModes
    bL = betas(i);
    beta = bL / L;
    sig = (cosh(bL) - cos(bL)) / (sinh(bL) - sin(bL));
    z = beta * xi;
    phiU = (cosh(z) + cos(z)) - sig * (sinh(z) + sin(z));
    dphiU = beta * ((sinh(z) - sin(z)) - sig * (cosh(z) + cos(z)));
    norm = sqrt(trapz(xi, mu * phiU.^2));
    phi = phiU / norm;
    dphi = dphiU / norm;
    modes(i).wn = w1 * (bL / betas(1))^2;
    modes(i).hz = modes(i).wn / (2 * pi);
    modes(i).zeta = c.zeta;
    modes(i).phiG = interp1(xi, phi, xG, 'linear');
    modes(i).dphiImu = interp1(xi, dphi, xImu, 'linear');
end
end


function c = bending_constants()
s = structure_constants();
c.f1RefHz = s.f1RefHz;
c.zeta = s.bendZeta;
c.imuFrac = s.imuFrac;
c.gimbalInset = s.gimbalInset;
c.nModes = s.nBend;
end


function m = empty_mode()
m = struct('wn', 0, 'hz', 0, 'zeta', 0, 'phiG', 0, 'dphiImu', 0);
end
