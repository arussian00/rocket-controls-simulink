function plant = mc_realize(V, draw, point)
%MC_REALIZE  Dispersed pitch plant at one trajectory point.
%
%   plant = mc_realize(V, draw, point)
%
%   point is a struct with mass, thrust, length, q, speed, lcp and mu, or
%   one entry of V.flight. The controller keeps point.mu (the scheduled
%   effectiveness). Thrust, inert mass, propellant, inertia, gimbal arm and
%   aero use the draw. That is the companion's truth-versus-guidance split:
%   the flight computer still divides by the nominal mu.
%
%   On this fixed trajectory the inert mass scales with dryMass and the
%   propellant still aboard scales with isp. The CG, inertia, gimbal arm
%   and aero stiffness come from vehicle_mass_properties (the tank model),
%   then the CG offset, inertia factor, CNa scale and static-margin offset
%   are applied. The guidance mu is that tank model with the draw at
%   nominal, not the mid-length rod stored in the CSV.
%
%   plant.knobs is the whole draw, including reserved names (lr_blend,
%   sloshFreq, bendingFreq, gridFinEff) that this rigid plant does not use.

if ~isfield(point, 'config')
    point = flight_to_point(point, V);
end

cfg = string(point.config);
if cfg == "stack"
    dry = V.mDryBooster + V.library.starship.mDry + V.library.starship.mPay;
    refSm = V.staticMarginAscent;
else
    dry = V.mDryBooster;
    refSm = V.staticMarginLanding;
end
prop = max(point.mass - dry, 0);
mass = dry * draw.dryMass + prop * draw.isp;
thrust = point.thrust * draw.thrust;
% Truth mass properties are the tank model, then the draw. The guidance
% mu is the same model with the draw held at nominal, so a thrust or CG
% error is not cancelled by rewriting the schedule. The rod column in the
% CSV is not the schedule anymore.
mp = vehicle_mass_properties(mass, thrust, point.q, cfg, V, refSm);
mpCmd = vehicle_mass_properties(point.mass, point.thrust, point.q, cfg, V, refSm);
arm = max(mp.arm + draw.cg, 0.05 * mp.length);
I = mp.I * draw.inertia;
lcp = mp.lcp - draw.staticMargin * V.diam;
lambda = point.q * (V.CNa * draw.cna) * V.area * lcp / max(I, 1);
muTrue = thrust * arm * V.gimbalThrustFraction / max(I, 1);
muCmd = max(mpCmd.mu, 1e-6);
if point.speed > 1 && lambda ~= 0
    gustAcc = -lambda / point.speed;
else
    gustAcc = 0;
end

plant.mass = mass;
plant.thrust = thrust;
plant.arm = arm;
plant.inertia = I;
plant.lcp = lcp;
plant.lambda = lambda;
plant.muTrue = muTrue;
plant.muCmd = muCmd;
plant.ratio = muTrue / muCmd;
plant.gustAcc = gustAcc;
plant.actHz = max(V.actHz * draw.actBandwidth, 1);
plant.actZeta = V.actZeta;
plant.delay = max(V.sensorDelay * draw.sensorDelay, 0);
plant.rateDeg = max(V.gimbalRateDeg * draw.rateLimit, 1);
plant.rate = deg2rad(plant.rateDeg);
plant.noiseDeg = draw.sensorNoise;
plant.wind = draw.wind;
plant.knobs = draw;
plant.name = string(point.name);
plant.time = point.time;
plant.config = cfg;
end


function point = flight_to_point(flight, V)
point.name = flight.name;
if isfield(flight, 'time')
    point.time = flight.time;
else
    point.time = NaN;
end
point.mass = flight.mass;
point.thrust = flight.thrust;
point.length = flight.length;
point.q = flight.q;
point.speed = flight.speed;
point.lcp = flight.lcp;
point.mu = flight.mu;
if flight.length < 0.5 * (V.stackLength + V.Lbooster)
    point.config = "booster";
else
    point.config = "stack";
end
end
