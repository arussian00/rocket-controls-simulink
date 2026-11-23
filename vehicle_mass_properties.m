function mp = vehicle_mass_properties(mass_kg, thrust_N, q_Pa, config, V, refSmCal)
%VEHICLE_MASS_PROPERTIES  CG, inertia, gimbal arm, mu and lambda vs propellant.
%
%   mp = vehicle_mass_properties(mass_kg, thrust_N, q_Pa, config, V)
%   mp = vehicle_mass_properties(..., refSmCal)
%
%   The companion simulator treats the booster as a uniform rod (CG at
%   mid-length, I = m*L^2/12). This replaces that with a LOX/CH4 tank
%   layout whose dry masses, lengths and diameter are the Super Heavy and
%   Starship numbers in vehicle_params (themselves from rocket_params.m).
%   Both tanks drain at the same fill fraction, which is a constant
%   mixture ratio.
%
%   The centre of pressure is body-fixed. refSmCal is the static margin
%   that this CP would give if the CG were still at mid-length (the old
%   assumption: -0.5 caliber on the stack, 0 on the booster). The margin
%   that comes back is the real one, after the CG has moved.
%
%   config is "stack" (Starship on Super Heavy) or "booster".

if nargin < 6 || isempty(refSmCal) || ~isfinite(refSmCal)
    if strcmp(string(config), "stack")
        refSmCal = V.staticMarginAscent;
    else
        refSmCal = V.staticMarginLanding;
    end
end
config = string(config);
if ~(strcmp(config, "stack") || strcmp(config, "booster"))
    error('vehicle_mass_properties:Config', ...
        'config must be "stack" or "booster", not "%s".', config);
end

geom = tank_geometry(V);
R = V.diam / 2;
area = V.area;

if strcmp(config, "stack")
    mShip = geom.ship.dry + geom.ship.prop + geom.ship.pay;
    mProp = mass_kg - mShip - geom.booster.dry;
    mProp = min(max(mProp, 0), geom.booster.prop);
    parts = stage_parts(geom.ship, geom.ship.prop, 0);
    parts = stage_parts(geom.booster, mProp, geom.ship.L, parts);
    vehicleL = geom.ship.L + geom.booster.L;
    xGimbal = geom.ship.L + (geom.booster.L - geom.gimbalInset);
else
    mProp = mass_kg - geom.booster.dry;
    mProp = min(max(mProp, 0), geom.booster.prop);
    parts = stage_parts(geom.booster, mProp, 0);
    vehicleL = geom.booster.L;
    xGimbal = geom.booster.L - geom.gimbalInset;
end

m = [parts.m].';
x = [parts.x].';
Iself = [parts.I].';
M = sum(m);
cg = sum(m .* x) / M;
I = sum(Iself + m .* (x - cg).^2);
arm = xGimbal - cg;
% CP station that reproduces refSmCal when the CG sits at mid-length.
% sm = (xCp - L/2)/diam, so a negative (unstable) margin puts the CP forward.
xCp = vehicleL / 2 + refSmCal * V.diam;
lcp = cg - xCp;
sm = -lcp / V.diam;
lamCoeff = q_Pa * V.CNa * area * lcp;
lambda = lamCoeff / I;
mu = thrust_N * arm * V.gimbalThrustFraction / I;

tanks = [parts.tank];
tanks = tanks([tanks.m] > 0);

mp.mass = M;
mp.massRequested = mass_kg;
mp.thrust = thrust_N;
mp.q = q_Pa;
mp.config = config;
mp.length = vehicleL;
mp.cg = cg;
mp.I = I;
mp.arm = arm;
mp.xGimbal = xGimbal;
mp.xCp = xCp;
mp.lcp = lcp;
mp.staticMarginCal = sm;
mp.lambda = lambda;
mp.mu = mu;
mp.ax = thrust_N / M;
mp.radius = R;
mp.tanks = tanks;
end


function geom = tank_geometry(V)
% Stations are metres from each stage's nose, positive aft. The columns
% are the cylindrical equivalent of the propellant volume (dome volume is
% folded in). Bulkheads are fixed from the full-load column so the liquid
% always sits against the aft dome under thrust.
%
% Raptor tanks burn about 3.6 kg of LOX per kg of methane. Densities are
% the boiling-point liquids. Engine mass is a public Raptor estimate and
% is a slice of the dry mass, not an extra mass. None of these three
% numbers are in rocket_params.m; the dry mass, propellant load, length
% and diameter are.
c = structure_constants();
geom.of = c.of;
geom.rhoLox = c.rhoLox;
geom.rhoCh4 = c.rhoCh4;
geom.mEngine = c.mEngine;
geom.gimbalInset = c.gimbalInset;
geom.booster = stage_geom(V.library.superheavy, V.Lbooster, ...
    V.library.superheavy.engAscent, geom, "booster");
nShip = 6;
if isfield(V.library.starship, 'engAscent')
    nShip = V.library.starship.engAscent;
end
mPay = 0;
if isfield(V.library.starship, 'mPay')
    mPay = V.library.starship.mPay;
end
ship = stage_geom(V.library.starship, V.library.starship.L, nShip, geom, "ship");
ship.pay = mPay;
geom.ship = ship;
end


function stage = stage_geom(lib, L, nEng, geom, kind)
stage.kind = kind;
stage.L = L;
stage.dry = lib.mDry;
stage.prop = lib.mProp;
stage.nEng = nEng;
stage.pay = 0;
[mLox, mCh4] = split_prop(stage.prop, geom.of);
A = pi * (9/2)^2;
% Diameter is the Super Heavy / Starship 9 m. vehicle_params stores one
% diameter for the study; both stages use it.
if isfield(lib, 'diam')
    A = pi * (lib.diam / 2)^2;
end
stage.A = A;
hLox = mLox / (geom.rhoLox * A);
hCh4 = mCh4 / (geom.rhoCh4 * A);
if strcmp(kind, "booster")
    % 5 m engine bay at the tail. 0.8 m between the CH4 aft dome and the
    % full LOX surface. The full CH4 column then starts about 1 m from the nose.
    stage.loxAft = L - 5;
    stage.ch4Aft = stage.loxAft - hLox - 0.8;
    stage.fwdX = 4;
    stage.tankA = 5;
    stage.tankB = L - 5;
    stage.aftX = L - 3;
    stage.engX = L - 1.5;
else
    stage.loxAft = L - 10;
    stage.ch4Aft = stage.loxAft - hLox - 1.5;
    stage.fwdX = 6;
    stage.tankA = 10;
    stage.tankB = L - 8;
    stage.aftX = L - 4;
    stage.engX = L - 2;
    stage.payX = 8;
end
stage.hLoxFull = hLox;
stage.hCh4Full = hCh4;
end


function parts = stage_parts(stage, mProp, x0, parts)
if nargin < 4
    parts = struct('m', {}, 'x', {}, 'I', {}, 'tank', {});
end
mEng = min(stage.nEng * structure_constants().mEngine, 0.40 * stage.dry);
mAft = 0.15 * stage.dry;
mFwd = 0.20 * stage.dry;
mWall = stage.dry - mEng - mAft - mFwd;
if mWall < 0
    mWall = 0;
end
span = stage.tankB - stage.tankA;
parts = add_part(parts, mFwd, x0 + stage.fwdX, 0);
parts = add_part(parts, mWall, x0 + 0.5 * (stage.tankA + stage.tankB), ...
    mWall * span^2 / 12);
parts = add_part(parts, mAft, x0 + stage.aftX, 0);
parts = add_part(parts, mEng, x0 + stage.engX, 0);
if stage.pay > 0
    parts = add_part(parts, stage.pay, x0 + stage.payX, 0);
end
c = structure_constants();
[mLox, mCh4] = split_prop(mProp, c.of);
frac = 0;
if stage.prop > 0
    frac = mProp / stage.prop;
end
parts = add_liquid(parts, strcat(stage.kind, " CH4"), mCh4, c.rhoCh4, ...
    x0 + stage.ch4Aft, frac * stage.hCh4Full);
parts = add_liquid(parts, strcat(stage.kind, " LOX"), mLox, c.rhoLox, ...
    x0 + stage.loxAft, frac * stage.hLoxFull);
end


function [mLox, mCh4] = split_prop(mProp, of)
mLox = mProp * of / (of + 1);
mCh4 = mProp - mLox;
end


function parts = add_part(parts, m, x, Iself)
if m <= 0
    return
end
blank = empty_tank();
blank.m = 0;
parts(end + 1).m = m; %#ok<AGROW>
parts(end).x = x;
parts(end).I = Iself;
parts(end).tank = blank;
end


function parts = add_liquid(parts, name, m, rho, xAft, h)
blank = empty_tank();
if m <= 0 || h <= 0
    return
end
xCg = xAft - 0.5 * h;
Iself = m * h^2 / 12;
tank = blank;
tank.name = name;
tank.m = m;
tank.rho = rho;
tank.h = h;
tank.xAft = xAft;
tank.xCg = xCg;
tank.xFs = xAft - h;
tank.Iself = Iself;
parts(end + 1).m = m; %#ok<AGROW>
parts(end).x = xCg;
parts(end).I = Iself;
parts(end).tank = tank;
end


function tank = empty_tank()
tank = struct('name', "", 'm', 0, 'rho', 0, 'h', 0, ...
    'xAft', 0, 'xCg', 0, 'xFs', 0, 'Iself', 0);
end
