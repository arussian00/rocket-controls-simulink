function outFile = make_booster_trajectory(companionDir)
%MAKE_BOOSTER_TRAJECTORY  Rebuild the schedule CSV from matlab-rocket-sim.
%
%   make_booster_trajectory
%   make_booster_trajectory("/path/to/matlab-rocket-sim")
%
%   Runs the companion's Super Heavy ascent (starshipStack) and booster
%   return (simulateBooster with the 'superheavy' preset) with ode45, then
%   writes data/superheavy_booster_trajectory.csv. The committed CSV is an
%   RK4 reproduction of the same equations (tools/build_superheavy_trajectory.py)
%   for use without MATLAB. After this function succeeds, re-run
%   run_pitch_tvc so the margins and the Simulink lookups use the ode45 table.
%
%   Centre of mass, inertia, mu and lambda are written by
%   vehicle_mass_properties (tank CG, body-fixed CP). The reference static
%   margin that places the CP is still -0.5 caliber on the stack and 0 on
%   the booster, which is what the rod model used at mid-length.

if nargin < 1 || strlength(string(companionDir)) == 0
    companionDir = find_companion();
end
companionDir = char(companionDir);
addpath(companionDir);
need = {'rocket_params', 'starshipStack', 'simulateBooster', 'fullFlight', ...
    'pointMassSegment', 'earthModel', 'evMECO'};
for k = 1:numel(need)
    if isempty(which(need{k}))
        error('make_booster_trajectory:Missing', ...
            'Cannot find %s. Pass the matlab-rocket-sim folder.', need{k});
    end
end

Pb = rocket_params('superheavy');
Ps = rocket_params('starship');
mu = Pb.g0 * Pb.Re^2;
S = starship_ascent_params(Pb, Ps);

m0 = S.m1dry + S.m1prop + S.m2dry + S.m2prop + S.mPay;
Y0 = [0; 0; 0; 0; m0];
mMECO = m0 - (S.m1prop - S.reserve1);
opts = odeset('Events', @(t, Y) evMECO(t, Y, mMECO), ...
    'RelTol', 1e-8, 'AbsTol', 1e-6, 'MaxStep', 0.5);
[tA, YA] = ode45(@(t, Y) starshipStack(t, Y, S, Pb, mu, 'stack'), [0 400], Y0, opts);
Ysep = YA(end, :).';
mShip = S.m2dry + S.m2prop + S.mPay;
mBoost = Ysep(5) - mShip;
Xb0 = [Ysep(1:4); mBoost; atan2(Ysep(3), Ysep(4)); Ysep(3) / (Pb.Re + Ysep(2))];
outB = simulateBooster(Pb, Xb0, tA(end), Pb.phaseList);
segA = pointMassSegment(tA, YA, Pb, @(t, Y) starshipStack(t, Y, S, Pb, mu, 'stack'), S.T1);
fullB = fullFlight({segA}, {'stack ascent'}, outB);

outFile = fullfile(fileparts(mfilename('fullpath')), 'data', 'superheavy_booster_trajectory.csv');
write_trajectory(outFile, fullB, Pb);
fprintf('Wrote %s (%d rows before decimation are stored at about 1 Hz).\n', ...
    outFile, height(readtable(outFile)));
end


function S = starship_ascent_params(Pb, Ps)
S.m1dry = Pb.m_dry;
S.m1prop = Pb.m_prop;
S.T1 = Pb.engAscent * Pb.T_eng;
S.Isp1sl = Pb.Isp_sl;
S.Isp1vac = Pb.Isp_vac;
S.reserve1 = 400e3;
S.m2dry = Ps.m_dry;
S.m2prop = Ps.m_prop;
S.mPay = 10e3;
S.Cd = 0.4;
S.thrBucket = 0.75;
S.bucketMach = [0.6 0.8 1.5 1.8];
S.A = Pb.A;
S.tKick = 8;
S.kickDur = 5;
S.kickDeg = 0.8;
end


function write_trajectory(outFile, fullB, Pb)
Lstack = 121;
Lboost = Pb.L;
smStack = -0.5;
smBoost = 0.0;
names = string(fullB.phaseList);
n = numel(fullB.t);
phase = strings(n, 1);
config = strings(n, 1);
nEng = zeros(n, 1);
len = zeros(n, 1);
sm = zeros(n, 1);
for i = 1:n
    phase(i) = names(fullB.phase(i));
    if phase(i) == "stack ascent"
        phase(i) = "ascent";
        config(i) = "stack";
        nEng(i) = Pb.engAscent;
        len(i) = Lstack;
        sm(i) = smStack;
    else
        config(i) = "booster";
        len(i) = Lboost;
        sm(i) = smBoost;
        nEng(i) = engines_of(phase(i), Pb);
    end
end
keep = decimate_times(fullB.t, phase);
keep = keep(:);
h = fullB.X(keep, 2);
vx = fullB.X(keep, 3);
vh = fullB.X(keep, 4);
mass = fullB.X(keep, 5);
speed = hypot(vx, vh);
q = fullB.aux.q(keep);
thrust = fullB.aux.thrust(keep);
throttle = fullB.aux.throttle(keep);
mach = zeros(numel(keep), 1);
for i = 1:numel(keep)
    [~, ~, aSound] = earthModel(h(i), Pb);
    mach(i) = speed(i) / aSound;
end
% sm(keep) is the reference margin used to place the CP, not the
% margin after the CG has moved. vehicle_mass_properties overwrites it.
cfgKeep = config(keep);
smKeep = sm(keep);
nk = numel(keep);
cg = zeros(nk, 1);
arm = zeros(nk, 1);
cp = zeros(nk, 1);
lcp = zeros(nk, 1);
I = zeros(nk, 1);
smOut = zeros(nk, 1);
mu = zeros(nk, 1);
lam = zeros(nk, 1);
addpath(fileparts(mfilename('fullpath')));
V = vehicle_params();
for i = 1:nk
    mp = vehicle_mass_properties(mass(i), thrust(i), q(i), cfgKeep(i), V, smKeep(i));
    cg(i) = mp.cg;
    arm(i) = mp.arm;
    cp(i) = mp.xCp;
    lcp(i) = mp.lcp;
    I(i) = mp.I;
    smOut(i) = mp.staticMarginCal;
    mu(i) = mp.mu;
    lam(i) = mp.lambda;
end
tvc = double(thrust > 1);
T = table(fullB.t(keep), phase(keep), config(keep), h, speed, mach, q, mass, ...
    throttle, nEng(keep), thrust, len(keep), cg, arm, cp, lcp, I, sm(keep), ...
    mu, lam, tvc, ...
    'VariableNames', {'t_s', 'phase', 'config', 'altitude_m', 'speed_mps', 'mach', ...
    'q_Pa', 'mass_kg', 'throttle', 'n_eng', 'thrust_N', 'length_m', ...
    'cg_from_nose_m', 'arm_m', 'cp_from_nose_m', 'lcp_m', 'inertia_kgm2', ...
    'static_margin_cal', 'mu_rps2', 'lambda_rps2', 'tvc_active'});
T.static_margin_cal = smOut;
if ~isfolder(fileparts(outFile))
    mkdir(fileparts(outFile));
end
writetable(T, outFile);
end


function keep = decimate_times(t, phase)
keep = 1;
nextT = 0;
prev = phase(1);
for i = 2:numel(t)
    if phase(i) ~= prev || t(i) + 1e-9 >= nextT
        keep(end + 1) = i; %#ok<AGROW>
        nextT = floor(t(i)) + 1;
        prev = phase(i);
    end
end
keep = unique(keep, 'stable');
end


function n = engines_of(phase, Pb)
switch phase
    case "boostback", n = Pb.engBoost;
    case "entry",     n = Pb.engEntry;
    case "brake",     n = Pb.engBrake;
    case "landing",   n = Pb.engLand;
    otherwise,        n = 0;
end
end


function found = find_companion()
candidates = ["../matlab-rocket-sim", "matlab-rocket-sim", ...
    fullfile(getenv('HOME'), 'matlab-rocket-sim')];
for k = 1:numel(candidates)
    if isfile(fullfile(candidates(k), 'rocket_params.m'))
        found = candidates(k);
        return
    end
end
error('make_booster_trajectory:NotFound', ...
    ['Clone https://github.com/arussian00/matlab-rocket-sim and pass its folder. ' ...
     'The CSV already in data/ is the RK4 stand-in.']);
end
