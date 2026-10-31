function ret = make_booster_return_trajectory(traj, V)
%MAKE_BOOSTER_RETURN_TRAJECTORY  Booster return for the pitch loop.
%
%   ret = make_booster_return_trajectory()
%   ret = make_booster_return_trajectory(traj, V)
%
%   Starts at hot staging (the first sample that is not stack ascent) and
%   runs through the tower catch. The time history is the committed
%   Super Heavy table, which is the RK4 twin of matlab-rocket-sim
%   (starshipStack to MECO, then simulateBooster 'superheavy': flip,
%   boostback on 13, coast, brake on 13, landing on 3, catch at 65 m).
%   There is no entry burn (engEntry = 0). "entry" here is the unpowered
%   grid-fin descent: the coast samples below finAlt while falling.
%
%   Columns already on the CSV (mass, thrust, q, mu, engine count) are
%   kept. This adds the pitch command, the fin effectiveness, and the
%   steady allocation target. It does not rewrite the CSV.

if nargin < 1 || isempty(traj)
    traj = booster_trajectory();
end
if nargin < 2 || isempty(V)
    V = vehicle_params();
end
G = grid_fin_model(V);

keep = traj.phase ~= "ascent";
tAll = traj.t_s(keep);
[~, ia] = unique(tAll, 'stable');
kept = find(keep);
kept = kept(ia);
pick = @(name) traj.(name)(kept);

ret.t = pick('t_s');
ret.t = ret.t(:);
n = numel(ret.t);
ret.phase = pick('phase');
ret.phase = ret.phase(:);
ret.h = pick('altitude_m'); ret.h = ret.h(:);
ret.speed = pick('speed_mps'); ret.speed = ret.speed(:);
ret.q = pick('q_Pa'); ret.q = ret.q(:);
ret.mass = pick('mass_kg'); ret.mass = ret.mass(:);
ret.thrust = pick('thrust_N'); ret.thrust = ret.thrust(:);
ret.nEng = pick('n_eng'); ret.nEng = ret.nEng(:);
ret.muTvc = pick('mu_rps2'); ret.muTvc = ret.muTvc(:);
ret.inertia = pick('inertia_kgm2'); ret.inertia = ret.inertia(:);
ret.cg = pick('cg_from_nose_m'); ret.cg = ret.cg(:);
ret.length = pick('length_m'); ret.length = ret.length(:);

ret.vh = vertical_rate(ret.t, ret.h);
ret.vx = sqrt(max(ret.speed.^2 - ret.vh.^2, 0));

[ret.muFin, ret.lambdaFin, ret.finArm] = grid_fin_effectiveness( ...
    G, ret.q, ret.inertia, ret.cg);

ret.phaseCode = zeros(n, 1);
for k = 1:n
    ret.phaseCode(k) = phase_code(ret.phase(k), ret.h(k), ret.vh(k), G.finAlt);
end
labels = ["unused", "flip", "boostback", "coast", "entry", "brake", "landing"];
ret.controlPhase = labels(ret.phaseCode + 1).';
ret.controlPhase = ret.controlPhase(:);

ret.thetaCmd = theta_command(ret.phaseCode, ret.h, ret.vx, ret.vh, G);
ret.thetaTrim = atan2(-ret.vx, max(abs(ret.vh), G.aeroVhFloor));
ret.theta0 = atan2(ret.vx(1), ret.vh(1));

ret.wTvc = zeros(n, 1);
ret.wFin = zeros(n, 1);
ret.wRcs = zeros(n, 1);
for k = 1:n
    w = descent_targets(ret.phaseCode(k), ret.muTvc(k), ret.muFin(k));
    ret.wTvc(k) = w(1);
    ret.wFin(k) = w(2);
    ret.wRcs(k) = w(3);
end

ret.events = return_events(ret);
ret.G = G;
ret.note = ['Return from hot staging through the tower catch. ' ...
    'entry is the grid-fin descent below finAlt, not an entry burn.'];
end


function vh = vertical_rate(t, h)
% Central difference, matching a nonuniform numpy.gradient.
n = numel(t);
vh = zeros(n, 1);
vh(1) = (h(2) - h(1)) / (t(2) - t(1));
vh(n) = (h(n) - h(n-1)) / (t(n) - t(n-1));
vh(2:n-1) = (h(3:n) - h(1:n-2)) ./ (t(3:n) - t(1:n-2));
end


function code = phase_code(phase, h, vh, finAlt)
switch phase
    case "flip"
        code = 1;
    case "boostback"
        code = 2;
    case "coast"
        if h < finAlt && vh < 0
            code = 4;
        else
            code = 3;
        end
    case "brake"
        code = 5;
    case "landing"
        code = 6;
    otherwise
        code = 0;
end
end


function thetaCmd = theta_command(phaseCode, h, vx, vh, G)
% Nose angle from local vertical. Flip and boostback point the engines
% uprange (companion bbDir = -1, theta = -pi/2). Coast and entry are
% engines-first. Brake and landing tilt to kill horizontal speed, capped
% at maxTiltDeg, and stand up before the catch.
thetaCmd = atan2(-vx, max(abs(vh), G.aeroVhFloor));
boost = phaseCode == 1 | phaseCode == 2;
thetaCmd(boost) = -pi/2;
land = phaseCode == 5 | phaseCode == 6;
if any(land)
    hAbove = h(land) - G.hLand;
    fade = min(1, max(hAbove - G.fadeUpright, 0) / G.fadeSpan);
    tiltLim = deg2rad(G.maxTiltDeg) .* fade;
    raw = atan2(-vx(land), max(abs(vh(land)), G.landingVhFloor));
    thetaCmd(land) = min(max(raw, -tiltLim), tiltLim);
end
end


function events = return_events(ret)
% The four handoffs the dashboard zooms on. Each time is the first CSV
% sample of the new mode; the blend filter then moves the weight.
specs = { ...
    2, 'Boostback light'; ...
    3, 'Boostback cutoff'; ...
    5, 'Landing-burn light'; ...
    6, '13 to 3 engines'};
events = struct('name', {}, 't', {}, 'phaseCode', {});
for k = 1:size(specs, 1)
    idx = find(ret.phaseCode == specs{k, 1}, 1, 'first');
    if isempty(idx)
        continue
    end
    events(end + 1).name = specs{k, 2}; %#ok<AGROW>
    events(end).t = ret.t(idx);
    events(end).phaseCode = specs{k, 1};
end
end
