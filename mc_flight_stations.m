function stations = mc_flight_stations(traj)
%MC_FLIGHT_STATIONS  Powered samples for the margin-versus-time curves.
%
%   Liftoff, ascent max-q, mid ascent, late ascent, boostback at peak mu,
%   brake at peak dynamic pressure, landing nearest 1 km, and the last
%   powered sample. Engines-off flip and coast samples are left out.

active = find(traj.tvc_active ~= 0 & traj.mu_rps2 > 1e-3);
if isempty(active)
    error('mc_flight_stations:NoTVC', 'Trajectory has no powered samples.');
end

asc = active(traj.phase(active) == "ascent");
bb = active(traj.phase(active) == "boostback");
br = active(traj.phase(active) == "brake");
ld = active(traj.phase(active) == "landing");
if isempty(asc) || isempty(bb) || isempty(br) || isempty(ld)
    error('mc_flight_stations:Phases', ...
        'Need ascent, boostback, brake and landing samples in the trajectory.');
end

[~, iQ] = max(traj.q_Pa(asc));
[~, iMu] = max(traj.mu_rps2(bb));
[~, iBr] = max(traj.q_Pa(br));
[~, iLd] = min(abs(traj.altitude_m(ld) - 1000));
mid = asc(nearest_time(traj.t_s(asc), 93));
late = asc(nearest_time(traj.t_s(asc), 140));
idx = unique([active(1), asc(iQ), mid, late, bb(iMu), br(iBr), ld(iLd), active(end)], 'stable');

stations = repmat(empty_point(), numel(idx), 1);
for k = 1:numel(idx)
    i = idx(k);
    stations(k).name = traj.phase(i);
    stations(k).time = traj.t_s(i);
    stations(k).mass = traj.mass_kg(i);
    stations(k).thrust = traj.thrust_N(i);
    stations(k).length = traj.length_m(i);
    stations(k).q = traj.q_Pa(i);
    stations(k).speed = traj.speed_mps(i);
    stations(k).lcp = traj.lcp_m(i);
    stations(k).mu = traj.mu_rps2(i);
    stations(k).config = traj.config(i);
    stations(k).phase = traj.phase(i);
end
end


function point = empty_point()
point = struct('name', "", 'time', NaN, 'mass', NaN, 'thrust', NaN, ...
    'length', NaN, 'q', NaN, 'speed', NaN, 'lcp', NaN, 'mu', NaN, ...
    'config', "", 'phase', "");
end


function k = nearest_time(t, target)
[~, k] = min(abs(t(:) - target));
end
