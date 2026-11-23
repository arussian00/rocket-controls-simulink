function traj = booster_trajectory()
%BOOSTER_TRAJECTORY  Super Heavy time history used by the pitch schedule.
%
%   traj = booster_trajectory()
%
%   The CSV is an RK4 reproduction of matlab-rocket-sim
%   (rocket_starship_sim stack ascent + simulateBooster 'superheavy'
%   return). Regenerate the ode45 version with make_booster_trajectory
%   once that repo is on the path; the columns are the same.
%
%   On load, cg, arm, inertia, mu, lambda and the static margin are
%   replaced by vehicle_mass_properties (LOX/CH4 tanks, body-fixed CP).
%   The CSV columns of those names are the old mid-length rod and are not
%   what the schedule uses. tvc_active is 0 when the engines are off
%   (flip and coast). Super Heavy has no entry burn.

here = fileparts(mfilename('fullpath'));
file = fullfile(here, 'data', 'superheavy_booster_trajectory.csv');
if ~isfile(file)
    error('booster_trajectory:Missing', ...
        'Cannot find %s. Run tools/build_superheavy_trajectory.py or make_booster_trajectory.', file);
end
traj = readtable(file, 'TextType', 'string');
V = vehicle_params();
traj = apply_mass_properties(traj, V);
end
