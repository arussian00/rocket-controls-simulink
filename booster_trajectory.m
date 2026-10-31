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
%   The companion holds the centre of mass at mid-length (arm = L/2) and
%   has no centre-of-pressure schedule. cg_from_nose_m and cp_from_nose_m
%   are that assumption: static margin -0.5 caliber on the stack and 0 on
%   the booster alone (grid fins are a later step). tvc_active is 0 when
%   the engines are off (flip and coast). Super Heavy has no entry burn.

here = fileparts(mfilename('fullpath'));
file = fullfile(here, 'data', 'superheavy_booster_trajectory.csv');
if ~isfile(file)
    error('booster_trajectory:Missing', ...
        'Cannot find %s. Run tools/build_superheavy_trajectory.py or make_booster_trajectory.', file);
end
traj = readtable(file, 'TextType', 'string');
end
