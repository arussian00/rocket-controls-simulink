function test_flight_mass_contract()
%TEST_FLIGHT_MASS_CONTRACT  massProps may be absent on D.flight; scalars are enough.
%
%   test_flight_mass_contract
%
%   No Simulink and no Control System Toolbox. Does not run the pitch
%   loop and does not treat the brake-entry instability as a failure.
%   A negative margin there is the recorded flight result; this test
%   checks that the tank model still produces it.

here = fileparts(mfilename('fullpath'));
addpath(here);
V = vehicle_params();

fprintf('V.flight massProps field present: %d\n', double(isfield(V.flight, 'massProps')));
for k = 1:numel(V.flight)
    flight = V.flight(k);
    [mp, ~, ~] = flight_mass_props(flight, V);
    match_record(mp, flight);
    fprintf('  %s  mu %.4f  lambda %.4e  arm %.2f m  (record matches rebuild)\n', ...
        char(string(flight.name)), flight.mu, flight.lambda, flight.arm);
end

% Same copy design_pitch_controller does, then drop the nested struct.
% structure_flight has to keep working from the scalar record alone.
D = struct();
for k = 1:numel(V.flight)
    flight = V.flight(k);
    flight.marker = k;
    if isfield(flight, 'massProps')
        flight = rmfield(flight, 'massProps');
    end
    D.flight(k) = flight;
end
if isfield(D.flight, 'massProps')
    error('test_flight_mass_contract:Strip', ...
        'rmfield did not remove massProps; the regression is not exercising the missing-field path.');
end
for k = 1:numel(D.flight)
    [mp, bendWn, ~] = flight_mass_props(D.flight(k), V);
    match_record(mp, D.flight(k));
    if numel(bendWn) < 1 || ~(bendWn(1) > 0)
        error('test_flight_mass_contract:Bend', ...
            'No bending frequency for %s.', char(string(D.flight(k).name)));
    end
end
fprintf('Missing massProps: rebuild matches mu, lambda and arm on all three conditions.\n');

mpBrake = first_brake(V, here);
if ~(mpBrake.lambda > 1)
    error('test_flight_mass_contract:BrakeHidden', ...
        ['Brake lambda is %.4f /s^2. The unstable entry must stay in the model, ' ...
         'not be replaced by a zero aero term.'], mpBrake.lambda);
end
fprintf(['Brake sample lambda %.3f /s^2, static margin %.2f cal. ' ...
    'Unstable entry, not a pass.\n'], mpBrake.lambda, mpBrake.staticMarginCal);
fprintf('test_flight_mass_contract passed.\n');
end


function match_record(mp, flight)
tol = 1e-6 * max(1, abs(flight.mu));
if abs(mp.mu - flight.mu) > tol || abs(mp.lambda - flight.lambda) > 1e-8 ...
        || abs(mp.arm - flight.arm) > 1e-6 || abs(mp.I - flight.inertia) > 1e-3 * flight.inertia
    error('test_flight_mass_contract:Mismatch', ...
        ['%s rebuild mu %.6g lambda %.6g arm %.6g I %.6g; ' ...
         'flight record mu %.6g lambda %.6g arm %.6g I %.6g.'], ...
        char(string(flight.name)), mp.mu, mp.lambda, mp.arm, mp.I, ...
        flight.mu, flight.lambda, flight.arm, flight.inertia);
end
end


function mp = first_brake(V, here)
file = fullfile(here, 'data', 'superheavy_booster_trajectory.csv');
fid = fopen(file, 'r');
if fid < 0
    error('test_flight_mass_contract:Csv', 'Cannot open %s.', file);
end
cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
header = fgetl(fid);
cols = split_csv(header);
iPhase = find(strcmp(cols, 'phase'), 1);
iMass = find(strcmp(cols, 'mass_kg'), 1);
iThrust = find(strcmp(cols, 'thrust_N'), 1);
iQ = find(strcmp(cols, 'q_Pa'), 1);
if isempty(iPhase) || isempty(iMass) || isempty(iThrust) || isempty(iQ)
    error('test_flight_mass_contract:Csv', 'Trajectory CSV is missing phase, mass, thrust or q.');
end
while true
    line = fgetl(fid);
    if ~ischar(line)
        error('test_flight_mass_contract:Csv', 'No brake row in %s.', file);
    end
    parts = split_csv(line);
    if numel(parts) < max([iPhase, iMass, iThrust, iQ])
        continue
    end
    if strcmp(parts{iPhase}, 'brake')
        mass = str2double(parts{iMass});
        thrust = str2double(parts{iThrust});
        q = str2double(parts{iQ});
        mp = vehicle_mass_properties(mass, thrust, q, "booster", V, 0);
        return
    end
end
end


function parts = split_csv(line)
parts = strsplit(line, ',');
end
