function open_pitch_tuner(target, conditionName)
%OPEN_PITCH_TUNER  Open PID Tuner or Control System Designer.
%
%   open_pitch_tuner("pid")
%   open_pitch_tuner("pid", "max-q")
%   open_pitch_tuner("designer", "landing")
%
%   The plant is a_cmd [rad/s^2] -> theta [rad]: rigid body, gimbal actuator
%   and sensor delay. The PID is the normalized controller (Kp, Ki, Kd, Tf).
%   Gimbal angle is a_cmd/mu, with mu on the flight condition (printed below).
%
%   What the apps show is the 1-DOF loop (derivative of the error, no command
%   prefilter). Margins match the Simulink model. The step plot in the app
%   is harsher than run_pitch_tvc, which filters the command and takes the
%   derivative of measured rate so a step does not kick the gimbal.
%
%   Paste tuned gains back by setting V.Kp, V.Ki, V.Kd, V.Tf on the struct
%   from vehicle_params before calling design_pitch_controller.

arguments
    target (1,1) string {mustBeMember(target, ["pid", "designer"])} = "pid"
    conditionName (1,1) string = "liftoff"
end

if ~license('test', 'Control_Toolbox')
    error('open_pitch_tuner:NeedsCST', ...
        'PID Tuner and Control System Designer are part of Control System Toolbox.');
end

V = vehicle_params();
D = design_pitch_controller(V, plot=false, verbose=false);
idx = find([D.flight.name] == conditionName, 1);
if isempty(idx)
    error('open_pitch_tuner:Unknown', ...
        'No flight condition "%s". Use liftoff, max-q or landing.', conditionName);
end
flight = D.flight(idx);

fprintf(['\n%s  |  mu = %.3f 1/s^2   lambda = %.4g 1/s^2\n' ...
    'PID: Kp %.4g, Ki %.4g, Kd %.4g, Tf %.3f\n' ...
    'delta [rad] = a_cmd / mu. Loop margins are the ones to tune.\n\n'], ...
    flight.name, flight.mu, flight.lambda, D.Kp, D.Ki, D.Kd, D.Tf);

switch target
    case "pid"
        open_pid_tuner(flight.plant.G, D.pid);
    case "designer"
        open_designer(flight.plant.G, D.Ctf);
end
end


function open_pid_tuner(G, C)
% pidTuner accepts an input delay on current releases. A release that
% refuses it gets a Padé plant, and the warning says the margins in that
% window are then approximate.
try
    pidTuner(G, C);
    return
catch ME1
end
warning('open_pitch_tuner:Pade', ...
    ['PID Tuner did not open the exact-delay plant (%s). ' ...
     'Opening a 4th-order Padé approximation. Margins in that window ' ...
     'are not the exact-delay margins from run_pitch_tvc.'], ME1.message);
try
    pidTuner(rational_delay(G, 4), C);
catch ME2
    error('open_pitch_tuner:PidTuner', ...
        'Exact delay failed (%s). Padé approximation also failed (%s).', ...
        ME1.message, ME2.message);
end
end


function open_designer(G, C)
% Root locus cannot draw a pure delay, so some releases reject the plant
% even though the Bode editor could show it. Fall back to Padé.
try
    controlSystemDesigner(G, C);
    return
catch ME1
end
warning('open_pitch_tuner:Pade', ...
    ['Control System Designer did not open the exact-delay plant (%s). ' ...
     'Opening a 4th-order Padé approximation.'], ME1.message);
try
    controlSystemDesigner(rational_delay(G, 4), C);
catch ME2
    error('open_pitch_tuner:Designer', ...
        'Exact delay failed (%s). Padé approximation also failed (%s).', ...
        ME1.message, ME2.message);
end
end
