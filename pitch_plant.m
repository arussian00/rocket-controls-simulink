function P = pitch_plant(flight, V, opts)
%PITCH_PLANT  Linear pitch plant from acceleration command to pitch.
%
%   P = pitch_plant(flight, V)
%   P = pitch_plant(flight, V, delay=0.05, actHz=3, lambda=0.1)
%
%   Perturbation model about a trim with zero angle of attack and zero
%   gimbal, pitch measured from the trim attitude (the flight-path angle):
%
%       theta'' = lambda*theta + a_delivered + d
%       a_delivered = Actuator(s) * a_cmd
%       delta_cmd   = a_cmd / mu          (the gain schedule)
%
%   lambda > 0 is aerodynamically unstable. mu = F*arm/I is the gimbal
%   effectiveness [rad/s^2 per rad]. The sensor delay sits on the feedback,
%   which for loop gain is the same as an input delay on P.G.
%
%   P.G  is the plant PID Tuner and Control System Designer should see:
%   a_cmd [rad/s^2] -> theta [rad], actuator and delay included.
%   Requires Control System Toolbox.

arguments
    flight (1,1) struct
    V (1,1) struct
    opts.delay (1,1) double = V.sensorDelay
    opts.actHz (1,1) double = V.actHz
    opts.actZeta (1,1) double = V.actZeta
    opts.lambda (1,1) double = flight.lambda
end

require_control_toolbox();

actWn = 2*pi*opts.actHz;
P.actuator = tf(actWn^2, [1, 2*opts.actZeta*actWn, actWn^2]);
P.rigid = tf(1, [1, 0, -opts.lambda]);          % a_delivered -> theta
P.G = P.actuator * P.rigid;
P.G.InputDelay = opts.delay;
P.G.InputName = 'a_cmd';
P.G.OutputName = 'theta';
P.G.Name = "pitch from a_cmd, " + flight.name;

P.mu = flight.mu;
P.lambda = opts.lambda;
P.delay = opts.delay;
P.actHz = opts.actHz;
P.actZeta = opts.actZeta;
P.flightName = flight.name;
end


function require_control_toolbox()
if ~license('test', 'Control_Toolbox')
    error('pitch_plant:NeedsCST', ...
        ['Control System Toolbox is required for tf, pid, margin and step. ' ...
         'The Simulink model itself uses only base Simulink blocks.']);
end
end
