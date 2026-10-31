function [Tcmd, Tdist] = notched_command(D, flight, V)
%NOTCHED_COMMAND  Rigid command and disturbance responses with the sensor notch.
%
%   [Tcmd, Tdist] = notched_command(D, flight, V)
%
%   Same assembly as loop_transfers in design_pitch_controller, with the
%   notch on the measured pitch and rate. The prefilter stays outside the
%   loop. The Simulink diagram uses the same notch, so the dashed overlay
%   should track the solid curve.
P = pitch_plant(flight, V);
N = notch_tf(flight.bendWn);
Cpid = D.Ctf;
Cpi = tf(D.Kp) + tf(D.Ki, [1 0]);
DtermOverCpi = tf([D.Kd, 0, 0], conv([D.Tf, 1], [D.Kp, D.Ki]));
Del = tf(1, 1, 'InputDelay', P.delay);
Fp = tf(1, [D.cmdTau 1]);
M = P.rigid * P.actuator * Cpi;
H = N * Del * (1 + DtermOverCpi);
Tcmd = Fp * feedback(M, H);
Tdist = feedback(P.rigid, Cpid * P.actuator * N * Del);
Tcmd.Name = "theta/cmd notched " + flight.name;
Tdist.Name = "theta/d notched " + flight.name;
end
