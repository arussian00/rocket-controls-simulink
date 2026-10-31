function G = grid_fin_model(V)
%GRID_FIN_MODEL  Pitch-axis grid fins for the Super Heavy return.
%
%   G = grid_fin_model(V)
%
%   The companion simulator (matlab-rocket-sim, rocket_params 'superheavy')
%   uses four grid fins as a lateral force during the unpowered fall:
%   max side force = q * P.finCLA, with finCLA = 25 m^2, and only below
%   P.finAlt = 40 km. It does not put a pitching moment on those fins.
%   The numbers below turn that force model into a pitch control surface.
%   They are design assumptions on top of the companion, in the same sense
%   as the 6 Hz gimbal servo. vehicle_params is left alone so the parallel
%   load-relief and slosh work can edit it without colliding.
%
%   Positive fin deflection produces positive pitch acceleration, the same
%   sign as positive gimbal, so one acceleration command feeds both.

if nargin < 1 || isempty(V)
    V = vehicle_params();
end

G.finCLA = 25;                 % companion P.finCLA [m^2], four fins together
G.finPitchFraction = 0.5;      % pitch pair; the other pair is the yaw channel
G.finStaticFraction = 0.25;    % alpha moment per rad / control moment per rad
G.finDeltaMaxDeg = 20;         % deflection stop [deg]
G.finRateDeg = 30;             % deflection rate stop [deg/s]
G.finHz = 2;                   % second-order servo, slower than the 6 Hz gimbal
G.finZeta = 0.7;
G.finStationFromNose = 4;      % hinge line [m]. CG is mid-body, so the arm is ~31.5 m
G.finAlt = 40e3;               % companion P.finAlt [m]
G.alphaCap = 0.5;              % linear fin stiffness is not trusted past this alpha [rad]

G.finDeltaMax = deg2rad(G.finDeltaMaxDeg);
G.finRate = deg2rad(G.finRateDeg);
G.finWn = 2*pi*G.finHz;

% Flip effector. The companion's attitudeControl covers whatever the gimbal
% cannot with RCS, and Super Heavy's flip is above the air (tauRCS = 30e6 N*m,
% "flip takes ~15 s"). Engines-off descent below that is fins only.
G.tauRCS = 30e6;               % companion superheavy [N*m]
G.rcsTau = 0.05;               % first-order thruster lag [s]

% Allocation. Same time constant on the mode weight and on the mu used in
% the divider, so lighting an engine does not spike the gimbal command.
G.blendTau = 0.5;              % [s]
G.muTvcFloor = 0.05;           % below this, inv(mu) is 0 [1/s^2]
G.muFinFloor = 1e-4;
G.authorityMin = 0.05;         % rad/s^2. Below this the phase is not "controlled"

% Command-governor floors for this return. The ascent floors (6 deg, 12.5 deg/s)
% are sized for a 5 deg design step and are reused on burns. On fins alone
% they would drive the surface into the stop, so the floor drops. The flip
% is ~140 deg; a 30 deg lead is what lets the rate loop ask for ~15 deg/s
% while RCS acceleration still saturates a_cmd.
G.flipLeadDeg = 30;
G.flipRateDeg = 16;
G.finLeadFloorDeg = 1;
G.finRateFloorDeg = 2;
G.flipLead = deg2rad(G.flipLeadDeg);
G.flipRate = deg2rad(G.flipRateDeg);
G.finLeadFloor = deg2rad(G.finLeadFloorDeg);
G.finRateFloor = deg2rad(G.finRateFloorDeg);

% Pitch command on the landing burn. Companion maxTiltDeg is 30 and the
% thrust tilt fades over the last 20 m (tiltFadeAlt). That fade is faster
% than the 1 rad/s attitude loop, so the pitch command stands up from
% 400 m above the arms and is upright by 150 m. The 30 deg cap is unchanged.
G.hLand = 65;                  % companion superheavy tower-catch height [m]
G.maxTiltDeg = 30;             % companion P.maxTiltDeg
G.fadeUpright = 150;           % upright by this height above the arms [m]
G.fadeSpan = 250;              % full 30 deg at fadeUpright+fadeSpan [m]
G.landingVhFloor = 40;         % [m/s] keeps atan2 from flipping when vh ~ 0
G.aeroVhFloor = 1;             % [m/s] engines-first trim

G.dt = 0.002;                  % fixed-step return simulation [s]
G.sensorDelay = V.sensorDelay;
G.deltaAvailFrac = V.deltaAvailFrac;
end
