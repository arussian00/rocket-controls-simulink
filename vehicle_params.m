function V = vehicle_params()
%VEHICLE_PARAMS  Numbers for the Super Heavy pitch-TVC study.
%
%   V = vehicle_params()
%
%   Vehicle data match matlab-rocket-sim (rocket_params.m and
%   rocket_starship_sim.m) so the two projects stay consistent:
%
%     https://github.com/arussian00/matlab-rocket-sim
%
%   The companion integrates the stack as a point mass and only gives the
%   booster a rigid-body pitch model after separation (uniform rod,
%   I = m*L^2/12, gimbal arm = L/2, instantaneous gimbal). This file keeps
%   that inertia model, extends it to the full stack for liftoff and max-q,
%   and adds the actuator, delay and aerodynamic moment the companion leaves
%   out. Those additions are named below as design assumptions.
%
%   V.flight is the three frozen conditions the controller is scheduled on.
%   Edit the knobs in the "design assumptions" section, then re-run
%   design_pitch_controller or run_pitch_tvc.

V.g0   = 9.80665;   % standard gravity [m/s^2]          (rocket_params)
V.Re   = 6371e3;    % mean Earth radius [m]
V.rho0 = 1.225;     % sea-level density [kg/m^3]
V.Hscale = 8500;    % exponential scale height [m]     (not used for q)

%% Companion vehicles (rocket_params.m). Active study uses Super Heavy.
V.library.hop = struct( ...
    'name', "hop", 'mDry', 6000, 'mProp', 19000, ...
    'Teng', 400e3, 'thrMin', 0.30, 'diam', 3.5, 'L', 40, ...
    'gimbalMaxDeg', 6, 'attWn', 1.0, 'attZeta', 0.8, ...
    'engAscent', 1, 'engLand', 1);
V.library.booster = struct( ...
    'name', "booster", 'mDry', 14000, 'mProp', 130000, ...
    'Teng', 2.6e6/9, 'thrMin', 0.40, 'diam', 3.7, 'L', 45, ...
    'gimbalMaxDeg', 6, 'attWn', 1.0, 'attZeta', 0.8, ...
    'engAscent', 9, 'engLand', 1);
V.library.superheavy = struct( ...
    'name', "superheavy", 'mDry', 230e3, 'mProp', 3400e3, ...
    'Teng', 2.3e6, 'thrMin', 0.40, 'diam', 9, 'L', 71, ...
    'gimbalMaxDeg', 15, 'attWn', 1.0, 'attZeta', 0.8, ...
    'engAscent', 33, 'engLand', 3, 'engBrake', 13, ...
    'IspSL', 327, 'IspVac', 350);
V.library.starship = struct( ...
    'name', "starship", 'mDry', 120e3, 'mProp', 1500e3, ...
    'mPay', 10e3, 'Teng', 2.3e6, 'thrMin', 0.40, 'diam', 9, 'L', 50, ...
    'gimbalMaxDeg', 15, 'attWn', 1.5, 'attZeta', 0.8, 'engLand', 2);

B = V.library.superheavy;
S = V.library.starship;
V.vehicleName = B.name;

%% Stack geometry for the ascent pitch model
% The companion stores the stages separately and does not simulate stack
% attitude. Length is the sum of the two rigid lengths (71 m + 50 m).
V.stackLength = B.L + S.L;
V.diam = B.diam;
V.area = pi * (B.diam/2)^2;          % frontal area, companion P.A [m^2]
V.Lbooster = B.L;
V.mDryBooster = B.mDry;
V.Teng = B.Teng;
V.thrMin = B.thrMin;
V.gimbalMaxDeg = B.gimbalMaxDeg;
V.gimbalMax = deg2rad(B.gimbalMaxDeg);

% Fraction of thrust that is gimballed. 1 matches attitudeControl in the
% companion, which applies the whole thrust at one equivalent nozzle angle.
V.gimbalThrustFraction = 1.0;

%% Design assumptions the companion does not have
% Actuator: second order, plus the angle limit from the companion and a
% rate limit. 6 Hz and 20 deg/s are sizing assumptions for a large-engine
% hydraulic TVC, not identified hardware data.
V.actHz = 6;
V.actWn = 2*pi*V.actHz;
V.actZeta = 0.7;
V.gimbalRateDeg = 20;                         % [deg/s]
V.gimbalRate = deg2rad(V.gimbalRateDeg);      % [rad/s]
V.sensorDelay = 0.020;                        % IMU + flight computer [s]

% Aerodynamic normal force, slender-body slope on the frontal area.
% Static margin in calibers (diameters): positive is stable (CP behind CG).
% Ascent uses a mild unstable margin. Landing uses 0 because this model has
% no grid-fin moment yet (the companion body aero moment is also zero).
V.CNa = 2.0;                                  % [1/rad]
V.staticMarginAscent = -0.5;                  % [calibers]
V.staticMarginLanding = 0.0;

% Cascade: inner rate gain Kq, outer attitude-to-rate gain Kth.
% Equivalent acceleration PID is Kp = Kq*Kth, Kd = Kq, Ki = Kq*KiTh.
% Kq = 2 gives a critically damped wn = 1 loop (zeta = 1) with about
% 64 deg / 19 dB, still above the 30 deg / 6 dB floor on the scheduled flight.
% attZeta stays at the companion's 0.8. It is printed as that target.
% It does not set Kq.
V.attWn = B.attWn;
V.attZeta = B.attZeta;
V.attKiOverWn3 = 0.02;
V.Kq = 2.0;
V.attTf = 0.05;                               % rate filter [s]
V.kaw = 1.0;                                  % back-calculation [1/s]

% Command prefilter, outside the feedback loop. The authority governor
% (etaAuth) then rate-limits and lead-limits the command using the gimbal
% angle still available after trim. A floor keeps the 5 deg design step
% off that limiter so the linear overlay still applies.
V.cmdTau = 0.5;
V.etaAuth = 0.85;
V.shapeKs = 20;                               % rate-limiter tracking [1/s]
V.trimTau = 3;                                % trim-gimbal filter [s]
V.deltaAvailFrac = 0.25;                      % minimum fraction of the stop kept free

% Scenario used by run_pitch_tvc.
V.thetaStepDeg = 5;
V.thetaStepTime = 1;
V.gustSpeed = 15;                             % crosswind step [m/s]
V.gustTime = 14;
V.tStop = 22;
V.largeStepDeg = 30;                          % big enough to saturate an ungoverned command
V.largeStepStop = 45;                         % governed 30 deg step needs the extra time

% Load relief. 0 holds the trajectory attitude. 1 commands the attitude
% that zeros angle of attack. The dynamic-pressure schedule multiplies the
% knob: full authority once q reaches qEngage, and fading below that, so
% the blend is off at liftoff and on through the max-q bucket.
V.lrBlend = 0.5;
V.qEngage = 12e3;                             % [Pa]
V.lrVfloor = 30;                              % [m/s] keeps wind/V finite on the pad
% Crosswind through max-q. The shear reaches 12 m/s at T+46 s and holds.
% The 1-cosine gust peaks at 9 m/s at T+46 s (starts at 42 s, lasts 8 s).
V.windShear = 12;                             % [m/s]
V.windShearTime = 46;                         % [s]
V.windGust = 9;                               % [m/s] peak of the 1-cosine
V.windGustStart = 42;                         % [s]
V.windGustDur = 8;                            % [s]
V.loadReliefStop = 110;                       % [s] through the post-max-q fade

%% Flight conditions
% max-q mass, speed and altitude are a snapshot of the companion stack
% ascent (starshipStack: 33 engines, 75% throttle bucket through Mach 1,
% gravity turn). RK4 at 0.05 s lands on the companion's reported max-q,
% about 22 kPa near T+46 s. q below is recomputed from that altitude and
% speed with the US76 troposphere.
mLiftoff = B.mDry + B.mProp + S.mDry + S.mProp + S.mPay;

V.flight = [ ...
    make_flight(V, B, "liftoff", struct( ...
        'time', 0, 'altitude', 0, 'speed', 0, 'mass', mLiftoff, ...
        'throttle', 1.0, 'nEng', B.engAscent, 'length', V.stackLength, ...
        'staticMarginCal', V.staticMarginAscent, ...
        'note', "Full stack, 33 engines, vertical, q = 0")), ...
    make_flight(V, B, "max-q", struct( ...
        'time', 46, 'altitude', 5880, 'speed', 258.6, 'mass', 4224.4e3, ...
        'throttle', 0.75, 'nEng', B.engAscent, 'length', V.stackLength, ...
        'staticMarginCal', V.staticMarginAscent, ...
        'note', "Throttle bucket, mild aero instability")), ...
    make_flight(V, B, "landing", struct( ...
        'time', NaN, 'altitude', 1000, 'speed', 80, ...
        'mass', B.mDry + 55e3, ...
        'throttle', 0.70, 'nEng', B.engLand, 'length', B.L, ...
        'staticMarginCal', V.staticMarginLanding, ...
        'note', "3 centre engines, ~55 t propellant, body margin 0")) ];

% qEngage is the schedule. qPeak is the ascent max-q condition, for the report.
mq = find([V.flight.name] == "max-q", 1);
V.qPeak = V.flight(mq).q;
end


function flight = make_flight(V, B, name, in)
% Build one frozen condition and the rigid-body pitch coefficients.
flight = in;
flight.name = name;
flight.thrust = in.throttle * in.nEng * B.Teng;          % [N]
flight.weight = in.mass * V.g0;
flight.thrustToWeight = flight.thrust / flight.weight;

flight.inertia = in.mass * in.length^2 / 12;             % uniform rod [kg*m^2]
flight.arm = in.length / 2;                              % CG at mid-length [m]
flight.mu = flight.thrust * flight.arm * V.gimbalThrustFraction ...
    / flight.inertia;                                    % [rad/s^2 per rad]

[flight.density, flight.q] = dynamic_pressure(in.altitude, in.speed);
% lcp > 0 means the CP is ahead of the CG (unstable), in metres.
flight.lcp = -in.staticMarginCal * V.diam;
flight.aeroMomentPerAlpha = flight.q * V.CNa * V.area * flight.lcp;  % [N*m/rad]
flight.lambda = flight.aeroMomentPerAlpha / flight.inertia;          % [1/s^2]
if flight.lambda > 0
    flight.unstablePole = sqrt(flight.lambda);           % [rad/s]
else
    flight.unstablePole = 0;
end

% Crosswind contribution to angular acceleration: lambda*(theta - w/V).
% At rest there is no aero, so the gain is zero rather than 1/V.
if in.speed > 1 && flight.lambda ~= 0
    flight.gustAccPerWind = -flight.lambda / in.speed;   % [rad/s^2 per (m/s)]
else
    flight.gustAccPerWind = 0;
end
end


function [rho, q] = dynamic_pressure(altitude, speed)
% US76 troposphere (0-11 km). Enough for liftoff, max-q and the landing point.
% Geopotential correction matches earthModel.m in the companion.
R = 287.053;
g0 = 9.80665;
r0 = 6356766;
L = -0.0065;
T0 = 288.15;
p0 = 101325;
h = max(altitude, 0);
if h > 11000
    error('vehicle_params:Atmosphere', ...
        'This helper only covers the troposphere (h = %.0f m).', h);
end
H = r0 * h / (r0 + h);
T = T0 + L * H;
p = p0 * (T / T0)^(-g0 / (R * L));
rho = p / (R * T);
q = 0.5 * rho * speed^2;
end
