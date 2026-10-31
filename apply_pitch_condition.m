function apply_pitch_condition(D, name, opts)
%APPLY_PITCH_CONDITION  Push one flight condition into the base workspace.
%
%   The Simulink model reads Kp, mu, lambdaAero, gustAccPerWind and the
%   scenario times from the base workspace. Call this before sim, or let
%   pitch_model_init do it for a manual Run of the liftoff case.
%
%   apply_pitch_condition(D, "max-q")
%   apply_pitch_condition(D, "liftoff", thetaStepDeg=30, gustSpeed=0, tStop=45, shapeOn=0)

arguments
    D (1,1) struct
    name (1,1) string
    opts.thetaStepDeg (1,1) double = NaN
    opts.thetaStepTime (1,1) double = NaN
    opts.gustSpeed (1,1) double = NaN
    opts.gustTime (1,1) double = NaN
    opts.misalignDeg (1,1) double = 0
    opts.misalignTime (1,1) double = 12
    opts.tStop (1,1) double = NaN
    opts.shapeOn (1,1) double = 1
end

flight = find_flight(D, name);
if isnan(opts.thetaStepDeg),  opts.thetaStepDeg  = D.thetaStepDeg;  end
if isnan(opts.thetaStepTime), opts.thetaStepTime = D.thetaStepTime; end
if isnan(opts.gustSpeed),     opts.gustSpeed     = D.gustSpeed;     end
if isnan(opts.gustTime),      opts.gustTime      = D.gustTime;      end
if isnan(opts.tStop),         opts.tStop         = D.tStop;         end

assign('tStop', opts.tStop);
assign('thetaStep', deg2rad(opts.thetaStepDeg));
assign('thetaStepTime', opts.thetaStepTime);
assign('cmdTau', D.cmdTau);
assign('sensorDelay', D.sensorDelay);

assign('Kp', D.Kp);
assign('Ki', D.Ki);
assign('Kd', D.Kd);
assign('Kq', D.Kq);
assign('Kth', D.Kth);
assign('KiTh', D.KiTh);
assign('Tf', D.Tf);
assign('Kaw', D.kaw);
assign('KawA', D.kaw / max(D.Kq, 1e-9));
assign('shapeOn', opts.shapeOn);
assign('shapeKs', D.shapeKs);
assign('shapeQ', D.shapeQ);
assign('shapeE', D.shapeE);
assign('qLimFloor', D.qLimFloor);
assign('eLimFloor', D.eLimFloor);
assign('trimTau', D.trimTau);
assign('deltaAvailFloor', D.deltaAvailFloor);

assign('actWn', D.actWn);
assign('actZeta', D.actZeta);
assign('gimbalRate', D.gimbalRate);
assign('deltaMax', D.deltaMax);

assign('mu', flight.mu);
assign('lambdaAero', flight.lambda);
assign('aCmdMax', flight.mu * D.deltaMax);
% Frozen-condition lookups: the Simulink plant reads these, not the scalars.
assign('schedTime', [0 1e6]);
assign('schedMu', flight.mu * [1 1]);
assign('schedLambda', flight.lambda * [1 1]);
assign('schedInvMu', (1 / flight.mu) * [1 1]);
assign('schedACmdMax', (flight.mu * D.deltaMax) * [1 1]);

assign('gustTime', opts.gustTime);
assign('gustSpeed', opts.gustSpeed);
assign('gustAccPerWind', flight.gustAccPerWind);

assign('misalignTime', opts.misalignTime);
assign('misRad', deg2rad(opts.misalignDeg));

assign('pitchConditionName', char(name));
end


function flight = find_flight(D, name)
names = [D.flight.name];
idx = find(names == name, 1);
if isempty(idx)
    error('apply_pitch_condition:Unknown', ...
        'No flight condition named "%s".', name);
end
flight = D.flight(idx);
end


function assign(varName, value)
assignin('base', varName, value);
end
