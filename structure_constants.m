function c = structure_constants()
%STRUCTURE_CONSTANTS  Shared numbers for the mass, slosh, bending and notch models.
%
%   Dry mass, propellant load, length and diameter stay in vehicle_params
%   (the rocket_params values). The numbers here are the layout assumptions
%   matlab-rocket-sim does not carry: mixture ratio, liquid densities, a
%   Raptor mass used only to place the dry mass, slosh and bending
%   damping, and the notch.

c.of = 3.6;
c.rhoLox = 1141.0;
c.rhoCh4 = 422.6;
c.mEngine = 1600.0;          % kg, a slice of dry mass, not an added mass
c.gimbalInset = 1.5;         % m, gimbal plane forward of the aft skin

c.sloshZeta = 0.05;          % ring-baffle damping, light
c.sloshLambda1 = 1.8411837813406593;
c.sloshMinH = 0.05;          % m, below this the pendulum model is dropped
c.sloshNames = ["ship CH4", "ship LOX", "booster CH4", "booster LOX"];

c.f1RefHz = 3.5;             % first stack mode at liftoff mass and length
c.bendZeta = 0.01;
c.imuFrac = 0.08;            % IMU station, fraction of length from the nose
c.nBend = 2;

c.notchZetaZ = 0.015;
c.notchZetaP = 0.40;
end
