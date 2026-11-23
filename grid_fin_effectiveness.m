function [muFin, lambdaFin, arm] = grid_fin_effectiveness(G, q, inertia, cgFromNose)
%GRID_FIN_EFFECTIVENESS  Pitch acceleration per radian, and the aero stiffness.
%
%   [muFin, lambdaFin, arm] = grid_fin_effectiveness(G, q, inertia, cgFromNose)
%
%   At full deflection the pitch pair can push q * finCLA * finPitchFraction.
%   Dividing by the deflection stop gives the small-signal force per radian.
%   The moment arm is from the tank-model centre of mass up to the hinge
%   line. lambdaFin is restoring (negative): on the engines-first fall the
%   fins are the tail, not a nose fin. It is added to the body lambda from
%   vehicle_mass_properties; it does not replace that stiffness.

arm = max(cgFromNose - G.finStationFromNose, 1);
muFin = q .* G.finCLA .* G.finPitchFraction .* arm ./ (G.finDeltaMax .* inertia);
lambdaFin = -G.finStaticFraction .* muFin;
end