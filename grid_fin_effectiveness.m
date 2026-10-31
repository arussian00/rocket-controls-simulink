function [muFin, lambdaFin, arm] = grid_fin_effectiveness(G, q, inertia, cgFromNose)
%GRID_FIN_EFFECTIVENESS  Pitch acceleration per radian, and the aero stiffness.
%
%   [muFin, lambdaFin, arm] = grid_fin_effectiveness(G, q, inertia, cgFromNose)
%
%   At full deflection the pitch pair can push q * finCLA * finPitchFraction.
%   Dividing by the deflection stop gives the small-signal force per radian.
%   The moment arm is from the trajectory centre of mass (mid-body here) up
%   to the hinge line. lambdaFin is restoring (negative): on the engines-first
%   fall the fins are the tail, not a nose fin.

arm = max(cgFromNose - G.finStationFromNose, 1);
muFin = q .* G.finCLA .* G.finPitchFraction .* arm ./ (G.finDeltaMax .* inertia);
lambdaFin = -G.finStaticFraction .* muFin;
end