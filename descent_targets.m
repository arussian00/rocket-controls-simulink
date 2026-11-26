function w = descent_targets(phaseCode, muTvc, muFin, aFin, aRcs, ownLo, ownHi)
%DESCENT_TARGETS  Steady allocation [tvc; fin; rcs] for one return sample.
%
%   w = descent_targets(phaseCode, muTvc, muFin)
%   w = descent_targets(phaseCode, muTvc, muFin, aFin, aRcs, ownLo, ownHi)
%
%   phaseCode: 1 flip, 2 boostback, 3 coast, 4 entry, 5 brake, 6 landing.
%   Flip is RCS (above the air). Boostback is TVC on the inner 13.
%   Coast and entry blend RCS into the fins by the acceleration the fin
%   stop can produce (aFin). Below ownLo that acceleration is not a pitch
%   effector and RCS tracks the trim; above ownHi the fins take the
%   channel. The two call forms without aFin keep the older fins-only
%   coast. The landing burn blends TVC and fins by mu.

if nargin < 5 || isempty(aRcs)
    aFin = 0;
    aRcs = 0;
    legacyCoast = true;
else
    legacyCoast = false;
end
if nargin < 6 || isempty(ownLo)
    ownLo = 0.05;
end
if nargin < 7 || isempty(ownHi)
    ownHi = 0.25;
end

switch phaseCode
    case 1
        w = [0; 0; 1];
    case 2
        w = [1; 0; 0];
    case {3, 4}
        if legacyCoast
            w = [0; 1; 0];
        else
            w = coast_blend(aFin, aRcs, ownLo, ownHi);
        end
    otherwise
        mt = max(muTvc, 0);
        mf = max(muFin, 0);
        total = mt + mf;
        if total < 1e-8
            w = [0; 0; 0];
        else
            w = [mt; mf; 0] / total;
        end
end
end


function w = coast_blend(aFin, aRcs, ownLo, ownHi)
% s = 0 is RCS, s = 1 is fins. A zero of both leaves the channel off.
aF = max(aFin, 0);
aR = max(aRcs, 0);
if aF + aR < 1e-8
    w = [0; 0; 0];
    return
end
span = max(ownHi - ownLo, 1e-6);
s = (aF - ownLo) / span;
s = min(1, max(0, s));
w = [0; s; 1 - s];
end