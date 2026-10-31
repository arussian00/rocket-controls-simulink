function w = descent_targets(phaseCode, muTvc, muFin)
%DESCENT_TARGETS  Steady allocation [tvc; fin; rcs] for one return sample.
%
%   w = descent_targets(phaseCode, muTvc, muFin)
%
%   phaseCode: 1 flip, 2 boostback, 3 coast, 4 entry, 5 brake, 6 landing.
%   Flip is RCS (above the air). Boostback is TVC on the inner 13.
%   Coast and the grid-fin entry are fins only. The landing burn blends
%   TVC and fins by the pitch acceleration each can produce.

switch phaseCode
    case 1
        w = [0; 0; 1];
    case 2
        w = [1; 0; 0];
    case {3, 4}
        w = [0; 1; 0];
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