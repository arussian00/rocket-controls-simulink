function apply_descent_workspace(S, D, V, ret)
%APPLY_DESCENT_WORKSPACE  Base-workspace names for boosterDescentPitch.
%
%   apply_descent_workspace(S, D, V, ret)
%
%   Lookup breakpoints are a row vector. Tables are clipped outside the
%   grid by the block (ExtrapMethod Clip). Names are prefixed with d so a
%   descent sim does not overwrite the ascent pitch-model variables.

G = ret.G;
[t, ia] = unique(S.t(:), 'stable');
t = t(:).';
row = @(x) reshape(x(ia), 1, []);

assignin('base', 'dTStart', t(1));
assignin('base', 'dTEnd', t(end));
assignin('base', 'dTheta0', ret.theta0);
assignin('base', 'dTime', t);

assignin('base', 'dKq', D.Kq);
assignin('base', 'dKth', D.Kth);
assignin('base', 'dKiTh', D.KiTh);
assignin('base', 'dKaw', D.kaw);
assignin('base', 'dKawA', D.kaw / max(D.Kq, 1e-9));
assignin('base', 'dCmdTau', D.cmdTau);
assignin('base', 'dTf', D.Tf);
assignin('base', 'dTrimTau', D.trimTau);
assignin('base', 'dShapeKs', D.shapeKs);
assignin('base', 'dShapeE', D.shapeE);
assignin('base', 'dShapeQ', D.shapeQ);
assignin('base', 'dDeltaMax', D.deltaMax);
assignin('base', 'dRateMax', V.gimbalRate);
assignin('base', 'dDeltaAvailFloor', G.deltaAvailFrac * D.deltaMax);
assignin('base', 'dActWn', V.actWn);
assignin('base', 'dActZeta', V.actZeta);
assignin('base', 'dFinMax', G.finDeltaMax);
assignin('base', 'dFinRate', G.finRate);
assignin('base', 'dFinWn', G.finWn);
assignin('base', 'dFinZeta', G.finZeta);
assignin('base', 'dFinAvailFloor', G.deltaAvailFrac * G.finDeltaMax);
assignin('base', 'dRcsTau', G.rcsTau);
assignin('base', 'dRcsSlew', 50);
assignin('base', 'dAlphaCap', G.alphaCap);
assignin('base', 'dSensorDelay', V.sensorDelay);

assignin('base', 'dThetaCmd', row(S.thetaCmd));
assignin('base', 'dThetaTrim', row(S.thetaTrim));
assignin('base', 'dMuTvc', row(S.muTvc));
assignin('base', 'dMuFin', row(S.muFin));
assignin('base', 'dLambda', row(S.lambdaFin));
assignin('base', 'dWTvc', row(S.wTvc));
assignin('base', 'dWFin', row(S.wFin));
assignin('base', 'dWRcs', row(S.wRcs));
assignin('base', 'dInvTvc', row(S.invTvc));
assignin('base', 'dInvFin', row(S.invFin));
assignin('base', 'dAMax', row(S.aMax));
assignin('base', 'dEFloor', row(S.eFloor));
assignin('base', 'dQFloor', row(S.qFloor));
assignin('base', 'dGateT', row(S.gateT));
assignin('base', 'dGateF', row(S.gateF));
assignin('base', 'dGateR', row(S.gateR));
assignin('base', 'dARcs', row(S.aRcs));
end