function S = flight_margin_schedule(D, V, traj)
%FLIGHT_MARGIN_SCHEDULE  Margins of the pitch loop along the trajectory.
%
%   S = flight_margin_schedule(D, V, traj)
%
%   The scheduled law keeps Kp, Ki, Kd in acceleration units and sets
%   delta_cmd = a_cmd/mu(t). Its margins move only with lambda(t).
%   The fixed-gain law freezes the liftoff gimbal gain (multiplies the
%   loop by mu(t)/mu(0)). Samples are about 4 s apart, and only where
%   thrust-vector control is active. Stability Padé checks are skipped
%   on this grid; the 6 dB / 30 deg floor is the flag.

mu0 = traj.mu_rps2(1);
active = find(traj.tvc_active ~= 0 & traj.mu_rps2 > 1e-3);
if isempty(active)
    error('flight_margin_schedule:NoTVC', 'Trajectory has no powered samples.');
end
keep = active(1);
for i = active(2:end).'
    if traj.t_s(i) - traj.t_s(keep(end)) >= 4
        keep(end + 1) = i; %#ok<AGROW>
    end
end
keep(end + 1) = active(end);
keep = unique(keep, 'stable');

n = numel(keep);
S.t = traj.t_s(keep);
S.phase = traj.phase(keep);
S.q = traj.q_Pa(keep);
S.mu = traj.mu_rps2(keep);
S.lambda = traj.lambda_rps2(keep);
S.pm = nan(n, 1);
S.gm = nan(n, 1);
S.wc = nan(n, 1);
S.pmFixed = nan(n, 1);
S.gmFixed = nan(n, 1);
S.below = false(n, 1);
S.belowFixed = false(n, 1);
S.mu0 = mu0;
S.floorGm = 6;
S.floorPm = 30;

fprintf('Flight-schedule margins at %d powered samples...\n', n);
for k = 1:n
    flight = struct('name', "sched", 'mu', S.mu(k), 'lambda', S.lambda(k));
    P = pitch_plant(flight, V, lambda=S.lambda(k));
    m = loop_margins(D.Ctf * P.G, checkStability=false);
    S.pm(k) = m.pm;
    S.gm(k) = m.gmUp;
    S.wc(k) = m.wc;
    mFix = loop_margins((S.mu(k) / mu0) * D.Ctf * P.G, checkStability=false);
    S.pmFixed(k) = mFix.pm;
    S.gmFixed(k) = mFix.gmUp;
    S.below(k) = ~(m.pm >= S.floorPm && m.gmUp >= S.floorGm);
    S.belowFixed(k) = ~(mFix.pm >= S.floorPm && mFix.gmUp >= S.floorGm);
end

[~, iS] = min(S.gm);
[~, iF] = min(S.gmFixed);
S.worstScheduled = pack_worst(S, iS, false);
S.worstFixed = pack_worst(S, iF, true);
S.nBelow = nnz(S.below);
S.nBelowFixed = nnz(S.belowFixed);

fprintf('  Scheduled worst GM %.1f dB, PM %.1f deg at t = %.0f s (%s). Samples under the floor: %d.\n', ...
    S.worstScheduled.gm, S.worstScheduled.pm, S.worstScheduled.t, ...
    S.worstScheduled.phase, S.nBelow);
fprintf('  Fixed liftoff gain worst GM %.1f dB, PM %.1f deg at t = %.0f s (%s). Samples under the floor: %d.\n', ...
    S.worstFixed.gm, S.worstFixed.pm, S.worstFixed.t, ...
    S.worstFixed.phase, S.nBelowFixed);
end


function w = pack_worst(S, idx, fixed)
w.t = S.t(idx);
w.phase = S.phase(idx);
w.mu = S.mu(idx);
w.q = S.q(idx);
if fixed
    w.pm = S.pmFixed(idx);
    w.gm = S.gmFixed(idx);
else
    w.pm = S.pm(idx);
    w.gm = S.gm(idx);
end
end
