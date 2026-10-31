function D = structure_flight(D, V, traj)
%STRUCTURE_FLIGHT  Margins of the pitch loop with slosh, bending and the notch.
%
%   D = structure_flight(D, V, traj)
%
%   The scheduled acceleration gains are unchanged. At each powered sample
%   the notch is retuned to that instant's bending frequencies (the
%   Simulink runs are frozen conditions, so each of those gets one notch).
%   Margins use loop_margins: exact delay for the margins, Padé only for
%   the stability flag, and that flag is computed at the three design
%   points and at the worst samples rather than on every grid point.
%
%   Bending-mode gain margin is the least amount by which |L| sits below
%   0 dB in a factor-of-two band around the mode. That catches the residual
%   peak and the notch shoulders, not the bottom of the notch.
%
%   D.structure holds the histories the dashboard plots. D.flight(k) gains
%   TcmdNotch and TdistNotch so the Simulink overlay includes the notch.

S = empty_result();
c = structure_constants();
S.notchZetaZ = c.notchZetaZ;
S.notchZetaP = c.notchZetaP;
S.notchDepthDb = 20 * log10(c.notchZetaZ / c.notchZetaP);
S.f1RefHz = c.f1RefHz;
S.sloshZeta = c.sloshZeta;

check_rigid_reduction(D, V);

n = height(traj);
S.t = traj.t_s(:);
S.phase = traj.phase(:);
S.config = traj.config(:);
S.tvc = traj.tvc_active(:) ~= 0;
S.cg = zeros(n, 1);
S.arm = zeros(n, 1);
S.inertia = zeros(n, 1);
S.mu = zeros(n, 1);
S.lambda = zeros(n, 1);
S.sm = zeros(n, 1);
S.fSlosh = nan(n, numel(c.sloshNames));
S.fBend = nan(n, c.nBend);
S.sloshNames = cellstr(c.sloshNames);
for i = 1:n
    mp = mass_at(traj, i, V);
    S.cg(i) = mp.cg;
    S.arm(i) = mp.arm;
    S.inertia(i) = mp.I;
    S.mu(i) = mp.mu;
    S.lambda(i) = mp.lambda;
    S.sm(i) = mp.staticMarginCal;
    S.fSlosh(i, :) = slosh_row(slosh_model(mp));
    bd = bending_model(mp, V);
    S.fBend(i, :) = [bd.hz];
end
stage = find(S.config(1:end-1) ~= S.config(2:end), 1, 'first');
if isempty(stage)
    S.tStage = NaN;
else
    S.tStage = S.t(stage + 1);
end

for k = 1:numel(D.flight)
    [Tcmd, Tdist] = notched_command(D, D.flight(k), V);
    D.flight(k).TcmdNotch = Tcmd;
    D.flight(k).TdistNotch = Tdist;
    S.points(k) = point_margins(D, V, D.flight(k).massProps, D.flight(k).name, true); %#ok<AGROW>
end

keep = margin_samples(traj);
fprintf('Structure margins (slosh + bending + notch) at %d powered samples...\n', numel(keep));
mN = numel(keep);
S.mIdx = keep;
S.mt = traj.t_s(keep);
S.mPhase = traj.phase(keep);
S.pm = nan(mN, 1);
S.gm = nan(mN, 1);
S.wc = nan(mN, 1);
S.pmBare = nan(mN, 1);
S.gmBare = nan(mN, 1);
S.bendGm = nan(mN, 2);
S.bendGmBare = nan(mN, 2);
S.mLambda = nan(mN, 1);
for k = 1:mN
    mp = mass_at(traj, keep(k), V);
    row = point_margins(D, V, mp, "sched", false);
    S.pm(k) = row.pm;
    S.gm(k) = row.gm;
    S.wc(k) = row.wc;
    S.pmBare(k) = row.pmBare;
    S.gmBare(k) = row.gmBare;
    S.bendGm(k, :) = row.bendGm;
    S.bendGmBare(k, :) = row.bendGmBare;
    S.mLambda(k) = mp.lambda;
end

S.worstPm = worst_of(S, S.pm, true);
S.worstGm = worst_of(S, S.gm, true);
S.worstBend = worst_of(S, S.bendGm(:, 1), true);
S.worstPm.stable = stable_at(D, V, traj, S.worstPm.idx);
S.worstGm.stable = stable_at(D, V, traj, S.worstGm.idx);
S.bode = liftoff_bode(D, V);

D.structure = S;
print_structure(S);
end


function row = point_margins(D, V, mp, name, withStability)
slosh = slosh_model(mp);
bend = bending_model(mp, V);
P = flex_pitch_plant(mp, slosh, bend, V);
N = notch_tf([bend.wn]);
L = D.Ctf * N * P.sys;
Lbare = D.Ctf * P.sys;
m = loop_margins(L, checkStability=withStability);
mb = loop_margins(Lbare, checkStability=false);
row.name = string(name);
row.cg = mp.cg;
row.arm = mp.arm;
row.inertia = mp.I;
row.mu = mp.mu;
row.lambda = mp.lambda;
row.sm = mp.staticMarginCal;
row.ax = mp.ax;
row.fSlosh = slosh_row(slosh);
row.fBend = [bend.hz];
row.wnBend = [bend.wn];
row.pm = m.pm;
row.gm = m.gmUp;
row.wc = m.wc;
row.stable = m.stable;
row.pmBare = mb.pm;
row.gmBare = mb.gmUp;
row.bendGm = [bending_gain_margin(L, bend(1).wn), bending_gain_margin(L, bend(2).wn)];
row.bendGmBare = [bending_gain_margin(Lbare, bend(1).wn), bending_gain_margin(Lbare, bend(2).wn)];
row.slosh = summarize_slosh(slosh);
end


function gm = bending_gain_margin(L, wn)
% -max|L| in [wn/2, 2*wn], in dB. Positive means the whole band is below 0 dB.
if ~(isfinite(wn) && wn > 0)
    gm = NaN;
    return
end
w = wn * logspace(log10(0.5), log10(2), 400).';
try
    h = freqresp(L, w);
catch
    h = freqresp(rational_delay(L, 6), w);
end
h = abs(squeeze(h));
h = h(:);
peakDb = 20 * log10(max(h));
gm = -peakDb;
end


function bode = liftoff_bode(D, V)
mp = D.flight(1).massProps;
slosh = slosh_model(mp);
bend = bending_model(mp, V);
P = flex_pitch_plant(mp, slosh, bend, V);
N = notch_tf([bend.wn]);
L = D.Ctf * N * P.sys;
Lbare = D.Ctf * P.sys;
w = logspace(log10(0.05), log10(200), 700).';
bode.w = w;
bode.magWith = mag_db(L, w);
bode.magWithout = mag_db(Lbare, w);
bode.name = "liftoff";
bode.fBend = [bend.hz];
bode.wnBend = [bend.wn];
end


function mag = mag_db(L, w)
try
    h = freqresp(L, w);
catch
    h = freqresp(rational_delay(L, 6), w);
end
mag = 20 * log10(abs(squeeze(h)));
mag = mag(:);
end


function stable = stable_at(D, V, traj, idx)
if isempty(idx) || ~isfinite(idx)
    stable = false;
    return
end
mp = mass_at(traj, idx, V);
row = point_margins(D, V, mp, "worst", true);
stable = row.stable;
end


function w = worst_of(S, values, smallest)
values = values(:);
if smallest
    [~, k] = min(values);
else
    [~, k] = max(values);
end
w.k = k;
w.idx = S.mIdx(k);
w.t = S.mt(k);
w.phase = S.mPhase(k);
w.value = values(k);
w.pm = S.pm(k);
w.gm = S.gm(k);
w.bendGm = S.bendGm(k, 1);
w.lambda = S.mLambda(k);
end


function keep = margin_samples(traj)
active = find(traj.tvc_active ~= 0 & traj.mu_rps2 > 1e-3);
if isempty(active)
    error('structure_flight:NoTVC', 'Trajectory has no powered samples.');
end
keep = active(1);
for i = active(2:end).'
    if strcmp(traj.phase(i), "brake")
        dt = 1;
    else
        dt = 2;
    end
    if traj.t_s(i) - traj.t_s(keep(end)) >= dt - 1e-9
        keep(end + 1) = i; %#ok<AGROW>
    end
end
keep(end + 1) = active(end); %#ok<AGROW>
keep = unique(keep, 'stable');
end


function mp = mass_at(traj, idx, V)
cfg = traj.config(idx);
if strcmp(cfg, "stack")
    refSm = V.staticMarginAscent;
else
    refSm = V.staticMarginLanding;
end
mp = vehicle_mass_properties(traj.mass_kg(idx), traj.thrust_N(idx), ...
    traj.q_Pa(idx), cfg, V, refSm);
end


function hz = slosh_row(modes)
names = structure_constants().sloshNames;
hz = nan(1, numel(names));
for i = 1:numel(modes)
    k = find(names == modes(i).name, 1);
    if ~isempty(k)
        hz(k) = modes(i).hz;
    end
end
end


function text = summarize_slosh(modes)
if isempty(modes)
    text = "none";
    return
end
bits = strings(1, numel(modes));
for i = 1:numel(modes)
    s = modes(i);
    bits(i) = sprintf('%s %.2f Hz, %.0f t, xB %+.1f m', ...
        s.name, s.hz, s.ms / 1e3, s.xB);
end
text = strjoin(bits, '; ');
end


function check_rigid_reduction(D, V)
% Empty slosh and bending must reproduce pitch_plant. A mismatch means the
% sign convention or the mu cancellation drifted.
mp = D.flight(1).massProps;
P = flex_pitch_plant(mp, [], [], V);
m = loop_margins(D.Ctf * P.sys, checkStability=false);
ref = D.flight(1).margins;
if ~(isfinite(m.pm) && isfinite(ref.pm) ...
        && abs(m.pm - ref.pm) < 1.5 && abs(m.gmUp - ref.gmUp) < 1.5)
    warning('structure_flight:RigidMismatch', ...
        ['Flex plant with slosh and bending removed gives PM %.2f deg / GM %.2f dB. ' ...
         'pitch_plant gives PM %.2f deg / GM %.2f dB.'], ...
        m.pm, m.gmUp, ref.pm, ref.gmUp);
end
end


function S = empty_result()
S = struct();
end


function word = yesno(flag)
if flag
    word = 'yes';
else
    word = 'no';
end
end


function print_structure(S)
fprintf('\nStructure, slosh and bending (notch retuned to the bending frequencies)\n');
fprintf(['Tanks: LOX/CH4 at mixture ratio 3.6, CG and inertia from the layout.\n' ...
    'CP is fixed where the old model had -0.5 cal (stack) or 0 cal (booster)\n' ...
    'with the CG at mid-length, so the margin now moves with the CG.\n']);
fprintf(['Slosh: NASA SP-106 pendulum, baffle damping %.2f. ' ...
    'Bending: two free-free modes, %.1f Hz on the full stack at liftoff,\n' ...
    'damping 0.01, frequency scaled as 1/sqrt(m*L^3). ' ...
    'Notch depth %.1f dB (zero zeta %.3f, pole zeta %.2f).\n'], ...
    S.sloshZeta, S.f1RefHz, S.notchDepthDb, S.notchZetaZ, S.notchZetaP);

fprintf(['\n%-10s %8s %8s %10s %8s %8s %8s %8s %8s %8s %8s\n'], ...
    'Condition', 'CG [m]', 'arm [m]', 'I [1e9]', 'mu', 'SM cal', ...
    'PM', 'GM', 'bGM1', 'bGM2', 'stable');
for k = 1:numel(S.points)
    p = S.points(k);
    fprintf('%-10s %8.2f %8.2f %10.3f %8.3f %8.2f %8.1f %8.1f %8.1f %8.1f %8s\n', ...
        char(p.name), p.cg, p.arm, p.inertia / 1e9, p.mu, p.sm, ...
        p.pm, p.gm, p.bendGm(1), p.bendGm(2), yesno(p.stable));
end
p = S.points(1);
fprintf(['\nLiftoff without the notch: PM %.1f deg, GM %.1f dB, ' ...
    'bending GM %.1f / %.1f dB.\n'], p.pmBare, p.gmBare, p.bendGmBare(1), p.bendGmBare(2));
fprintf('Liftoff slosh: %s\n', char(p.slosh));
fprintf('Liftoff bending: %.2f Hz and %.2f Hz. Gain crossover %.2f Hz.\n', ...
    p.fBend(1), p.fBend(2), p.wc / (2 * pi));
fprintf('Max-q slosh:    %s\n', char(S.points(2).slosh));
fprintf('Landing slosh:  %s\n', char(S.points(3).slosh));
fprintf('Landing bending: %.2f Hz and %.2f Hz.\n', ...
    S.points(3).fBend(1), S.points(3).fBend(2));

w = S.worstPm;
fprintf(['\nWorst phase margin: %.1f deg at t = %.0f s (%s), GM %.1f dB, ' ...
    'lambda %.3f /s^2, bending GM %.1f dB, stable %s.\n'], ...
    w.pm, w.t, char(w.phase), w.gm, w.lambda, w.bendGm, yesno(w.stable));
w = S.worstGm;
fprintf(['Worst gain margin:  %.1f dB at t = %.0f s (%s), PM %.1f deg, ' ...
    'bending GM %.1f dB, stable %s.\n'], ...
    w.gm, w.t, char(w.phase), w.pm, w.bendGm, yesno(w.stable));
w = S.worstBend;
fprintf('Worst bending GM:   %.1f dB (mode 1) at t = %.0f s (%s), PM %.1f deg, GM %.1f dB.\n\n', ...
    w.bendGm, w.t, char(w.phase), w.pm, w.gm);
end
