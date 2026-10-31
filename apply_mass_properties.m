function traj = apply_mass_properties(traj, V)
%APPLY_MASS_PROPERTIES  Replace rod CG / mu / lambda columns on a trajectory.
%
%   traj = apply_mass_properties(traj, V)
%
%   The CSV remains the companion's mass, thrust and dynamic pressure.
%   Centre of mass, gimbal arm, inertia, mu, lambda and the static margin
%   are recomputed from the tank layout. The centre of pressure stays at
%   the station implied by V.staticMarginAscent (stack) or
%   V.staticMarginLanding (booster) with the CG at mid-length. Running
%   this twice does not walk the CP, because the reference margin is read
%   from V and not from the column that gets overwritten.

n = height(traj);
cg = zeros(n, 1);
arm = zeros(n, 1);
cp = zeros(n, 1);
lcp = zeros(n, 1);
inertia = zeros(n, 1);
sm = zeros(n, 1);
mu = zeros(n, 1);
lam = zeros(n, 1);
for i = 1:n
    cfg = traj.config(i);
    if strcmp(cfg, "stack")
        refSm = V.staticMarginAscent;
    else
        refSm = V.staticMarginLanding;
    end
    mp = vehicle_mass_properties(traj.mass_kg(i), traj.thrust_N(i), ...
        traj.q_Pa(i), cfg, V, refSm);
    cg(i) = mp.cg;
    arm(i) = mp.arm;
    cp(i) = mp.xCp;
    lcp(i) = mp.lcp;
    inertia(i) = mp.I;
    sm(i) = mp.staticMarginCal;
    mu(i) = mp.mu;
    lam(i) = mp.lambda;
end
traj.cg_from_nose_m = cg;
traj.arm_m = arm;
traj.cp_from_nose_m = cp;
traj.lcp_m = lcp;
traj.inertia_kgm2 = inertia;
traj.static_margin_cal = sm;
traj.mu_rps2 = mu;
traj.lambda_rps2 = lam;
end
