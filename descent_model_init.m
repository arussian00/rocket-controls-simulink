function descent_model_init()
%DESCENT_MODEL_INIT  Fill the base workspace so boosterDescentPitch can Run.
%
%   The diagram reads dTime, dThetaCmd, dMuTvc and the rest from the base
%   workspace. run_booster_descent writes them before sim. If they are
%   missing (the model was opened and Run was pressed), load the return
%   schedule quietly. This does not run the fixed-step study.

if evalin('base', 'exist(''dTime'', ''var'')')
    return
end

V = vehicle_params();
D = design_pitch_controller(V, plot=false, verbose=false);
ret = make_booster_return_trajectory([], V);
tq = schedule_times(ret);
S = descent_openloop(ret, tq, D);
apply_descent_workspace(S, D, V, ret);
fprintf(['descent_model_init: base workspace was empty, loaded the booster ' ...
    'return from %.0f s to %.0f s.\n'], ret.t(1), ret.t(end));
end


function tq = schedule_times(ret)
dt = 0.05;
n = round((ret.t(end) - ret.t(1)) / dt) + 1;
tq = ret.t(1) + (0:n-1).' * dt;
tq = min(tq, ret.t(end));
end