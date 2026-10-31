function pitch_model_init()
%PITCH_MODEL_INIT  Default base-workspace parameters for a manual model Run.
%
%   Simulink evaluates Kp, mu, tStop and the rest from the base workspace.
%   run_pitch_tvc sets them before each sim. If they are missing (the model
%   was opened and Run was pressed), load the liftoff condition quietly.

if evalin('base', 'exist(''deltaMax'', ''var'')')
    return
end

V = vehicle_params();
D = design_pitch_controller(V, plot=false, verbose=false);
apply_pitch_condition(D, "liftoff");
assignin('base', 'pitchDesign', D);
fprintf(['pitch_model_init: base workspace was empty, loaded the liftoff ' ...
    'condition (%.0f deg step, %.0f m/s gust).\n'], D.thetaStepDeg, D.gustSpeed);
end
