function mdl = build_pitch_model()
%BUILD_PITCH_MODEL  Create boosterPitchTVC.slx from standard Simulink blocks.
%
%   mdl = build_pitch_model()
%
%   The diagram is three subsystems:
%
%     theta_cmd -> Controller -> delta_cmd -> Actuator -> delta -> Plant
%                     ^                                           |
%                     +---------------- theta, q -----------------+
%
%   Controller: command prefilter, then a governor that limits how far
%   and how fast the pitch command can lead the measured pitch. An inner
%   rate loop and an outer attitude loop (cascade) follow, with
%   back-calculation anti-windup, and the 1/mu gain schedule.
%   Actuator: second order wn, zeta, with integrator saturation for the
%   gimbal rate and angle limits.
%   Plant: theta'' = mu*sin(delta + misalign) + lambda*theta
%          + gustAccPerWind*wind.
%
%   Every numeric setting is a base-workspace name (Kp, mu, ...), filled by
%   apply_pitch_condition. The file is written to the current folder. It is
%   gitignored: generating an .slx needs Simulink, which this repo does not
%   run in CI.

mdl = 'boosterPitchTVC';
require_simulink();

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
target = fullfile(pwd, [mdl '.slx']);
if isfile(target)
    delete(target);
end

new_system(mdl);
open_system(mdl);

% Solver and callbacks are required. Description and a few optional
% flags are applied one at a time so a rejected cosmetic name cannot
% undo the solver setup.
set_param(mdl, ...
    'SolverType', 'Variable-step', ...
    'Solver', 'ode45', ...
    'RelTol', '1e-6', ...
    'AbsTol', '1e-8', ...
    'MaxStep', '0.005', ...
    'StopTime', 'tStop', ...
    'SaveTime', 'on', ...
    'SaveOutput', 'on', ...
    'SignalLogging', 'off', ...
    'SimulationMode', 'normal', ...
    'PreLoadFcn', 'addpath(fileparts(get_param(bdroot,''FileName'')))', ...
    'InitFcn', 'pitch_model_init');
try_set(mdl, ...
    'ReturnWorkspaceOutputs', 'on', ...
    'Description', ['Super Heavy booster pitch attitude control through ' ...
        'thrust-vector control. Built by build_pitch_model.m.']);

add_std('simulink/Sources/Step', mdl, 'theta_cmd', [40 148 90 182], ...
    'Time', 'thetaStepTime', 'Before', '0', 'After', 'thetaStep');

add_controller(mdl);
add_actuator(mdl);
add_plant(mdl);
add_monitoring(mdl);

add_line(mdl, 'theta_cmd/1', 'Controller/1', 'autorouting', 'on');
add_line(mdl, 'Controller/1', 'Actuator/1', 'autorouting', 'on');
add_line(mdl, 'Actuator/1', 'Plant/1', 'autorouting', 'on');
add_line(mdl, 'Actuator/1', 'Controller/4', 'autorouting', 'on');
add_line(mdl, 'Plant/1', 'Controller/2', 'autorouting', 'on');
add_line(mdl, 'Plant/2', 'Controller/3', 'autorouting', 'on');

% Branches into scopes and logs. Outport 2 of the actuator is gimbal rate.
add_line(mdl, 'theta_cmd/1', 'scope_attitude/1', 'autorouting', 'on');
add_line(mdl, 'Plant/1', 'scope_attitude/2', 'autorouting', 'on');
add_line(mdl, 'Controller/1', 'scope_gimbal/1', 'autorouting', 'on');
add_line(mdl, 'Actuator/1', 'scope_gimbal/2', 'autorouting', 'on');
add_line(mdl, 'Actuator/2', 'scope_rate/1', 'autorouting', 'on');

add_line(mdl, 'Plant/1', 'log_theta/1', 'autorouting', 'on');
add_line(mdl, 'Plant/2', 'log_q/1', 'autorouting', 'on');
add_line(mdl, 'Actuator/1', 'log_delta/1', 'autorouting', 'on');
add_line(mdl, 'Controller/1', 'log_deltaCmd/1', 'autorouting', 'on');
add_line(mdl, 'Actuator/2', 'log_rate/1', 'autorouting', 'on');
add_line(mdl, 'theta_cmd/1', 'log_thetaCmd/1', 'autorouting', 'on');
add_line(mdl, 'Controller/2', 'log_integ/1', 'autorouting', 'on');

name_line(mdl, 'Controller/1', 'delta_cmd');
name_line(mdl, 'Actuator/1', 'delta');
name_line(mdl, 'Plant/1', 'theta');
name_line(mdl, 'Plant/2', 'q');

annotate(mdl);
arrange_quietly(blk(mdl, 'Controller'));
arrange_quietly(blk(mdl, 'Actuator'));
arrange_quietly(blk(mdl, 'Plant'));
arrange_quietly(mdl);

save_system(mdl, target);
fprintf('Saved %s\n', target);
end


function add_controller(mdl)
% Cascade plus a command governor.
%
%   pref = 1/(cmdTau*s+1) * theta_cmd
%   lead is clipped to +/- e_lim, then the shaped command slews at +/- q_lim.
%   e_lim and q_lim scale with mu*sin(delta still available after trim),
%   and they never drop below the 5 deg floors.
%   shapeOn >= 0.5 uses the shaped command and the rate-command clip.
%   shapeOn < 0.5 is the same cascade with only the acceleration limit.
%
%   q_cmd = Kth*(theta_ref - theta) + integ
%   a_cmd = Kq*(q_cmd - q_filtered)
%   integ_dot = KiTh*e + Kaw*(q_cmd - q_unsat) + (Kaw/Kq)*(a_sat - a_raw)
sub = add_empty_subsystem(mdl, 'Controller', [160 80 420 300]);
paint(sub, 'lightBlue');
try_set(sub, 'Description', ['Prefilter, command governor, cascade rate and ' ...
    'attitude loops, 1/mu schedule.']);

add_in(sub, 'theta_cmd', 1, [20 28 50 42]);
add_in(sub, 'theta', 2, [20 118 50 132]);
add_in(sub, 'q', 3, [20 208 50 222]);
add_in(sub, 'delta', 4, [20 520 50 534]);

add_std('simulink/Continuous/Transfer Fcn', sub, 'cmd_prefilter', [90 15 180 55], ...
    'Numerator', '[1]', 'Denominator', '[cmdTau 1]');
add_std('simulink/Continuous/Transport Delay', sub, 'delay_theta', [90 105 180 145], ...
    'DelayTime', 'sensorDelay', 'InitialOutput', '0');
add_std('simulink/Continuous/Transport Delay', sub, 'delay_q', [90 195 180 235], ...
    'DelayTime', 'sensorDelay', 'InitialOutput', '0');
add_std('simulink/Continuous/Transfer Fcn', sub, 'rate_filter', [220 195 310 235], ...
    'Numerator', '[1]', 'Denominator', '[Tf 1]');

% Authority left after a slow measure of gimbal trim.
add_std('simulink/Continuous/Transfer Fcn', sub, 'trim_lpf', [90 505 180 545], ...
    'Numerator', '[1]', 'Denominator', '[trimTau 1]');
add_std('simulink/Math Operations/Abs', sub, 'abs_trim', [220 510 260 540]);
add_std('simulink/Sources/Constant', sub, 'const_dMax', [220 460 280 490], ...
    'Value', 'deltaMax');
add_sum(sub, 'sum_head', '+-', [320 470 350 530]);
add_std('simulink/Sources/Constant', sub, 'const_floor', [320 560 400 590], ...
    'Value', 'deltaAvailFloor');
add_minmax(sub, 'max_avail', [430 500 480 560]);
add_std('simulink/Math Operations/Trigonometric Function', sub, 'sin_avail', ...
    [520 515 570 545], 'Operator', 'sin');
add_std('simulink/Sources/Clock', sub, 'clock', [430 620 470 650]);
add_lookup(sub, 'lookup_mu', 'schedMu', [520 600 640 640]);
add_lookup(sub, 'lookup_invMu', 'schedInvMu', [900 250 1020 290]);
add_lookup(sub, 'lookup_aMax', 'schedACmdMax', [900 310 1020 350]);
add_std('simulink/Math Operations/Product', sub, 'prod_aAvail', [680 530 720 570], ...
    'Inputs', '**');
add_std('simulink/Math Operations/Gain', sub, 'gain_qAuth', [760 500 830 530], ...
    'Gain', 'shapeQ');
add_std('simulink/Math Operations/Gain', sub, 'gain_eAuth', [760 560 830 590], ...
    'Gain', 'shapeE');
add_std('simulink/Sources/Constant', sub, 'const_qFloor', [760 430 850 460], ...
    'Value', 'qLimFloor');
add_std('simulink/Sources/Constant', sub, 'const_eFloor', [760 640 850 670], ...
    'Value', 'eLimFloor');
add_minmax(sub, 'max_q', [900 450 950 510]);
add_minmax(sub, 'max_e', [900 560 950 620]);
add_std('simulink/Math Operations/Gain', sub, 'neg_q', [1000 450 1050 480], 'Gain', '-1');
add_std('simulink/Math Operations/Gain', sub, 'neg_e', [1000 560 1050 590], 'Gain', '-1');

% Lead limit, then a rate limit, around the prefiltered command.
add_sum(sub, 'sum_lead', '+-', [240 20 270 70]);
add_dynsat(sub, 'sat_lead', [360 20 420 80]);
add_sum(sub, 'sum_target', '++', [480 20 510 70]);
add_sum(sub, 'sum_serr', '+-', [600 20 630 70]);
add_std('simulink/Math Operations/Gain', sub, 'gain_ks', [680 25 740 55], ...
    'Gain', 'shapeKs');
add_dynsat(sub, 'sat_sdot', [800 15 860 75]);
add_std('simulink/Continuous/Integrator', sub, 'int_shaped', [920 25 960 55], ...
    'InitialCondition', '0');
add_std('simulink/Sources/Constant', sub, 'const_shape', [700 120 780 150], ...
    'Value', 'shapeOn');
add_switch(sub, 'sw_ref', [1040 20 1090 80]);

% Outer attitude loop. q_cmd is clipped only while the governor is on.
add_sum(sub, 'sum_e', '+-', [1160 30 1190 80]);
add_std('simulink/Math Operations/Gain', sub, 'gain_Kth', [1240 15 1300 45], ...
    'Gain', 'Kth');
add_std('simulink/Math Operations/Gain', sub, 'gain_KiTh', [1240 90 1300 120], ...
    'Gain', 'KiTh');
add_sum(sub, 'sum_q', '++', [1360 20 1390 70]);
add_dynsat(sub, 'sat_q', [1460 10 1520 70]);
add_switch(sub, 'sw_q', [1580 10 1630 70]);

% Inner rate loop, acceleration limit, 1/mu.
add_sum(sub, 'sum_rate', '+-', [1700 30 1730 80]);
add_std('simulink/Math Operations/Gain', sub, 'gain_Kq', [1780 35 1840 65], ...
    'Gain', 'Kq');
add_std('simulink/Math Operations/Gain', sub, 'neg_a', [1100 320 1150 350], 'Gain', '-1');
add_dynsat(sub, 'sat_a', [1900 20 1960 90]);
add_std('simulink/Math Operations/Product', sub, 'prod_invMu', [2020 30 2060 70], ...
    'Inputs', '**');

% Back-calculation from both the rate-command clip and the acceleration clip.
add_sum(sub, 'sum_qErr', '+-', [1700 140 1730 190]);
add_std('simulink/Math Operations/Gain', sub, 'gain_Kaw', [1780 145 1840 175], ...
    'Gain', 'Kaw');
add_sum(sub, 'sum_aErr', '+-', [1900 140 1930 190]);
add_std('simulink/Math Operations/Gain', sub, 'gain_KawA', [1980 145 2050 175], ...
    'Gain', 'KawA');
add_sum(sub, 'sum_idot', '+++', [2120 90 2150 150]);
add_std('simulink/Continuous/Integrator', sub, 'int_I', [2200 100 2240 130], ...
    'InitialCondition', '0');

add_out(sub, 'delta_cmd', 1, [2120 38 2150 52]);
add_out(sub, 'integ', 2, [2300 108 2330 122]);

wire = @(src, dst) add_line(sub, src, dst, 'autorouting', 'on');
wire('theta_cmd/1', 'cmd_prefilter/1');
wire('theta/1', 'delay_theta/1');
wire('q/1', 'delay_q/1');
wire('delay_q/1', 'rate_filter/1');
wire('delta/1', 'trim_lpf/1');
wire('trim_lpf/1', 'abs_trim/1');
wire('const_dMax/1', 'sum_head/1');
wire('abs_trim/1', 'sum_head/2');
wire('sum_head/1', 'max_avail/1');
wire('const_floor/1', 'max_avail/2');
wire('max_avail/1', 'sin_avail/1');
wire('clock/1', 'lookup_mu/1');
wire('clock/1', 'lookup_invMu/1');
wire('clock/1', 'lookup_aMax/1');
wire('sin_avail/1', 'prod_aAvail/1');
wire('lookup_mu/1', 'prod_aAvail/2');
wire('prod_aAvail/1', 'gain_qAuth/1');
wire('prod_aAvail/1', 'gain_eAuth/1');
wire('gain_qAuth/1', 'max_q/1');
wire('const_qFloor/1', 'max_q/2');
wire('gain_eAuth/1', 'max_e/1');
wire('const_eFloor/1', 'max_e/2');
wire('max_q/1', 'neg_q/1');
wire('max_e/1', 'neg_e/1');

wire('cmd_prefilter/1', 'sum_lead/1');
wire('delay_theta/1', 'sum_lead/2');
wire('max_e/1', 'sat_lead/1');
wire('sum_lead/1', 'sat_lead/2');
wire('neg_e/1', 'sat_lead/3');
wire('delay_theta/1', 'sum_target/1');
wire('sat_lead/1', 'sum_target/2');
wire('sum_target/1', 'sum_serr/1');
wire('int_shaped/1', 'sum_serr/2');
wire('sum_serr/1', 'gain_ks/1');
wire('max_q/1', 'sat_sdot/1');
wire('gain_ks/1', 'sat_sdot/2');
wire('neg_q/1', 'sat_sdot/3');
wire('sat_sdot/1', 'int_shaped/1');
wire('int_shaped/1', 'sw_ref/1');
wire('const_shape/1', 'sw_ref/2');
wire('cmd_prefilter/1', 'sw_ref/3');

wire('sw_ref/1', 'sum_e/1');
wire('delay_theta/1', 'sum_e/2');
wire('sum_e/1', 'gain_Kth/1');
wire('sum_e/1', 'gain_KiTh/1');
wire('gain_Kth/1', 'sum_q/1');
wire('int_I/1', 'sum_q/2');
wire('max_q/1', 'sat_q/1');
wire('sum_q/1', 'sat_q/2');
wire('neg_q/1', 'sat_q/3');
wire('sat_q/1', 'sw_q/1');
wire('const_shape/1', 'sw_q/2');
wire('sum_q/1', 'sw_q/3');

wire('sw_q/1', 'sum_rate/1');
wire('rate_filter/1', 'sum_rate/2');
wire('sum_rate/1', 'gain_Kq/1');
wire('lookup_aMax/1', 'sat_a/1');
wire('lookup_aMax/1', 'neg_a/1');
wire('neg_a/1', 'sat_a/3');
wire('gain_Kq/1', 'sat_a/2');
wire('sat_a/1', 'prod_invMu/1');
wire('lookup_invMu/1', 'prod_invMu/2');
wire('prod_invMu/1', 'delta_cmd/1');

wire('sw_q/1', 'sum_qErr/1');
wire('sum_q/1', 'sum_qErr/2');
wire('sum_qErr/1', 'gain_Kaw/1');
wire('sat_a/1', 'sum_aErr/1');
wire('gain_Kq/1', 'sum_aErr/2');
wire('sum_aErr/1', 'gain_KawA/1');
wire('gain_KiTh/1', 'sum_idot/1');
wire('gain_Kaw/1', 'sum_idot/2');
wire('gain_KawA/1', 'sum_idot/3');
wire('sum_idot/1', 'int_I/1');
wire('int_I/1', 'integ/1');
end


function add_actuator(mdl)
sub = add_empty_subsystem(mdl, 'Actuator', [430 110 630 250]);
paint(sub, 'orange');
try_set(sub, 'Description', ['Second-order gimbal servo. Integrator saturation is ' ...
    'the rate limit and the angle limit.']);

add_in(sub, 'delta_cmd', 1, [30 78 60 92]);
add_sum(sub, 'sum_err', '+-', [120 70 150 110]);
add_std('simulink/Math Operations/Gain', sub, 'wn2', [190 75 250 105], 'Gain', 'actWn^2');
add_sum(sub, 'sum_acc', '+-', [300 70 330 120]);
add_std('simulink/Continuous/Integrator', sub, 'int_rate', [380 78 420 112], ...
    'InitialCondition', '0', ...
    'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'gimbalRate', ...
    'LowerSaturationLimit', '-gimbalRate');
add_std('simulink/Continuous/Integrator', sub, 'int_angle', [480 78 520 112], ...
    'InitialCondition', '0', ...
    'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'deltaMax', ...
    'LowerSaturationLimit', '-deltaMax');
add_std('simulink/Math Operations/Gain', sub, 'damping', [380 160 460 190], ...
    'Gain', '2*actZeta*actWn');

add_out(sub, 'delta', 1, [580 78 610 92]);
add_out(sub, 'rate', 2, [480 160 510 174]);

add_line(sub, 'delta_cmd/1', 'sum_err/1', 'autorouting', 'on');
add_line(sub, 'int_angle/1', 'sum_err/2', 'autorouting', 'on');
add_line(sub, 'sum_err/1', 'wn2/1', 'autorouting', 'on');
add_line(sub, 'wn2/1', 'sum_acc/1', 'autorouting', 'on');
add_line(sub, 'damping/1', 'sum_acc/2', 'autorouting', 'on');
add_line(sub, 'sum_acc/1', 'int_rate/1', 'autorouting', 'on');
add_line(sub, 'int_rate/1', 'int_angle/1', 'autorouting', 'on');
add_line(sub, 'int_rate/1', 'damping/1', 'autorouting', 'on');
add_line(sub, 'int_angle/1', 'delta/1', 'autorouting', 'on');
add_line(sub, 'int_rate/1', 'rate/1', 'autorouting', 'on');
end


function add_plant(mdl)
sub = add_empty_subsystem(mdl, 'Plant', [700 90 920 280]);
% 'lightGreen' is not a Simulink colour name. Use an RGB string, and do
% not let a rejected colour stop the build.
paint(sub, '[0.75,0.93,0.75]');
try_set(sub, 'Description', ['Rigid pitch: theta'''' = mu*sin(delta+mis) + ' ...
    'lambda*theta + gustAccPerWind*wind.']);

add_in(sub, 'delta', 1, [30 48 60 62]);
add_std('simulink/Sources/Step', sub, 'misalign', [30 110 80 140], ...
    'Time', 'misalignTime', 'Before', '0', 'After', 'misRad');
add_std('simulink/Sources/Step', sub, 'wind', [30 250 80 280], ...
    'Time', 'gustTime', 'Before', '0', 'After', 'gustSpeed');

add_sum(sub, 'sum_angle', '++', [140 45 170 95]);
add_std('simulink/Math Operations/Trigonometric Function', sub, 'sin_angle', ...
    [210 55 260 85], 'Operator', 'sin');
add_std('simulink/Sources/Clock', sub, 'clock', [210 300 250 330]);
add_lookup(sub, 'lookup_mu', 'schedMu', [280 290 400 330]);
add_lookup(sub, 'lookup_lambda', 'schedLambda', [400 200 520 240]);
add_std('simulink/Math Operations/Product', sub, 'prod_mu', [430 50 470 90], ...
    'Inputs', '**');
add_std('simulink/Math Operations/Gain', sub, 'gust_gain', [140 245 220 275], ...
    'Gain', 'gustAccPerWind');
add_std('simulink/Math Operations/Product', sub, 'prod_lambda', [560 160 600 200], ...
    'Inputs', '**');

add_sum(sub, 'sum_qdot', '+++', [420 50 450 120]);
add_std('simulink/Continuous/Integrator', sub, 'int_q', [500 70 540 100], ...
    'InitialCondition', '0');
add_std('simulink/Continuous/Integrator', sub, 'int_theta', [600 70 640 100], ...
    'InitialCondition', '0');

add_out(sub, 'theta', 1, [700 73 730 87]);
add_out(sub, 'q', 2, [600 140 630 154]);

add_line(sub, 'delta/1', 'sum_angle/1', 'autorouting', 'on');
add_line(sub, 'misalign/1', 'sum_angle/2', 'autorouting', 'on');
add_line(sub, 'sum_angle/1', 'sin_angle/1', 'autorouting', 'on');
add_line(sub, 'clock/1', 'lookup_mu/1', 'autorouting', 'on');
add_line(sub, 'clock/1', 'lookup_lambda/1', 'autorouting', 'on');
add_line(sub, 'sin_angle/1', 'prod_mu/1', 'autorouting', 'on');
add_line(sub, 'lookup_mu/1', 'prod_mu/2', 'autorouting', 'on');
add_line(sub, 'prod_mu/1', 'sum_qdot/1', 'autorouting', 'on');
add_line(sub, 'lookup_lambda/1', 'prod_lambda/1', 'autorouting', 'on');
add_line(sub, 'int_theta/1', 'prod_lambda/2', 'autorouting', 'on');
add_line(sub, 'prod_lambda/1', 'sum_qdot/2', 'autorouting', 'on');
add_line(sub, 'wind/1', 'gust_gain/1', 'autorouting', 'on');
add_line(sub, 'gust_gain/1', 'sum_qdot/3', 'autorouting', 'on');
add_line(sub, 'sum_qdot/1', 'int_q/1', 'autorouting', 'on');
add_line(sub, 'int_q/1', 'int_theta/1', 'autorouting', 'on');
add_line(sub, 'int_theta/1', 'theta/1', 'autorouting', 'on');
add_line(sub, 'int_q/1', 'q/1', 'autorouting', 'on');
end


function add_monitoring(mdl)
add_scope(mdl, 'scope_attitude', 2, [980 40 1040 90]);
add_scope(mdl, 'scope_gimbal', 2, [980 120 1040 170]);
add_scope(mdl, 'scope_rate', 1, [980 200 1040 240]);
add_log(mdl, 'log_theta', 'theta', [980 270 1060 300]);
add_log(mdl, 'log_q', 'q', [980 310 1060 340]);
add_log(mdl, 'log_delta', 'delta', [980 350 1060 380]);
add_log(mdl, 'log_deltaCmd', 'deltaCmd', [980 390 1060 420]);
add_log(mdl, 'log_rate', 'gimbalRateMeas', [980 430 1060 460]);
add_log(mdl, 'log_thetaCmd', 'thetaCmd', [980 470 1060 500]);
add_log(mdl, 'log_integ', 'integ', [980 510 1060 540]);
end


function add_scope(parent, name, nports, pos)
add_std('simulink/Sinks/Scope', parent, name, pos);
set_param(blk(parent, name), 'NumInputPorts', num2str(nports));
end


function add_log(parent, name, varName, pos)
% Timeseries is what run_pitch_tvc reads. Structure With Time is the
% fallback unpack_log already accepts, if this release lacks Timeseries.
% MaxDataPoints is the current name; LimitDataPoints is the old one.
% Probe each name quietly so a missing parameter does not warn.
path = add_std('simulink/Sinks/To Workspace', parent, name, pos, ...
    'VariableName', varName, 'SampleTime', '-1');
if ~try_param(path, 'SaveFormat', 'Timeseries')
    try_set(path, 'SaveFormat', 'Structure With Time');
end
if ~try_param(path, 'MaxDataPoints', 'inf')
    try_param(path, 'LimitDataPoints', 'off');
end
end


function sub = add_empty_subsystem(parent, name, pos)
sub = blk(parent, name);
add_block('simulink/Ports & Subsystems/Subsystem', sub);
try_set(sub, 'Position', pos);
cleared = false;
try
    Simulink.SubSystem.deleteContents(sub);
    cleared = true;
catch
    cleared = false;
end
if ~cleared
    lineH = find_system(sub, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
    if ~isempty(lineH)
        delete_line(lineH);
    end
    blocks = find_system(sub, 'SearchDepth', 1, 'Type', 'block');
    for i = 1:numel(blocks)
        if ~strcmp(blocks{i}, sub)
            delete_block(blocks{i});
        end
    end
end
end


function add_in(parent, name, port, pos)
add_std('simulink/Sources/In1', parent, name, pos, 'Port', num2str(port));
end


function add_out(parent, name, port, pos)
add_std('simulink/Sinks/Out1', parent, name, pos, 'Port', num2str(port));
end


function add_dynsat(parent, name, pos)
% Ports: 1 = upper limit, 2 = signal, 3 = lower limit.
add_std('simulink/Discontinuities/Saturation Dynamic', parent, name, pos);
end


function add_switch(parent, name, pos)
% Port 1 passes when the control port is >= 0.5. Port 3 passes otherwise.
add_std('simulink/Signal Routing/Switch', parent, name, pos, ...
    'Criteria', 'u2 >= Threshold', ...
    'Threshold', '0.5');
end


function add_minmax(parent, name, pos)
add_std('simulink/Math Operations/MinMax', parent, name, pos, ...
    'Function', 'max', ...
    'Inputs', '2');
end


function add_sum(parent, name, signs, pos)
% Inputs is required. Icon shape is cosmetic.
add_std('simulink/Math Operations/Sum', parent, name, pos, 'Inputs', signs);
try_set(blk(parent, name), 'IconShape', 'rectangular');
end


function add_lookup(parent, name, tableName, pos)
% The 1-D Lookup Table block has no NumberOfTableDimensions parameter.
% BreakpointsForDimension1 and Table are the data. Interpolation and
% extrapolation names are applied separately so a release that uses a
% different label still keeps the table.
add_std('simulink/Lookup Tables/1-D Lookup Table', parent, name, pos, ...
    'BreakpointsForDimension1', 'schedTime', ...
    'Table', tableName);
path = blk(parent, name);
try_set(path, 'InterpMethod', 'Linear point-slope');
try_set(path, 'ExtrapMethod', 'Clip');
end


function path = add_std(lib, parent, name, pos, varargin)
% Functional parameters go to add_block and must succeed. Position is
% applied afterwards and must not abort the build.
path = blk(parent, name);
if isempty(varargin)
    add_block(lib, path);
else
    add_block(lib, path, varargin{:});
end
try_set(path, 'Position', pos);
end


function paint(block, color)
% Named colours Simulink accepts, or an '[r, g, b]' string with each
% channel in 0..1. lightGreen is not in the named list.
try_set(block, 'BackgroundColor', color);
end


function ok = try_param(block, name, value)
% True when Simulink accepts the setting. A miss is silent: callers decide
% whether to try another name or to warn.
try
    set_param(block, name, value);
    ok = true;
catch
    ok = false;
end
end


function try_set(block, varargin)
% One parameter per call. A rejected cosmetic setting warns and continues.
for k = 1:2:numel(varargin)
    name = varargin{k};
    value = varargin{k + 1};
    try
        set_param(block, name, value);
    catch ME
        warning('build_pitch_model:Param', ...
            'Skipped %s on %s (%s).', name, block, ME.message);
    end
end
end


function annotate(mdl)
try
    note = Simulink.Annotation(blk(mdl, 'Overview'));
    note.Text = ['Pitch TVC: command -> governor -> cascade -> actuator -> plant.' newline ...
        'Kq, Kth and KiTh are the cascade. delta_cmd = a_cmd / mu(t).' newline ...
        'shapeOn = 0 leaves the governor off. Variables come from apply_pitch_condition.'];
    note.Position = [40 20];
    note.FontSize = 11;
catch ME
    warning('build_pitch_model:Annotation', '%s', ME.message);
end
end


function arrange_quietly(sys)
try
    Simulink.BlockDiagram.arrangeSystem(sys);
catch ME
    warning('build_pitch_model:Arrange', ...
        'Auto-layout of %s was skipped (%s).', sys, ME.message);
end
end


function name_line(mdl, port, label)
% port is 'Block/N' on the top level. Naming is cosmetic.
try
    slash = strfind(port, '/');
    blockName = port(1:slash(end)-1);
    outIdx = str2double(port(slash(end)+1:end));
    ph = get_param(blk(mdl, blockName), 'PortHandles');
    lh = get_param(ph.Outport(outIdx), 'Line');
    if lh ~= -1
        set_param(lh, 'Name', label);
    end
catch
end
end


function p = blk(parent, name)
p = [parent '/' name];
end


function require_simulink()
if ~license('test', 'Simulink')
    error('build_pitch_model:NeedsSimulink', ...
        'Simulink is required to build boosterPitchTVC.slx.');
end
end
