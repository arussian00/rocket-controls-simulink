function mdl = build_descent_model()
%BUILD_DESCENT_MODEL  Create boosterDescentPitch.slx from standard blocks.
%
%   mdl = build_descent_model()
%
%   Pitch of the booster from hot staging to the tower catch. The controller
%   is the ascent cascade (governor, rate loop, back-calculation) plus an
%   allocation across RCS, TVC and grid fins. Lookup tables are filled by
%   apply_descent_workspace. The file is gitignored: building it needs
%   Simulink.

mdl = 'boosterDescentPitch';
if ~license('test', 'Simulink')
    error('build_descent_model:NeedsSimulink', ...
        'Simulink is required to build boosterDescentPitch.slx.');
end

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
target = fullfile(pwd, [mdl '.slx']);
if isfile(target)
    delete(target);
end

new_system(mdl);
open_system(mdl);
set_param(mdl, ...
    'SolverType', 'Variable-step', ...
    'Solver', 'ode45', ...
    'RelTol', '1e-5', ...
    'AbsTol', '1e-7', ...
    'MaxStep', '0.01', ...
    'StartTime', 'dTStart', ...
    'StopTime', 'dTEnd', ...
    'SaveTime', 'on', ...
    'SaveOutput', 'on', ...
    'SignalLogging', 'off', ...
    'SimulationMode', 'normal', ...
    'PreLoadFcn', 'addpath(fileparts(get_param(bdroot,''FileName'')))', ...
    'InitFcn', 'descent_model_init');
try_set(mdl, ...
    'ReturnWorkspaceOutputs', 'on', ...
    'Description', ['Super Heavy booster return: flip, boostback, grid-fin ' ...
        'entry, landing burn, tower catch. Built by build_descent_model.m.']);

add_controller(mdl);
add_servo(mdl, 'ActTVC', [430 40 620 160], ...
    'dActWn', 'dActZeta', 'dRateMax', 'dDeltaMax', '[0.98,0.75,0.45]');
add_servo(mdl, 'ActFin', [430 200 620 320], ...
    'dFinWn', 'dFinZeta', 'dFinRate', 'dFinMax', '[0.95,0.65,0.35]');
add_std('simulink/Sinks/Terminator', mdl, 'term_tvc_rate', [640 70 660 90]);
add_std('simulink/Sinks/Terminator', mdl, 'term_fin_rate', [640 230 660 250]);
add_rcs(mdl);
add_plant(mdl);
add_monitoring(mdl);

wire = @(a, b) add_line(mdl, a, b, 'autorouting', 'on');
wire('Controller/1', 'ActTVC/1');
wire('ActTVC/2', 'term_tvc_rate/1');
wire('Controller/2', 'ActFin/1');
wire('ActFin/2', 'term_fin_rate/1');
wire('Controller/3', 'RCS/1');
wire('ActTVC/1', 'Plant/1');
wire('ActFin/1', 'Plant/2');
wire('RCS/1', 'Plant/3');
wire('Plant/1', 'Controller/1');
wire('Plant/2', 'Controller/2');
wire('ActTVC/1', 'Controller/3');
wire('ActFin/1', 'Controller/4');

wire('Plant/1', 'log_theta/1');
wire('Plant/2', 'log_q/1');
wire('Controller/5', 'log_shaped/1');
wire('Controller/6', 'log_thetaCmd/1');
wire('ActTVC/1', 'log_tvc/1');
wire('ActFin/1', 'log_fin/1');
wire('Controller/1', 'log_cmdTvc/1');
wire('Controller/2', 'log_cmdFin/1');
wire('Controller/4', 'log_integ/1');
wire('RCS/1', 'log_rcs/1');
wire('Plant/1', 'scope_attitude/1');
wire('Controller/6', 'scope_attitude/2');
wire('ActTVC/1', 'scope_surface/1');
wire('ActFin/1', 'scope_surface/2');

arrange_quietly(blk(mdl, 'Controller'));
arrange_quietly(blk(mdl, 'ActTVC'));
arrange_quietly(blk(mdl, 'ActFin'));
arrange_quietly(blk(mdl, 'RCS'));
arrange_quietly(blk(mdl, 'Plant'));
arrange_quietly(mdl);
save_system(mdl, target);
fprintf('Saved %s\n', target);
end


function add_controller(mdl)
sub = add_empty_subsystem(mdl, 'Controller', [140 40 380 340]);
paint(sub, 'lightBlue');
try_set(sub, 'Description', ...
    'Governor, cascade, anti-windup, bumpless allocation to RCS, TVC and fins.');

add_in(sub, 'theta', 1, [20 80 50 94]);
add_in(sub, 'q', 2, [20 150 50 164]);
add_in(sub, 'delta_tvc', 3, [20 230 50 244]);
add_in(sub, 'delta_fin', 4, [20 300 50 314]);

add_clock_lookups(sub);
add_pref_and_filters(sub);
add_authority(sub);
add_governor_and_cascade(sub);
add_allocation(sub);

wire = @(a, b) add_line(sub, a, b, 'autorouting', 'on');

% Command, measurements, filters.
wire('lu_cmd/1', 'sum_pref/1');
wire('int_pref/1', 'sum_pref/2');
wire('sum_pref/1', 'gain_pref/1');
wire('gain_pref/1', 'int_pref/1');
wire('theta/1', 'delay_theta/1');
wire('q/1', 'delay_q/1');
wire('delay_q/1', 'sum_qf/1');
wire('int_qf/1', 'sum_qf/2');
wire('sum_qf/1', 'gain_qf/1');
wire('gain_qf/1', 'int_qf/1');
wire('delta_tvc/1', 'sum_trim/1');
wire('int_trim/1', 'sum_trim/2');
wire('sum_trim/1', 'gain_trim/1');
wire('gain_trim/1', 'int_trim/1');

% Authority left on each surface, then the governor floors.
wire('int_trim/1', 'abs_trim/1');
wire('const_dMax/1', 'sum_headT/1');
wire('abs_trim/1', 'sum_headT/2');
wire('sum_headT/1', 'max_headT/1');
wire('const_floorT/1', 'max_headT/2');
wire('max_headT/1', 'sin_head/1');
wire('sin_head/1', 'prod_aT/1');
wire('lu_muT/1', 'prod_aT/2');
wire('lu_gateT/1', 'prod_aT/3');
wire('delta_fin/1', 'abs_fin/1');
wire('const_fMax/1', 'sum_headF/1');
wire('abs_fin/1', 'sum_headF/2');
wire('sum_headF/1', 'max_headF/1');
wire('const_floorF/1', 'max_headF/2');
wire('max_headF/1', 'prod_aF/1');
wire('lu_muF/1', 'prod_aF/2');
wire('lu_gateF/1', 'prod_aF/3');
wire('lu_aRcs/1', 'prod_aR/1');
wire('lu_gateR/1', 'prod_aR/2');
wire('prod_aT/1', 'sum_avail/1');
wire('prod_aF/1', 'sum_avail/2');
wire('prod_aR/1', 'sum_avail/3');
wire('sum_avail/1', 'gain_eAuth/1');
wire('sum_avail/1', 'gain_qAuth/1');
wire('gain_eAuth/1', 'max_e/1');
wire('lu_eFloor/1', 'max_e/2');
wire('gain_qAuth/1', 'max_q/1');
wire('lu_qFloor/1', 'max_q/2');
wire('max_e/1', 'neg_e/1');
wire('max_q/1', 'neg_q/1');

% Governor and cascade. q_used is the saturated rate command.
wire('int_pref/1', 'sum_lead/1');
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
wire('int_shaped/1', 'sum_e/1');
wire('delay_theta/1', 'sum_e/2');
wire('sum_e/1', 'gain_Kth/1');
wire('sum_e/1', 'gain_KiTh/1');
wire('gain_Kth/1', 'sum_q/1');
wire('int_I/1', 'sum_q/2');
wire('max_q/1', 'sat_q/1');
wire('sum_q/1', 'sat_q/2');
wire('neg_q/1', 'sat_q/3');
wire('sat_q/1', 'sum_rate/1');
wire('int_qf/1', 'sum_rate/2');
wire('sum_rate/1', 'gain_Kq/1');
wire('lu_aMax/1', 'sat_a/1');
wire('gain_Kq/1', 'sat_a/2');
wire('lu_aMax/1', 'neg_a/1');
wire('neg_a/1', 'sat_a/3');

% Back-calculation on the rate-command clip and the acceleration clip.
wire('sat_q/1', 'sum_qErr/1');
wire('sum_q/1', 'sum_qErr/2');
wire('sum_qErr/1', 'gain_Kaw/1');
wire('sat_a/1', 'sum_aErr/1');
wire('gain_Kq/1', 'sum_aErr/2');
wire('sum_aErr/1', 'gain_KawA/1');
wire('gain_KiTh/1', 'sum_idot/1');
wire('gain_Kaw/1', 'sum_idot/2');
wire('gain_KawA/1', 'sum_idot/3');
wire('sum_idot/1', 'int_I/1');

% Allocation, then a rate limit so the command state cannot step.
wire('sat_a/1', 'prod_desT/1');
wire('lu_wT/1', 'prod_desT/2');
wire('lu_invT/1', 'prod_desT/3');
wire('prod_desT/1', 'sat_desT/1');
wire('sat_desT/1', 'sum_slT/1');
wire('int_cmdT/1', 'sum_slT/2');
wire('sum_slT/1', 'gain_slT/1');
wire('gain_slT/1', 'sat_slT/1');
wire('sat_slT/1', 'int_cmdT/1');

wire('sat_a/1', 'prod_desF/1');
wire('lu_wF/1', 'prod_desF/2');
wire('lu_invF/1', 'prod_desF/3');
wire('prod_desF/1', 'sat_desF/1');
wire('sat_desF/1', 'sum_slF/1');
wire('int_cmdF/1', 'sum_slF/2');
wire('sum_slF/1', 'gain_slF/1');
wire('gain_slF/1', 'sat_slF/1');
wire('sat_slF/1', 'int_cmdF/1');

wire('sat_a/1', 'prod_desR/1');
wire('lu_wR/1', 'prod_desR/2');
wire('prod_desR/1', 'sat_desR/2');
wire('lu_aRcs/1', 'sat_desR/1');
wire('lu_aRcs/1', 'neg_rcs/1');
wire('neg_rcs/1', 'sat_desR/3');
wire('sat_desR/1', 'sum_slR/1');
wire('int_cmdR/1', 'sum_slR/2');
wire('sum_slR/1', 'gain_slR/1');
wire('gain_slR/1', 'sat_slR/1');
wire('sat_slR/1', 'int_cmdR/1');

wire('int_cmdT/1', 'cmd_tvc/1');
wire('int_cmdF/1', 'cmd_fin/1');
wire('int_cmdR/1', 'cmd_rcs/1');
wire('int_I/1', 'integ/1');
wire('int_shaped/1', 'shaped/1');
wire('lu_cmd/1', 'theta_cmd/1');
end


function add_clock_lookups(sub)
add_std('simulink/Sources/Clock', sub, 'clock', [20 20 50 40]);
names = { ...
    'lu_cmd', 'dThetaCmd'; ...
    'lu_muT', 'dMuTvc'; ...
    'lu_muF', 'dMuFin'; ...
    'lu_gateT', 'dGateT'; ...
    'lu_gateF', 'dGateF'; ...
    'lu_gateR', 'dGateR'; ...
    'lu_aRcs', 'dARcs'; ...
    'lu_wT', 'dWTvc'; ...
    'lu_wF', 'dWFin'; ...
    'lu_wR', 'dWRcs'; ...
    'lu_invT', 'dInvTvc'; ...
    'lu_invF', 'dInvFin'; ...
    'lu_aMax', 'dAMax'; ...
    'lu_eFloor', 'dEFloor'; ...
    'lu_qFloor', 'dQFloor'};
for k = 1:size(names, 1)
    add_lookup(sub, names{k, 1}, names{k, 2}, [80 20 160 50]);
    add_line(sub, 'clock/1', [names{k, 1} '/1'], 'autorouting', 'on');
end
end


function add_pref_and_filters(sub)
add_sum(sub, 'sum_pref', '+-', [200 60 230 90]);
add_gain(sub, 'gain_pref', '1/dCmdTau', [250 65 310 95]);
add_int(sub, 'int_pref', 'dTheta0', [330 65 370 95]);
add_std('simulink/Continuous/Transport Delay', sub, 'delay_theta', [90 120 170 150], ...
    'DelayTime', 'dSensorDelay', 'InitialOutput', 'dTheta0');
add_std('simulink/Continuous/Transport Delay', sub, 'delay_q', [90 180 170 210], ...
    'DelayTime', 'dSensorDelay', 'InitialOutput', '0');
add_sum(sub, 'sum_qf', '+-', [200 175 230 205]);
add_gain(sub, 'gain_qf', '1/dTf', [250 180 310 210]);
add_int(sub, 'int_qf', '0', [330 180 370 210]);
add_sum(sub, 'sum_trim', '+-', [200 250 230 280]);
add_gain(sub, 'gain_trim', '1/dTrimTau', [250 255 310 285]);
add_int(sub, 'int_trim', '0', [330 255 370 285]);
end


function add_authority(sub)
add_std('simulink/Math Operations/Abs', sub, 'abs_trim', [400 250 430 280]);
add_const(sub, 'const_dMax', 'dDeltaMax', [400 210 470 230]);
add_sum(sub, 'sum_headT', '+-', [490 220 520 260]);
add_const(sub, 'const_floorT', 'dDeltaAvailFloor', [490 270 570 290]);
add_minmax(sub, 'max_headT', [590 230 630 270]);
add_std('simulink/Math Operations/Trigonometric Function', sub, 'sin_head', ...
    [650 235 690 265], 'Operator', 'sin');
add_prod(sub, 'prod_aT', '***', [720 220 760 270]);

add_std('simulink/Math Operations/Abs', sub, 'abs_fin', [400 330 430 360]);
add_const(sub, 'const_fMax', 'dFinMax', [400 300 470 320]);
add_sum(sub, 'sum_headF', '+-', [490 310 520 350]);
add_const(sub, 'const_floorF', 'dFinAvailFloor', [490 360 570 380]);
add_minmax(sub, 'max_headF', [590 320 630 360]);
add_prod(sub, 'prod_aF', '***', [720 310 760 360]);
add_prod(sub, 'prod_aR', '**', [720 390 760 430]);
add_sum(sub, 'sum_avail', '+++', [800 280 830 360]);
add_gain(sub, 'gain_eAuth', 'dShapeE', [860 270 920 300]);
add_gain(sub, 'gain_qAuth', 'dShapeQ', [860 330 920 360]);
add_minmax(sub, 'max_e', [950 260 990 310]);
add_minmax(sub, 'max_q', [950 330 990 380]);
add_gain(sub, 'neg_e', '-1', [1020 270 1060 300]);
add_gain(sub, 'neg_q', '-1', [1020 340 1060 370]);
end


function add_governor_and_cascade(sub)
add_sum(sub, 'sum_lead', '+-', [400 60 430 100]);
add_dynsat(sub, 'sat_lead', [460 55 520 105]);
add_sum(sub, 'sum_target', '++', [550 60 580 100]);
add_sum(sub, 'sum_serr', '+-', [610 60 640 100]);
add_gain(sub, 'gain_ks', 'dShapeKs', [660 65 720 95]);
add_dynsat(sub, 'sat_sdot', [750 55 810 105]);
add_int(sub, 'int_shaped', 'dTheta0', [840 65 880 95]);

add_sum(sub, 'sum_e', '+-', [920 60 950 100]);
add_gain(sub, 'gain_Kth', 'dKth', [980 40 1040 70]);
add_gain(sub, 'gain_KiTh', 'dKiTh', [980 100 1040 130]);
add_sum(sub, 'sum_q', '++', [1080 50 1110 90]);
add_dynsat(sub, 'sat_q', [1140 45 1200 95]);
add_sum(sub, 'sum_rate', '+-', [1240 50 1270 90]);
add_gain(sub, 'gain_Kq', 'dKq', [1300 55 1360 85]);
add_gain(sub, 'neg_a', '-1', [1300 110 1340 140]);
add_dynsat(sub, 'sat_a', [1400 45 1460 105]);

add_sum(sub, 'sum_qErr', '+-', [1140 140 1170 180]);
add_gain(sub, 'gain_Kaw', 'dKaw', [1200 145 1260 175]);
add_sum(sub, 'sum_aErr', '+-', [1400 140 1430 180]);
add_gain(sub, 'gain_KawA', 'dKawA', [1460 145 1520 175]);
add_sum(sub, 'sum_idot', '+++', [1560 100 1590 150]);
add_int(sub, 'int_I', '0', [1620 110 1660 140]);
end


function add_allocation(sub)
add_prod(sub, 'prod_desT', '***', [1500 200 1540 250]);
add_sat(sub, 'sat_desT', 'dDeltaMax', '-dDeltaMax', [1560 210 1620 240]);
add_sum(sub, 'sum_slT', '+-', [1660 205 1690 245]);
add_gain(sub, 'gain_slT', 'dShapeKs', [1720 210 1780 240]);
add_sat(sub, 'sat_slT', 'dRateMax', '-dRateMax', [1800 210 1860 240]);
add_int(sub, 'int_cmdT', '0', [1880 210 1920 240]);

add_prod(sub, 'prod_desF', '***', [1500 280 1540 330]);
add_sat(sub, 'sat_desF', 'dFinMax', '-dFinMax', [1560 290 1620 320]);
add_sum(sub, 'sum_slF', '+-', [1660 285 1690 325]);
add_gain(sub, 'gain_slF', 'dShapeKs', [1720 290 1780 320]);
add_sat(sub, 'sat_slF', 'dFinRate', '-dFinRate', [1800 290 1860 320]);
add_int(sub, 'int_cmdF', '0', [1880 290 1920 320]);

add_prod(sub, 'prod_desR', '**', [1500 360 1540 400]);
add_dynsat(sub, 'sat_desR', [1560 350 1620 400]);
add_gain(sub, 'neg_rcs', '-1', [1500 410 1540 440]);
add_sum(sub, 'sum_slR', '+-', [1660 355 1690 395]);
add_gain(sub, 'gain_slR', 'dShapeKs', [1720 360 1780 390]);
add_sat(sub, 'sat_slR', 'dRcsSlew', '-dRcsSlew', [1800 360 1860 390]);
add_int(sub, 'int_cmdR', '0', [1880 360 1920 390]);

add_out(sub, 'cmd_tvc', 1, [1960 214 1990 228]);
add_out(sub, 'cmd_fin', 2, [1960 294 1990 308]);
add_out(sub, 'cmd_rcs', 3, [1960 364 1990 378]);
add_out(sub, 'integ', 4, [1700 114 1730 128]);
add_out(sub, 'shaped', 5, [920 20 950 34]);
add_out(sub, 'theta_cmd', 6, [200 20 230 34]);
end


function add_servo(mdl, name, pos, wn, zeta, rateMax, angMax, color)
sub = add_empty_subsystem(mdl, name, pos);
paint(sub, color);
try_set(sub, 'Description', 'Second-order servo. Integrator saturation is the rate and angle stop.');
add_in(sub, 'cmd', 1, [20 48 50 62]);
add_sum(sub, 'sum_err', '+-', [80 40 110 80]);
add_gain(sub, 'wn2', [wn '^2'], [140 45 210 75]);
add_sum(sub, 'sum_acc', '+-', [240 40 270 90]);
add_std('simulink/Continuous/Integrator', sub, 'int_rate', [300 48 340 82], ...
    'InitialCondition', '0', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', rateMax, 'LowerSaturationLimit', ['-' rateMax]);
add_std('simulink/Continuous/Integrator', sub, 'int_angle', [380 48 420 82], ...
    'InitialCondition', '0', 'LimitOutput', 'on', ...
    'UpperSaturationLimit', angMax, 'LowerSaturationLimit', ['-' angMax]);
add_gain(sub, 'damping', ['2*' zeta '*' wn], [300 110 400 140]);
add_out(sub, 'angle', 1, [460 52 490 66]);
add_out(sub, 'rate', 2, [380 110 410 124]);
wire = @(a, b) add_line(sub, a, b, 'autorouting', 'on');
wire('cmd/1', 'sum_err/1');
wire('int_angle/1', 'sum_err/2');
wire('sum_err/1', 'wn2/1');
wire('wn2/1', 'sum_acc/1');
wire('damping/1', 'sum_acc/2');
wire('sum_acc/1', 'int_rate/1');
wire('int_rate/1', 'int_angle/1');
wire('int_rate/1', 'damping/1');
wire('int_angle/1', 'angle/1');
wire('int_rate/1', 'rate/1');
end


function add_rcs(mdl)
sub = add_empty_subsystem(mdl, 'RCS', [430 360 620 460]);
paint(sub, 'magenta');
try_set(sub, 'Description', 'First-order RCS acceleration. Used for the flip, then blended out.');
add_in(sub, 'cmd', 1, [20 48 50 62]);
add_sum(sub, 'sum_err', '+-', [90 40 120 80]);
add_gain(sub, 'invTau', '1/dRcsTau', [150 45 230 75]);
add_int(sub, 'int_a', '0', [260 48 300 78]);
add_out(sub, 'a_rcs', 1, [340 52 370 66]);
wire = @(a, b) add_line(sub, a, b, 'autorouting', 'on');
wire('cmd/1', 'sum_err/1');
wire('int_a/1', 'sum_err/2');
wire('sum_err/1', 'invTau/1');
wire('invTau/1', 'int_a/1');
wire('int_a/1', 'a_rcs/1');
end


function add_plant(mdl)
sub = add_empty_subsystem(mdl, 'Plant', [700 80 920 320]);
paint(sub, '[0.75,0.93,0.75]');
try_set(sub, 'Description', ...
    'theta'''' = muT*sin(delta) + muF*fin + lambda*alpha + a_rcs. alpha is capped.');
add_in(sub, 'delta_tvc', 1, [20 40 50 54]);
add_in(sub, 'delta_fin', 2, [20 100 50 114]);
add_in(sub, 'a_rcs', 3, [20 160 50 174]);
add_std('simulink/Sources/Clock', sub, 'clock', [20 220 50 250]);
add_lookup(sub, 'lu_muT', 'dMuTvc', [80 20 160 50]);
add_lookup(sub, 'lu_muF', 'dMuFin', [80 70 160 100]);
add_lookup(sub, 'lu_lam', 'dLambda', [80 130 160 160]);
add_lookup(sub, 'lu_trim', 'dThetaTrim', [80 190 160 220]);
add_std('simulink/Math Operations/Trigonometric Function', sub, 'sin_d', ...
    [180 30 220 60], 'Operator', 'sin');
add_prod(sub, 'prod_tvc', '**', [250 25 290 65]);
add_prod(sub, 'prod_fin', '**', [250 85 290 125]);
add_int(sub, 'int_theta', 'dTheta0', [520 40 560 70]);
add_sum(sub, 'sum_alpha', '+-', [300 180 330 220]);
add_sat(sub, 'sat_alpha', 'dAlphaCap', '-dAlphaCap', [360 185 420 215]);
add_prod(sub, 'prod_lam', '**', [450 140 490 180]);
add_sum(sub, 'sum_qdot', '++++', [360 40 390 120]);
add_int(sub, 'int_q', '0', [430 55 470 85]);
add_out(sub, 'theta', 1, [600 44 630 58]);
add_out(sub, 'q', 2, [520 100 550 114]);

wire = @(a, b) add_line(sub, a, b, 'autorouting', 'on');
wire('clock/1', 'lu_muT/1');
wire('clock/1', 'lu_muF/1');
wire('clock/1', 'lu_lam/1');
wire('clock/1', 'lu_trim/1');
wire('delta_tvc/1', 'sin_d/1');
wire('sin_d/1', 'prod_tvc/1');
wire('lu_muT/1', 'prod_tvc/2');
wire('delta_fin/1', 'prod_fin/1');
wire('lu_muF/1', 'prod_fin/2');
wire('int_theta/1', 'sum_alpha/1');
wire('lu_trim/1', 'sum_alpha/2');
wire('sum_alpha/1', 'sat_alpha/1');
wire('lu_lam/1', 'prod_lam/1');
wire('sat_alpha/1', 'prod_lam/2');
wire('prod_tvc/1', 'sum_qdot/1');
wire('prod_fin/1', 'sum_qdot/2');
wire('prod_lam/1', 'sum_qdot/3');
wire('a_rcs/1', 'sum_qdot/4');
wire('sum_qdot/1', 'int_q/1');
wire('int_q/1', 'int_theta/1');
wire('int_theta/1', 'theta/1');
wire('int_q/1', 'q/1');
end


function add_monitoring(mdl)
add_scope(mdl, 'scope_attitude', 2, [980 40 1040 100]);
add_scope(mdl, 'scope_surface', 2, [980 120 1040 180]);
add_log(mdl, 'log_theta', 'dTheta', [980 200 1060 230]);
add_log(mdl, 'log_q', 'dQ', [980 240 1060 270]);
add_log(mdl, 'log_shaped', 'dShaped', [980 280 1060 310]);
add_log(mdl, 'log_thetaCmd', 'dThetaCmdLog', [980 320 1060 350]);
add_log(mdl, 'log_tvc', 'dDeltaTvc', [980 360 1060 390]);
add_log(mdl, 'log_fin', 'dDeltaFin', [980 400 1060 430]);
add_log(mdl, 'log_cmdTvc', 'dCmdTvc', [980 440 1060 470]);
add_log(mdl, 'log_cmdFin', 'dCmdFin', [980 480 1060 510]);
add_log(mdl, 'log_integ', 'dInteg', [980 520 1060 550]);
add_log(mdl, 'log_rcs', 'dRcs', [980 560 1060 590]);
end


function add_scope(parent, name, nports, pos)
add_std('simulink/Sinks/Scope', parent, name, pos);
set_param(blk(parent, name), 'NumInputPorts', num2str(nports));
end


function add_log(parent, name, varName, pos)
path = add_std('simulink/Sinks/To Workspace', parent, name, pos, ...
    'VariableName', varName, 'SampleTime', '-1');
if ~try_param(path, 'SaveFormat', 'Timeseries')
    try_set(path, 'SaveFormat', 'Structure With Time');
end
if ~try_param(path, 'MaxDataPoints', 'inf')
    try_param(path, 'LimitDataPoints', 'off');
end
end


function add_lookup(parent, name, tableName, pos)
% 1-D Lookup Table has no NumberOfTableDimensions parameter.
add_std('simulink/Lookup Tables/1-D Lookup Table', parent, name, pos, ...
    'BreakpointsForDimension1', 'dTime', ...
    'Table', tableName);
path = blk(parent, name);
try_set(path, 'InterpMethod', 'Linear point-slope');
try_set(path, 'ExtrapMethod', 'Clip');
end


function add_prod(parent, name, inputs, pos)
add_std('simulink/Math Operations/Product', parent, name, pos, 'Inputs', inputs);
end


function add_gain(parent, name, value, pos)
add_std('simulink/Math Operations/Gain', parent, name, pos, 'Gain', value);
end


function add_const(parent, name, value, pos)
add_std('simulink/Sources/Constant', parent, name, pos, 'Value', value);
end


function add_int(parent, name, ic, pos)
add_std('simulink/Continuous/Integrator', parent, name, pos, 'InitialCondition', ic);
end


function add_sat(parent, name, up, lo, pos)
add_std('simulink/Discontinuities/Saturation', parent, name, pos, ...
    'UpperLimit', up, 'LowerLimit', lo);
end


function add_dynsat(parent, name, pos)
add_std('simulink/Discontinuities/Saturation Dynamic', parent, name, pos);
end


function add_minmax(parent, name, pos)
add_std('simulink/Math Operations/MinMax', parent, name, pos, ...
    'Function', 'max', 'Inputs', '2');
end


function add_sum(parent, name, signs, pos)
add_std('simulink/Math Operations/Sum', parent, name, pos, 'Inputs', signs);
try_set(blk(parent, name), 'IconShape', 'rectangular');
end


function add_in(parent, name, port, pos)
add_std('simulink/Sources/In1', parent, name, pos, 'Port', num2str(port));
end


function add_out(parent, name, port, pos)
add_std('simulink/Sinks/Out1', parent, name, pos, 'Port', num2str(port));
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


function path = add_std(lib, parent, name, pos, varargin)
path = blk(parent, name);
if isempty(varargin)
    add_block(lib, path);
else
    add_block(lib, path, varargin{:});
end
try_set(path, 'Position', pos);
end


function paint(block, color)
try_set(block, 'BackgroundColor', color);
end


function ok = try_param(block, name, value)
try
    set_param(block, name, value);
    ok = true;
catch
    ok = false;
end
end


function try_set(block, varargin)
for k = 1:2:numel(varargin)
    name = varargin{k};
    value = varargin{k + 1};
    try
        set_param(block, name, value);
    catch ME
        warning('build_descent_model:Param', ...
            'Skipped %s on %s (%s).', name, block, ME.message);
    end
end
end


function arrange_quietly(sys)
try
    Simulink.BlockDiagram.arrangeSystem(sys);
catch ME
    warning('build_descent_model:Arrange', ...
        'Auto-layout of %s was skipped (%s).', sys, ME.message);
end
end


function p = blk(parent, name)
p = [parent '/' name];
end
