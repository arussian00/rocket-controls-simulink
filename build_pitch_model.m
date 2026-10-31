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
%   Controller: command prefilter, sensor delay, PID on angular
%   acceleration (derivative on measured rate, back-calculation
%   anti-windup) and the 1/mu gain schedule.
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
    'ReturnWorkspaceOutputs', 'on', ...
    'SimulationMode', 'normal', ...
    'Description', ['Super Heavy booster pitch attitude control through ' ...
        'thrust-vector control. Built by build_pitch_model.m.'], ...
    'PreLoadFcn', 'addpath(fileparts(get_param(bdroot,''FileName'')))', ...
    'InitFcn', 'pitch_model_init');

add_block('simulink/Sources/Step', blk(mdl, 'theta_cmd'), ...
    'Position', [40 148 90 182], ...
    'Time', 'thetaStepTime', 'Before', '0', 'After', 'thetaStep');

add_controller(mdl);
add_actuator(mdl);
add_plant(mdl);
add_monitoring(mdl);

add_line(mdl, 'theta_cmd/1', 'Controller/1', 'autorouting', 'on');
add_line(mdl, 'Controller/1', 'Actuator/1', 'autorouting', 'on');
add_line(mdl, 'Actuator/1', 'Plant/1', 'autorouting', 'on');
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
sub = add_empty_subsystem(mdl, 'Controller', [160 110 360 250]);
set_param(sub, 'BackgroundColor', 'lightBlue', ...
    'Description', ['Prefilter, sensor delay, PID on angular acceleration, ' ...
        '1/mu gimbal schedule.']);

add_in(sub, 'theta_cmd', 1, [30 38 60 52]);
add_in(sub, 'theta', 2, [30 148 60 162]);
add_in(sub, 'q', 3, [30 248 60 262]);

add_block('simulink/Continuous/Transfer Fcn', blk(sub, 'cmd_prefilter'), ...
    'Position', [110 25 200 65], ...
    'Numerator', '[1]', 'Denominator', '[cmdTau 1]');
add_block('simulink/Continuous/Transport Delay', blk(sub, 'delay_theta'), ...
    'Position', [110 135 200 175], ...
    'DelayTime', 'sensorDelay', 'InitialOutput', '0');
add_block('simulink/Continuous/Transport Delay', blk(sub, 'delay_q'), ...
    'Position', [110 235 200 275], ...
    'DelayTime', 'sensorDelay', 'InitialOutput', '0');
add_block('simulink/Continuous/Transfer Fcn', blk(sub, 'rate_filter'), ...
    'Position', [240 235 330 275], ...
    'Numerator', '[1]', 'Denominator', '[Tf 1]');

add_sum(sub, 'sum_e', '+-', [250 30 280 70]);
add_block('simulink/Math Operations/Gain', blk(sub, 'Kp'), ...
    'Position', [340 15 400 45], 'Gain', 'Kp');
add_block('simulink/Math Operations/Gain', blk(sub, 'Ki'), ...
    'Position', [340 70 400 100], 'Gain', 'Ki');
add_block('simulink/Math Operations/Gain', blk(sub, 'Kd'), ...
    'Position', [380 235 440 265], 'Gain', '-Kd');

add_sum(sub, 'sum_idot', '++', [460 75 490 115]);
add_block('simulink/Continuous/Integrator', blk(sub, 'int_I'), ...
    'Position', [530 80 570 110], 'InitialCondition', '0');

add_sum(sub, 'sum_a', '+++', [640 40 670 100]);
add_block('simulink/Sources/Clock', blk(sub, 'clock'), ...
    'Position', [500 300 540 330]);
add_lookup(sub, 'lookup_aMax', 'schedACmdMax', [580 250 700 290]);
add_lookup(sub, 'lookup_invMu', 'schedInvMu', [580 310 700 350]);
add_block('simulink/Math Operations/Gain', blk(sub, 'neg_aMax'), ...
    'Position', [740 250 790 280], 'Gain', '-1');
add_block('simulink/Discontinuities/Saturation Dynamic', blk(sub, 'sat_a'), ...
    'Position', [760 40 820 110]);
add_block('simulink/Math Operations/Product', blk(sub, 'prod_invMu'), ...
    'Position', [860 40 900 80], 'Inputs', '**');
add_sum(sub, 'sum_aw', '+-', [800 150 830 190]);
add_block('simulink/Math Operations/Gain', blk(sub, 'Kaw'), ...
    'Position', [620 160 680 190], 'Gain', 'Kaw');

add_out(sub, 'delta_cmd', 1, [920 58 950 72]);

add_line(sub, 'theta_cmd/1', 'cmd_prefilter/1', 'autorouting', 'on');
add_line(sub, 'cmd_prefilter/1', 'sum_e/1', 'autorouting', 'on');
add_line(sub, 'theta/1', 'delay_theta/1', 'autorouting', 'on');
add_line(sub, 'delay_theta/1', 'sum_e/2', 'autorouting', 'on');
add_line(sub, 'sum_e/1', 'Kp/1', 'autorouting', 'on');
add_line(sub, 'sum_e/1', 'Ki/1', 'autorouting', 'on');
add_line(sub, 'q/1', 'delay_q/1', 'autorouting', 'on');
add_line(sub, 'delay_q/1', 'rate_filter/1', 'autorouting', 'on');
add_line(sub, 'rate_filter/1', 'Kd/1', 'autorouting', 'on');

add_line(sub, 'Ki/1', 'sum_idot/1', 'autorouting', 'on');
add_line(sub, 'Kaw/1', 'sum_idot/2', 'autorouting', 'on');
add_line(sub, 'sum_idot/1', 'int_I/1', 'autorouting', 'on');

add_line(sub, 'Kp/1', 'sum_a/1', 'autorouting', 'on');
add_line(sub, 'Kd/1', 'sum_a/2', 'autorouting', 'on');
add_line(sub, 'int_I/1', 'sum_a/3', 'autorouting', 'on');
add_line(sub, 'clock/1', 'lookup_aMax/1', 'autorouting', 'on');
add_line(sub, 'clock/1', 'lookup_invMu/1', 'autorouting', 'on');
add_line(sub, 'lookup_aMax/1', 'sat_a/1', 'autorouting', 'on');
add_line(sub, 'lookup_aMax/1', 'neg_aMax/1', 'autorouting', 'on');
add_line(sub, 'neg_aMax/1', 'sat_a/3', 'autorouting', 'on');
add_line(sub, 'sum_a/1', 'sat_a/2', 'autorouting', 'on');
add_line(sub, 'sat_a/1', 'prod_invMu/1', 'autorouting', 'on');
add_line(sub, 'lookup_invMu/1', 'prod_invMu/2', 'autorouting', 'on');
add_line(sub, 'prod_invMu/1', 'delta_cmd/1', 'autorouting', 'on');

add_line(sub, 'sat_a/1', 'sum_aw/1', 'autorouting', 'on');
add_line(sub, 'sum_a/1', 'sum_aw/2', 'autorouting', 'on');
add_line(sub, 'sum_aw/1', 'Kaw/1', 'autorouting', 'on');
end


function add_actuator(mdl)
sub = add_empty_subsystem(mdl, 'Actuator', [430 110 630 250]);
set_param(sub, 'BackgroundColor', 'orange', ...
    'Description', ['Second-order gimbal servo. Integrator saturation is ' ...
        'the rate limit and the angle limit.']);

add_in(sub, 'delta_cmd', 1, [30 78 60 92]);
add_sum(sub, 'sum_err', '+-', [120 70 150 110]);
add_block('simulink/Math Operations/Gain', blk(sub, 'wn2'), ...
    'Position', [190 75 250 105], 'Gain', 'actWn^2');
add_sum(sub, 'sum_acc', '+-', [300 70 330 120]);
add_block('simulink/Continuous/Integrator', blk(sub, 'int_rate'), ...
    'Position', [380 78 420 112], ...
    'InitialCondition', '0', ...
    'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'gimbalRate', ...
    'LowerSaturationLimit', '-gimbalRate');
add_block('simulink/Continuous/Integrator', blk(sub, 'int_angle'), ...
    'Position', [480 78 520 112], ...
    'InitialCondition', '0', ...
    'LimitOutput', 'on', ...
    'UpperSaturationLimit', 'deltaMax', ...
    'LowerSaturationLimit', '-deltaMax');
add_block('simulink/Math Operations/Gain', blk(sub, 'damping'), ...
    'Position', [380 160 460 190], 'Gain', '2*actZeta*actWn');

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
set_param(sub, 'BackgroundColor', 'lightGreen', ...
    'Description', ['Rigid pitch: theta'''' = mu*sin(delta+mis) + ' ...
        'lambda*theta + gustAccPerWind*wind.']);

add_in(sub, 'delta', 1, [30 48 60 62]);
add_block('simulink/Sources/Step', blk(sub, 'misalign'), ...
    'Position', [30 110 80 140], ...
    'Time', 'misalignTime', 'Before', '0', 'After', 'misRad');
add_block('simulink/Sources/Step', blk(sub, 'wind'), ...
    'Position', [30 250 80 280], ...
    'Time', 'gustTime', 'Before', '0', 'After', 'gustSpeed');

add_sum(sub, 'sum_angle', '++', [140 45 170 95]);
add_block('simulink/Math Operations/Trigonometric Function', blk(sub, 'sin_angle'), ...
    'Position', [210 55 260 85], 'Operator', 'sin');
add_block('simulink/Sources/Clock', blk(sub, 'clock'), ...
    'Position', [210 300 250 330]);
add_lookup(sub, 'lookup_mu', 'schedMu', [280 290 400 330]);
add_lookup(sub, 'lookup_lambda', 'schedLambda', [400 200 520 240]);
add_block('simulink/Math Operations/Product', blk(sub, 'prod_mu'), ...
    'Position', [430 50 470 90], 'Inputs', '**');
add_block('simulink/Math Operations/Gain', blk(sub, 'gust_gain'), ...
    'Position', [140 245 220 275], 'Gain', 'gustAccPerWind');
add_block('simulink/Math Operations/Product', blk(sub, 'prod_lambda'), ...
    'Position', [560 160 600 200], 'Inputs', '**');

add_sum(sub, 'sum_qdot', '+++', [420 50 450 120]);
add_block('simulink/Continuous/Integrator', blk(sub, 'int_q'), ...
    'Position', [500 70 540 100], 'InitialCondition', '0');
add_block('simulink/Continuous/Integrator', blk(sub, 'int_theta'), ...
    'Position', [600 70 640 100], 'InitialCondition', '0');

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
end


function add_scope(parent, name, nports, pos)
add_block('simulink/Sinks/Scope', blk(parent, name), 'Position', pos);
set_param(blk(parent, name), 'NumInputPorts', num2str(nports));
end


function add_log(parent, name, varName, pos)
add_block('simulink/Sinks/To Workspace', blk(parent, name), ...
    'Position', pos, ...
    'VariableName', varName, ...
    'SaveFormat', 'Timeseries', ...
    'LimitDataPoints', 'off', ...
    'SampleTime', '-1');
end


function sub = add_empty_subsystem(parent, name, pos)
sub = blk(parent, name);
add_block('simulink/Ports & Subsystems/Subsystem', sub, 'Position', pos);
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
add_block('simulink/Sources/In1', blk(parent, name), ...
    'Position', pos, 'Port', num2str(port));
end


function add_out(parent, name, port, pos)
add_block('simulink/Sinks/Out1', blk(parent, name), ...
    'Position', pos, 'Port', num2str(port));
end


function add_sum(parent, name, signs, pos)
add_block('simulink/Math Operations/Sum', blk(parent, name), ...
    'Position', pos, 'Inputs', signs, 'IconShape', 'rectangular');
end


function add_lookup(parent, name, tableName, pos)
% 1-D lookup versus schedTime. ExtrapMethod Clip holds the end value.
add_block('simulink/Lookup Tables/1-D Lookup Table', blk(parent, name), ...
    'Position', pos, ...
    'NumberOfTableDimensions', '1', ...
    'BreakpointsForDimension1', 'schedTime', ...
    'Table', tableName, ...
    'InterpMethod', 'Linear point-slope', ...
    'ExtrapMethod', 'Clip');
end


function annotate(mdl)
try
    note = Simulink.Annotation(blk(mdl, 'Overview'));
    note.Text = ['Pitch TVC: command -> controller -> actuator -> plant.' newline ...
        'PID gains are rad/s^2 per rad. inv_mu converts that to gimbal angle.' newline ...
        'Workspace variables come from apply_pitch_condition. Run run_pitch_tvc.'];
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
