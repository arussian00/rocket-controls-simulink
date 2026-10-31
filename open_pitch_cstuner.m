function open_pitch_cstuner(opts)
%OPEN_PITCH_CSTUNER  Open Control System Tuner on the pitch cascade.
%
%   open_pitch_cstuner
%   open_pitch_cstuner(Kp=1.1, Ki=0.03, Kd=2.1)
%
%   Builds boosterPitchTVC if needed and opens Control System Tuner on
%   Controller/gain_Kq, gain_Kth and gain_KiTh, starting from the gains
%   you pass or from the current design. The Gain parameters are set to
%   those numbers for the tuner. The slx on disk is not saved. run_pitch_tvc
%   rebuilds it with Gain set to the workspace names Kq, Kth and KiTh.
%   Close the model without saving if you opened an existing slx.
%
%   The app constructor does not take goals. When TuningGoal exists, the
%   soft and hard goals are left in the base workspace as pitchTunerSoft
%   and pitchTunerHard (6 dB / 30 deg hard, 4 s tracking, gimbal gain,
%   10 dB / 45 deg soft). Add those in the app if this release does not
%   import them. systune itself is tune_with_systune, and it does not
%   write vehicle_params.m.
%
%   Without Simulink Control Design the function prints the goals and
%   points at open_pitch_tuner("pid","max-q").

arguments
    opts.Kp (1,1) double = NaN
    opts.Ki (1,1) double = NaN
    opts.Kd (1,1) double = NaN
end

if ~license('test', 'Simulink')
    error('open_pitch_cstuner:NeedsSimulink', ...
        'Control System Tuner opens the Simulink model. Simulink is not licensed.');
end

V = vehicle_params();
D = design_pitch_controller(V, plot=false, verbose=false);
if ~isfinite(opts.Kp), opts.Kp = D.Kp; end
if ~isfinite(opts.Ki), opts.Ki = D.Ki; end
if ~isfinite(opts.Kd), opts.Kd = D.Kd; end
Kq = opts.Kd;
Kth = opts.Kp / max(opts.Kd, 1e-9);
KiTh = opts.Ki / max(opts.Kd, 1e-9);

mdl = 'boosterPitchTVC';
if ~isfile(fullfile(pwd, [mdl '.slx']))
    build_pitch_model();
elseif ~bdIsLoaded(mdl)
    load_system(mdl);
end

blocks = { ...
    [mdl '/Controller/gain_Kq'], ...
    [mdl '/Controller/gain_Kth'], ...
    [mdl '/Controller/gain_KiTh']};
values = {num2str(Kq, '%.8g'), num2str(Kth, '%.8g'), num2str(KiTh, '%.8g')};
for k = 1:numel(blocks)
    try_set(blocks{k}, 'Gain', values{k});
end
try_set([mdl '/Controller'], 'Description', ...
    ['Cascade Kq, Kth, KiTh. Control System Tuner goals are in ' ...
     'pitchTunerSoft and pitchTunerHard.']);

publish_goals();
fprintf(['\nControl System Tuner blocks: gain_Kq, gain_Kth, gain_KiTh\n' ...
    'Starting gains: Kq %.4g, Kth %.4g, KiTh %.4g (Kp %.4g, Ki %.4g, Kd %.4g)\n' ...
    'Hard goal: 6 dB and 30 deg at a_cmd. Soft: 4 s tracking, |delta/theta_cmd| < 3,\n' ...
    '10 dB and 45 deg. Workspace variables pitchTunerSoft and pitchTunerHard.\n\n'], ...
    Kq, Kth, KiTh, opts.Kp, opts.Ki, opts.Kd);

opened = open_tuner(mdl);
fprintf(['In-memory Gain values are the numbers above so the tuner starts there.\n' ...
    'They are not saved. run_pitch_tvc rebuilds the slx with Gain set back to Kq, Kth, KiTh.\n' ...
    'Close the model without saving if you do not want the numeric gains kept.\n\n']);
if ~opened
    fprintf(['controlSystemTuner is not on this path (Simulink Control Design).\n' ...
        'The goals above are still in the workspace. PID Tuner:\n' ...
        '  open_pitch_tuner("pid", "max-q")\n\n']);
end
end


function opened = open_tuner(mdl)
opened = false;
blocks = {'Controller/gain_Kq', 'Controller/gain_Kth', 'Controller/gain_KiTh'};
if exist('slTuner', 'file') == 2 && exist('controlSystemTuner', 'file') == 2
    try
        st = slTuner(mdl, blocks);
        try
            addPoint(st, {'theta_cmd', 'theta', 'delta_cmd'});
        catch
        end
        controlSystemTuner(st);
        opened = true;
        return
    catch ME
        warning('open_pitch_cstuner:SlTuner', ...
            'slTuner did not open the app (%s). Trying the model form.', ME.message);
    end
end
if exist('controlSystemTuner', 'file') == 2
    try
        controlSystemTuner(mdl, blocks);
        opened = true;
    catch ME
        warning('open_pitch_cstuner:App', ...
            'controlSystemTuner did not open (%s).', ME.message);
    end
end
end


function publish_goals()
soft = [];
hard = [];
try
    hard = TuningGoal.Margins('a_cmd', 6, 30);
    soft = [ ...
        TuningGoal.StepTracking('theta_cmd', 'theta', 4), ...
        TuningGoal.Gain('theta_cmd', 'delta', 3), ...
        TuningGoal.Margins('a_cmd', 10, 45)];
catch ME
    warning('open_pitch_cstuner:Goals', ...
        ['TuningGoal is not available (%s). The numeric goals are printed ' ...
         'above and used by tune_with_systune when systune exists.'], ME.message);
    return
end
assignin('base', 'pitchTunerHard', hard);
assignin('base', 'pitchTunerSoft', soft);
end


function try_set(block, name, value)
% Cosmetic or best-effort dialog settings. A miss warns and continues.
try
    set_param(block, name, value);
catch ME
    warning('open_pitch_cstuner:Param', ...
        'Skipped %s on %s (%s).', name, block, ME.message);
end
end
