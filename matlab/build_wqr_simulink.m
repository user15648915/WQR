%% ========================================================================
%  ГЕНЕРАТОР МОДЕЛИ SIMULINK ДЛЯ РОБОТА WQR v18.6 (wqr_digital_twin.slx)
%  ========================================================================
clear; clc; close all;

model_name = 'wqr_digital_twin';
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end
if isfile([model_name '.slx'])
    delete([model_name '.slx']);
end

new_system(model_name);
open_system(model_name);

set_param(model_name, 'StopTime', '4.0', 'Solver', 'ode4', 'FixedStep', '0.001');

%% КОНТУР 1: ПИД-ПРИВОД СУСТАВА НОГИ (Kp=220, Ki=10, Kd=8, Torque=+-220 Н*м)
add_block('simulink/Sources/Step', [model_name '/q3_Ref_Rad'], ...
    'Position', [40, 100, 80, 130], 'Time', '0.5', 'Before', '-1.0', 'After', '-0.50');

add_block('simulink/Math Operations/Sum', [model_name '/Sum_Joint'], ...
    'Position', [150, 100, 180, 130], 'Inputs', '|+-');

add_block('simulink/Continuous/PID Controller', [model_name '/Leg_PID_v18'], ...
    'Position', [240, 95, 305, 135], 'P', '220', 'I', '10', 'D', '8', 'N', '100');

add_block('simulink/Discontinuities/Saturation', [model_name '/Torque_Limit_220Nm'], ...
    'Position', [370, 100, 410, 130], 'UpperLimit', '220', 'LowerLimit', '-220');

add_block('simulink/Continuous/Transfer Fcn', [model_name '/Leg_Link_Dynamics'], ...
    'Position', [475, 92, 625, 138], 'Numerator', '[1]', 'Denominator', '[0.02 3.0 0]');

add_block('simulink/Signal Routing/Mux', [model_name '/Mux_Joint'], ...
    'Position', [700, 85, 706, 135], 'Inputs', '2');

add_block('simulink/Sinks/Scope', [model_name '/Scope_Joint_Tracking'], ...
    'Position', [765, 93, 805, 127]);

add_block('simulink/Sinks/To Workspace', [model_name '/Log_Joint_Ref'], ...
    'Position', [750, 25, 845, 50], 'VariableName', 'sim_q_ref', 'SaveFormat', 'Timeseries');

add_block('simulink/Sinks/To Workspace', [model_name '/Log_Joint_Act'], ...
    'Position', [750, 155, 845, 180], 'VariableName', 'sim_q_act', 'SaveFormat', 'Timeseries');

add_line(model_name, 'q3_Ref_Rad/1', 'Sum_Joint/1', 'autorouting', 'on');
add_line(model_name, 'Sum_Joint/1', 'Leg_PID_v18/1', 'autorouting', 'on');
add_line(model_name, 'Leg_PID_v18/1', 'Torque_Limit_220Nm/1', 'autorouting', 'on');
add_line(model_name, 'Torque_Limit_220Nm/1', 'Leg_Link_Dynamics/1', 'autorouting', 'on');
add_line(model_name, 'q3_Ref_Rad/1', 'Mux_Joint/1', 'autorouting', 'on');
add_line(model_name, 'Leg_Link_Dynamics/1', 'Mux_Joint/2', 'autorouting', 'on');
add_line(model_name, 'Leg_Link_Dynamics/1', 'Sum_Joint/2', 'autorouting', 'on');
add_line(model_name, 'Mux_Joint/1', 'Scope_Joint_Tracking/1', 'autorouting', 'on');
add_line(model_name, 'q3_Ref_Rad/1', 'Log_Joint_Ref/1', 'autorouting', 'on');
add_line(model_name, 'Leg_Link_Dynamics/1', 'Log_Joint_Act/1', 'autorouting', 'on');

%% КОНТУР 2: СТАБИЛИЗАЦИЯ ОБРАТНОГО МАЯТНИКА НА 2 КОЛЕСАХ (BIPED LQR/PID)
add_block('simulink/Sources/Step', [model_name '/Disturbance_Push'], ...
    'Position', [40, 290, 80, 320], 'Time', '0.8', 'Before', '0.18', 'After', '0.0');

add_block('simulink/Math Operations/Sum', [model_name '/Sum_Pitch'], ...
    'Position', [150, 290, 180, 320], 'Inputs', '|+-');

add_block('simulink/Continuous/PID Controller', [model_name '/Biped_Balance_Reg'], ...
    'Position', [240, 285, 305, 325], 'P', '22.0', 'I', '2.0', 'D', '4.5', 'N', '50');

add_block('simulink/Continuous/Transfer Fcn', [model_name '/Inverted_Pendulum_Body'], ...
    'Position', [390, 282, 550, 328], 'Numerator', '[1.2]', 'Denominator', '[0.55 1.8 -3.2]');

add_block('simulink/Sinks/Scope', [model_name '/Scope_Biped_Pitch_Rad'], ...
    'Position', [650, 288, 690, 322]);

add_block('simulink/Sinks/To Workspace', [model_name '/Log_Biped_Pitch'], ...
    'Position', [650, 355, 750, 380], 'VariableName', 'sim_pitch', 'SaveFormat', 'Timeseries');

add_line(model_name, 'Disturbance_Push/1', 'Sum_Pitch/1', 'autorouting', 'on');
add_line(model_name, 'Sum_Pitch/1', 'Biped_Balance_Reg/1', 'autorouting', 'on');
add_line(model_name, 'Biped_Balance_Reg/1', 'Inverted_Pendulum_Body/1', 'autorouting', 'on');
add_line(model_name, 'Inverted_Pendulum_Body/1', 'Sum_Pitch/2', 'autorouting', 'on');
add_line(model_name, 'Inverted_Pendulum_Body/1', 'Scope_Biped_Pitch_Rad/1', 'autorouting', 'on');
add_line(model_name, 'Inverted_Pendulum_Body/1', 'Log_Biped_Pitch/1', 'autorouting', 'on');

set_param(model_name, 'ZoomFactor', 'FitSystem');
save_system(model_name, [model_name '.slx']);

try
    print(['-s' model_name], '-dpng', '-r300', 'wqr_simulink_diagram.png');
    fprintf('Схема Simulink сохранена: %s\n', fullfile(pwd, 'wqr_simulink_diagram.png'));
catch
end

%% ЗАПУСК СИМУЛЯЦИИ И СВЕТЛЫЕ ГРАФИКИ ПЕРЕХОДНЫХ ПРОЦЕССОВ
out = sim(model_name);

if isa(out, 'Simulink.SimulationOutput')
    q_ref_ts = out.get('sim_q_ref');
    q_act_ts = out.get('sim_q_act');
    pitch_ts = out.get('sim_pitch');
else
    q_ref_ts = sim_q_ref;
    q_act_ts = sim_q_act;
    pitch_ts = sim_pitch;
end

fig = figure('Name', 'Simulink Digital Twin — Переходные процессы WQR v18.6', ...
             'Color', 'w', 'Position', [120 120 1180 480]);
try, theme(fig, 'light'); catch, end
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(q_ref_ts.Time, q_ref_ts.Data, 'k--', 'LineWidth', 1.6); hold on;
plot(q_act_ts.Time, q_act_ts.Data, 'b-',  'LineWidth', 2.0);
xlim([0.35 1.8]); ylim([-1.05 -0.45]);
grid on; xlabel('Время (с)'); ylabel('Угол колена q_3 (рад)');
title('Контур 1: Ступенька ПИД-привода (Kp=220, Ki=10, Kd=8)');
legend('Задание (ref)', 'Реакция сустава (act)', 'Location', 'southeast');

nexttile;
plot(pitch_ts.Time, rad2deg(pitch_ts.Data), 'r-', 'LineWidth', 2.0); hold on;
yline(0, 'k--', 'LineWidth', 1.2);
xlim([0 2.5]);
grid on; xlabel('Время (с)'); ylabel('Отклонение тангажа Pitch (град)');
title('Контур 2: Стабилизация обратного маятника (Biped)');
legend('Угол тангажа Pitch', 'Положение равновесия (0°)', 'Location', 'northeast');

set(findall(fig, 'Type', 'axes'), 'FontSize', 10, 'LineWidth', 0.9);
try
    exportgraphics(fig, 'wqr_simulink_results.png', 'Resolution', 300, 'BackgroundColor', 'white');
catch
    saveas(fig, 'wqr_simulink_results.png');
end
fprintf('Графики Simulink сохранены: %s\n', fullfile(pwd, 'wqr_simulink_results.png'));
