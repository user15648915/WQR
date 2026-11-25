%% ========================================================================
%  WHEELED-QUADRUPED ROBOT (WQR v18.6) — ФИНАЛЬНЫЙ АНАЛИЗ ТЕЛЕМЕТРИИ MATLAB
%  Строит 6-панельный дашборд + анализ активной подвески IMU и сравнение
%  сред (P, CoT), сохраняет PNG 300 dpi на белом фоне для отчета и защиты.
%  ========================================================================
clear; clc; close all;

csv_file = 'telemetry_log.csv';
if ~isfile(csv_file)
    alt_file = fullfile('..', 'telemetry_log.csv');
    if isfile(alt_file)
        csv_file = alt_file;
    else
        error('Файл telemetry_log.csv не найден! Нажми кнопку экспорта в пульте робота.');
    end
end

opts = detectImportOptions(csv_file, 'Encoding', 'UTF-8');
if any(strcmp(opts.VariableNames, 'surface_type'))
    opts = setvartype(opts, 'surface_type', 'string');
end
T = readtable(csv_file, opts);

calc_rms = @(v) sqrt(mean(v(~isnan(v)).^2));

t = T.time_s - T.time_s(1);
dist_2d    = sum(sqrt(diff(T.x_m).^2 + diff(T.y_m).^2));
dist_3d    = sum(sqrt(diff(T.x_m).^2 + diff(T.y_m).^2 + diff(T.z_m).^2));
rms_roll   = calc_rms(T.roll_deg);
rms_pitch  = calc_rms(T.pitch_deg);
q2_err     = T.q2_ref_rad - T.q2_act_rad;
q3_err     = T.q3_ref_rad - T.q3_act_rad;
rms_q2_err = calc_rms(q2_err);
rms_q3_err = calc_rms(q3_err);

has_susp = any(strcmp(T.Properties.VariableNames, 'suspension_corr_mm'));
if has_susp
    susp_mm = T.suspension_corr_mm;
    max_susp = max(susp_mm);
    mean_susp_active = mean(susp_mm(susp_mm > 0.5));
    if isnan(mean_susp_active), mean_susp_active = 0.0; end
else
    susp_mm = zeros(height(T), 1);
    max_susp = 0.0;
    mean_susp_active = 0.0;
end

fprintf('========================================================================\n');
fprintf('  ИТОГОВЫЙ ОТЧЕТ WQR v18.6 (Выборок: %d | Длительность: %.2f с)\n', height(T), t(end));
fprintf('========================================================================\n');
fprintf('  Пройденный путь (2D / 3D):         %.2f м / %.2f м\n', dist_2d, dist_3d);
fprintf('  Диапазон высоты корпуса Z:         %.3f .. %.3f м\n', min(T.z_m), max(T.z_m));
fprintf('  СКО колебаний корпуса (RMS):       Roll = %.2f° | Pitch = %.2f°\n', rms_roll, rms_pitch);
fprintf('  СКО слежения приводов (ПИД ног):   q2 = %.4f рад | q3 = %.4f рад\n', rms_q2_err, rms_q3_err);
fprintf('  Активная подвеска IMU (макс/ср):   %.1f мм / %.1f мм\n', max_susp, mean_susp_active);
fprintf('  Средний динам. запас ZMP:          %.1f %% (мин: %.1f %%)\n', mean(T.zmp_margin_pct), min(T.zmp_margin_pct));
fprintf('  Средняя мощность P / CoT:          %.1f Вт / %.3f\n', mean(T.power_w), mean(T.cot));
fprintf('------------------------------------------------------------------------\n');

surfs = strings(0);
surf_p = [];
surf_cot = [];
if any(strcmp(T.Properties.VariableNames, 'surface_type'))
    surfs = unique(T.surface_type, 'stable');
    fprintf('  СВОДКА ПО ТИПАМ ПОВЕРХНОСТЕЙ:\n');
    fprintf('  %-26s | %5s | %8s | %8s | %7s | %5s | %6s\n', ...
        'Поверхность', 'Точек', 'RMS Roll', 'RMS Ptch', 'P (Вт)', 'CoT', 'ZMP %');
    for k = 1:numel(surfs)
        idx = (T.surface_type == surfs(k));
        p_m = mean(T.power_w(idx));
        c_m = mean(T.cot(idx));
        z_m = mean(T.zmp_margin_pct(idx));
        surf_p(end+1) = p_m; %#ok<AGROW>
        surf_cot(end+1) = c_m; %#ok<AGROW>
        fprintf('  %-26s | %5d | %6.2f°  | %6.2f°  | %7.1f | %5.2f | %5.1f%%\n', ...
            char(surfs(k)), sum(idx), calc_rms(T.roll_deg(idx)), ...
            calc_rms(T.pitch_deg(idx)), p_m, c_m, z_m);
    end
end
fprintf('========================================================================\n');

%% РИСУНОК 1: 6-ПАНЕЛЬНЫЙ ИНЖЕНЕРНЫЙ ДАШБОРД
fig1 = figure('Name', 'WQR v18.6 — Дашборд испытаний ROS 2 / Gazebo', ...
              'Color', 'w', 'Position', [50 40 1440 840]);
try, theme(fig1, 'light'); catch, end
tiledlayout(2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

% 1. 3D-траектория
nexttile;
plot3(T.x_m, T.y_m, T.z_m, 'b-', 'LineWidth', 2.0); hold on;
plot3(T.x_m(1), T.y_m(1), T.z_m(1), 'go', 'MarkerSize', 9, 'MarkerFaceColor', 'g');
plot3(T.x_m(end), T.y_m(end), T.z_m(end), 'rs', 'MarkerSize', 9, 'MarkerFaceColor', 'r');
grid on; view(35, 28);
xlabel('X (м)'); ylabel('Y (м)'); zlabel('Z (м)');
title('1. 3D-траектория и рельеф (лестница / подиум)');
legend('Траектория', 'Старт', 'Финиш', 'Location', 'best');

% 2. Углы ориентации IMU
nexttile;
plot(t, T.roll_deg, 'r-', 'LineWidth', 1.3); hold on;
plot(t, T.pitch_deg, 'b-', 'LineWidth', 1.3);
yline(0, 'k--'); grid on;
xlabel('Время (с)'); ylabel('Угол (град)');
title('2. Ориентация корпуса по IMU (Roll / Pitch)');
legend('Крен (Roll)', 'Тангаж (Pitch)', 'Location', 'best');

% 3. Слежение приводов лапы (ПИД Kp=220, Ki=10, Kd=8)
nexttile;
plot(t, T.q2_ref_rad, 'b--', 'LineWidth', 1.2); hold on;
plot(t, T.q2_act_rad, 'b-',  'LineWidth', 1.5);
plot(t, T.q3_ref_rad, 'r--', 'LineWidth', 1.2);
plot(t, T.q3_act_rad, 'r-',  'LineWidth', 1.5);
grid on;
xlabel('Время (с)'); ylabel('Угол (рад)');
title('3. Точность приводов лапы (ref vs act)');
legend('q_2 ref (бедро)', 'q_2 act', 'q_3 ref (колено)', 'q_3 act', 'Location', 'best');

% 4. Высота Z и динамический запас устойчивости ZMP
nexttile;
yyaxis left;
plot(t, T.z_m, 'Color', [0.0 0.45 0.74], 'LineWidth', 1.6);
ylabel('Высота центра корпуса Z (м)');
yyaxis right;
plot(t, T.zmp_margin_pct, 'Color', [0.0 0.55 0.2], 'LineWidth', 1.4);
ylabel('Динамический запас ZMP (%)'); ylim([20 105]);
grid on; xlabel('Время (с)');
title('4. Профиль высоты и динамический ZMP (IMU)');

% 5. 3D-зрение PCL
nexttile;
obs_d = T.obs_dist_m; obs_d(obs_d > 4.5) = NaN;
obs_h = T.obs_height_m * 100; obs_h(T.obs_dist_m > 4.5) = NaN;
yyaxis left;
plot(t, obs_d, 'm-', 'LineWidth', 1.4);
ylabel('Дистанция до объекта PCL (м)');
yyaxis right;
plot(t, obs_h, 'Color', [0.15 0.15 0.15], 'LineWidth', 1.3);
ylabel('Высота объекта (см)');
grid on; xlabel('Время (с)');
title('5. Детектор препятствий 3D-камеры PCL');

% 6. Реальная мощность и CoT
nexttile;
yyaxis left;
plot(t, T.power_w, 'Color', [0.1 0.3 0.8], 'LineWidth', 1.3);
ylabel('Мощность приводов P (Вт)');
yyaxis right;
plot(t, T.cot, 'Color', [0.85 0.2 0.15], 'LineWidth', 1.5);
ylabel('Cost of Transport (CoT)');
grid on; xlabel('Время (с)');
title('6. Реальное энергопотребление P и CoT');

set(findall(fig1, 'Type', 'axes'), 'FontSize', 10, 'LineWidth', 0.9);
try
    exportgraphics(fig1, 'wqr_dashboard.png', 'Resolution', 300, 'BackgroundColor', 'white');
catch
    saveas(fig1, 'wqr_dashboard.png');
end
fprintf('Сохранено: %s\n', fullfile(pwd, 'wqr_dashboard.png'));

%% РИСУНОК 2: АКТИВНАЯ ПОДВЕСКА IMU И СРАВНЕНИЕ ПОВЕРХНОСТЕЙ
fig2 = figure('Name', 'WQR v18.6 — Активная подвеска IMU и энергоэффективность по средам', ...
              'Color', 'w', 'Position', [90 90 1280 500]);
try, theme(fig2, 'light'); catch, end
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
yyaxis left;
plot(t, susp_mm, 'Color', [0.0 0.5 0.25], 'LineWidth', 1.6);
ylabel('Коррекция длины лап IMU (мм)');
yyaxis right;
plot(t, abs(T.roll_deg) + abs(T.pitch_deg), 'Color', [0.85 0.33 0.1], 'LineWidth', 1.1);
ylabel('Суммарный наклон |Roll|+|Pitch| (град)');
grid on; xlabel('Время (с)');
title('Работа активной подвески IMU на неровностях и лестнице');
legend('Компенсация подвески (мм)', '|Roll| + |Pitch| (град)', 'Location', 'best');

nexttile;
if ~isempty(surfs)
    x_cat = categorical(cellstr(surfs));
    x_cat = reordercats(x_cat, cellstr(surfs));
    yyaxis left;
    bar(x_cat, surf_p, 0.45, 'FaceColor', [0.15 0.45 0.78]);
    ylabel('Средняя мощность P (Вт)');
    yyaxis right;
    plot(x_cat, surf_cot, '-o', 'Color', [0.85 0.2 0.15], 'LineWidth', 2.0, 'MarkerFaceColor', [0.85 0.2 0.15]);
    ylabel('Средний CoT');
    grid on;
    title('Сравнение мощности и CoT по типам поверхностей');
    legend('Мощность P (Вт)', 'Cost of Transport', 'Location', 'best');
end

set(findall(fig2, 'Type', 'axes'), 'FontSize', 10, 'LineWidth', 0.9);
try
    exportgraphics(fig2, 'wqr_imu_suspension.png', 'Resolution', 300, 'BackgroundColor', 'white');
catch
    saveas(fig2, 'wqr_imu_suspension.png');
end
fprintf('Сохранено: %s\n', fullfile(pwd, 'wqr_imu_suspension.png'));
