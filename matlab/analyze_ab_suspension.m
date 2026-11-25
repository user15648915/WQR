%% A/B-СРАВНЕНИЕ АКТИВНОЙ ПОДВЕСКИ IMU ПО КООРДИНАТЕ X (ЗОНА БУГРОВ)
clear; clc; close all;
Ton  = readtable('ab_susp_on.csv');
Toff = readtable('ab_susp_off.csv');

ion  = (Ton.x_m  >= 1.8 & Ton.x_m  <= 7.8);
ioff = (Toff.x_m >= 1.8 & Toff.x_m <= 7.8);
calc_rms = @(v) sqrt(mean(v.^2));

r_on = calc_rms(Ton.roll_deg(ion));   r_off = calc_rms(Toff.roll_deg(ioff));
p_on = calc_rms(Ton.pitch_deg(ion));  p_off = calc_rms(Toff.pitch_deg(ioff));

fig = figure('Name', 'A/B тест активной подвески IMU', 'Color', 'w', 'Position', [80 80 1280 460]);
try, theme(fig, 'light'); catch, end
tiledlayout(1, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(Toff.x_m(ioff), Toff.roll_deg(ioff), 'r--', 'LineWidth', 1.2); hold on;
plot(Ton.x_m(ion),   Ton.roll_deg(ion),   'b-',  'LineWidth', 1.4);
xlim([1.8 7.8]); grid on; xlabel('Координата X вдоль полосы (м)'); ylabel('Крен Roll (град)');
title('Крен корпуса на буграх (Roll)');
legend('Без подвески IMU', 'С активной подвеской', 'Location', 'best');

nexttile;
plot(Toff.x_m(ioff), Toff.pitch_deg(ioff), 'r--', 'LineWidth', 1.2); hold on;
plot(Ton.x_m(ion),   Ton.pitch_deg(ion),   'b-',  'LineWidth', 1.4);
xlim([1.8 7.8]); grid on; xlabel('Координата X вдоль полосы (м)'); ylabel('Тангаж Pitch (град)');
title('Тангаж корпуса на буграх (Pitch)');
legend('Без подвески IMU', 'С активной подвеской', 'Location', 'best');

nexttile;
cats = categorical({'RMS Roll (°)', 'RMS Pitch (°)'});
cats = reordercats(cats, {'RMS Roll (°)', 'RMS Pitch (°)'});
b = bar(cats, [r_off, r_on; p_off, p_on]);
b(1).FaceColor = [0.85 0.25 0.20];
b(2).FaceColor = [0.12 0.48 0.78];
grid on; ylabel('СКО колебаний (град)');
title('Сравнение RMS колебаний (зона бугров)');
legend('Подвеска ВЫКЛ', 'Подвеска ВКЛ', 'Location', 'best');

set(findall(fig, 'Type', 'axes'), 'FontSize', 10);
exportgraphics(fig, 'wqr_ab_suspension.png', 'Resolution', 300, 'BackgroundColor', 'white');
fprintf('Обновлённый график сохранён: %s\n', fullfile(pwd, 'wqr_ab_suspension.png'));
