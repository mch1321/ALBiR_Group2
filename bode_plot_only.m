%% Bode Plot Generation untuned (original and modified)

clear;
clc;

%% Load Original Data 
folder_orig = "C:\\Users\\sreed\\Downloads\\original\\original";
csvFiles_orig = dir(fullfile(folder_orig, '*.csv')); % Get all CSV files in folder
numFiles_orig = numel(csvFiles_orig);
allData_orig = cell(1, numFiles_orig); % Initialize allData to save CSV data

% Loop through each file and obtain the data
for i = 1:numFiles_orig
    filePath = fullfile(folder_orig, csvFiles_orig(i).name);
    allData_orig{i} = readtable(filePath);  
    disp(['Loaded Original: ', csvFiles_orig(i).name]); % To ensure files are loaded
end

%% Load Modified Data
folder_mod = "C:\\Users\\sreed\\Downloads\\modified\\modified";
csvFiles_mod = dir(fullfile(folder_mod, '*.csv')); % Get all CSV files in folder
numFiles_mod = numel(csvFiles_mod);
allData_mod = cell(1, numFiles_mod); % Initialize allData to save CSV data

% Loop through each file and obtain the data
for i = 1:numFiles_mod
    filePath = fullfile(folder_mod, csvFiles_mod(i).name);
    allData_mod{i} = readtable(filePath);  
    disp(['Loaded Modified: ', csvFiles_mod(i).name]); % To ensure files are loaded
end

%% Define Camera Field of View Parameters
h_fov = 31.5;
width = 640;
pxl_degrees = h_fov / width; % Degrees per pixel

target_max_angle = 100;
true_pixels = target_max_angle / pxl_degrees; % Target amplitude in pixels

%% Define Frequency Data
df = [0.03, 0.05, 0.07, 0.09, 0.1, 0.11,0.13,0.15  0.17, 0.19, 0.2, 0.3];%,  0.4, 0.5,  0.6, 0.7]; % 0.15, 0.5,0.3,0.19,
numFrequencies_orig = min(numFiles_orig, numel(df));
numFrequencies_mod = min(numFiles_mod, numel(df));

gains_orig = zeros(1, numFrequencies_orig);
phase_lags_orig = zeros(1, numFrequencies_orig);
gains_mod = zeros(1, numFrequencies_mod);
phase_lags_mod = zeros(1, numFrequencies_mod);

%% Compute Gain and Phase for Original Data
for i = 1:numFrequencies_orig
    colNames = allData_orig{i}.Properties.VariableNames;
    
    time = allData_orig{i}.(colNames{1}) / 1000; % Convert to seconds
    angle = allData_orig{i}.(colNames{3}); % Extract angle data
    
    max_angle = max(angle);
    min_angle = abs(min(angle));
    amplitude = (max_angle + min_angle) / pxl_degrees;
    gain = amplitude / true_pixels;
    
    gains_orig(i) = gain;
    
    % Phase Lag Calculation
    T = allData_orig{i}.(colNames{1}) / 1000; % Time in seconds
    t = linspace(T(1), T(end), numel(T));
    ideal_signal = 30 * sin(2 * pi * df(i) * t);
    
    [correlation, lags] = xcorr(angle, ideal_signal, 'coeff');
    [~, max_idx] = max(correlation);
    sample_lag = lags(max_idx);
    
    phase_lags_orig(i) = mod(2 * pi * df(i) * sample_lag + 180, 360) - 180;
end

%% Compute Gain and Phase for Modified Data
for i = 1:numFrequencies_mod
    colNames = allData_mod{i}.Properties.VariableNames;
    
    time = allData_mod{i}.(colNames{1}) / 1000; % Convert to seconds
    angle = allData_mod{i}.(colNames{3}); % Extract angle data
    
    max_angle = max(angle);
    min_angle = abs(min(angle));
    amplitude = (max_angle + min_angle) / pxl_degrees;
    gain = amplitude / true_pixels;
    
    gains_mod(i) = gain;
    
    % Phase Lag Calculation
    T = allData_mod{i}.(colNames{1}) / 1000; % Time in seconds
    t = linspace(T(1), T(end), numel(T));
    ideal_signal = 30 * sin(2 * pi * df(i) * t+180); %!!! this has been changed
    
    [correlation, lags] = xcorr(angle, ideal_signal, 'coeff');
    [~, max_idx] = max(correlation);
    sample_lag = lags(max_idx);
    
    phase_lags_mod(i) = mod(2 * pi * df(i) * sample_lag + 180, 360) - 180;
end

%% Plot Bode Plots (4-Subplot Layout)
figure;
subplot(2,2,1);
plot(df(1:numFrequencies_orig), gains_orig, '-b', 'LineWidth', 2);
grid on; grid minor;
title('Bode Plot - Gain (Original)', 'FontSize', 14);
%xlim([0.03 0.7]);
ylim([0 2]);
xlabel('Frequency (Hz)', 'FontSize', 12);
ylabel('Gain', 'FontSize', 12);
set(gca, 'FontSize', 10);

%figure;
subplot(2,2,3);
plot(df(1:numFrequencies_orig), phase_lags_orig, '-r', 'LineWidth', 2);
grid on; grid minor;
title('Bode Plot - Phase (Original)', 'FontSize', 14);
%xlim([0.03 0.7]);
%ylim([-150 100]);
xlabel('Frequency (Hz)', 'FontSize', 12);
ylabel('Phase Lag (°)', 'FontSize', 12);
set(gca, 'FontSize', 10);

%figure;
subplot(2,2,2);
plot(df(1:numFrequencies_mod), gains_mod, '-b', 'LineWidth', 2);
grid on; grid minor;
title('Bode Plot - Gain (Modified)', 'FontSize', 14);
%xlim([0.03 0.2]);
%ylim([0 2]);
xlabel('Frequency (Hz)', 'FontSize', 12);
ylabel('Gain', 'FontSize', 12);
set(gca, 'FontSize', 10);

%figure;
subplot(2,2,4);
plot(df(1:numFrequencies_mod), phase_lags_mod, '-r', 'LineWidth', 2);
grid on; grid minor;
title('Bode Plot - Phase (Modified)', 'FontSize', 14);
%xlim([0.03 0.2]);
%ylim([-50 50]);
xlabel('Frequency (Hz)', 'FontSize', 12);
ylabel('Phase Lag (°)', 'FontSize', 12);
set(gca, 'FontSize', 10);

