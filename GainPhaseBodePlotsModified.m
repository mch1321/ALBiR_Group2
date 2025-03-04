%% Bode Plot Generation tuned (modified)

clear all;
close all;
clc;

%% load data 
  
folder = 'Modified';
% folder = "Original";

csvFiles = dir(fullfile(folder, '*.csv')); % get all csv files in folder
allData = cell(1, numel(csvFiles)); % initialise allData to save csv data here

% loop through each file and obtain the data
for i = 1:numel(csvFiles)
    filePath = fullfile(folder, csvFiles(i).name);
    allData{i} = readtable(filePath);  
    disp(['Loaded: ', csvFiles(i).name]); % to ensure files are loaded
end

%% Plot the camera heading angle and error against time

% loop through all the csv data
for i = 1:numel(allData)
    time = allData{i}.(1); 
        error = allData{i}.(2);  
    angle = allData{i}.(3);     
    
    % Plot for visualisation purposes
    figure;
    % Time vs Camera Angle
    subplot(1,1,1)
    plot(time, angle);
    title(['Camera angle against Time for file ' csvFiles(i).name]);
    xlabel('Time');
    ylabel('Camera angle');

end

%% Perfect tracking angle

% Field of view and resolution obtained from camera.py
h_fov = 31.5;
width = 640;

% Therefore, 1 pixel corresponds to this many degrees:
pxl_degrees = h_fov/width;

% Calculated by hand as explained in report
targetmax_angle = 60;
% this corresponds to an amplitude of this many pixels
true_pixels = targetmax_angle / pxl_degrees; 

%% Gain and Phase Bode plot

% Getting gain:
df = [0.03, 0.05, 0.07, 0.09, 0.11, 0.13, 0.15, 0.17, 0.19]; %, 0.1, 0.12, 0.14, 0.16, 0.18, 0.2, 0.22, 0.24];
gains = [];

figure;
% loop through all the csv data
for i = 1:9
    time = allData{i}.(1); % time data
    error = allData{i}.(2);  
    angle = allData{i}.(3); % angle vector   
    
    t = time/1000; % to get time in seconds
    
    % Plot for visualisation purposes
    % Time vs Camera Angle
    subplot(3,4,i)
    
    t_smooth = linspace(min(t), max(t), 1000);
    sin_wave = 30 * sin(t_smooth);

    plot(t, angle, 'b');
    hold on;
    plot(t_smooth, sin_wave, 'r');
    hold off;

    title(['Camera angle against Time for ' num2str(df(i))]);
    xlabel('Time');
    ylabel('Camera angle');
    
    % Displays the angle amplitude for each file
    max_angle = max(angle);
    min_angle = abs(min(angle));
    amplitude = (max_angle+min_angle)/pxl_degrees;
    disp(['For frequency: ', num2str(df(i))])
    disp(['max: ', num2str(max_angle)])
    disp(['min: ', num2str(min_angle)])
    disp(['Amplitude: ', num2str(amplitude)]);

    % Calculate gain: amplitude normalised by the perfect amplitude (60)
    gain = amplitude / true_pixels;
    disp(['gain: ', num2str(gain)])
    gains = [gains, gain];

end

% Getting phase_lags
phase_lags = [];

figure;
% loop through all the csv data
for i = 1:length(df)
    
    disp(df(i))
    angle1 = allData{i}.(3); % angle data
    T = allData{i}.(1); % time vector
    N = length(angle1); 
    
    fs = 1; % sampling frequency
    T = T/1000; % time in seconds
    T_max = T(end); % get maximum time
    T_start = T(1,:); % get start time
    dt = (T_max-T_start)/(length(angle1)-1); % calculate the average step
    t = T_start:dt:T_max; % ideal time vector (linear step)
    
    % Ideal sine wave
    ideal_signal = 30 * sin(2 * pi * df(i) * t);

    % Plot ideal signal
    subplot(3,3,i)
    plot(t, ideal_signal, 'r', 'LineWidth', 1.5); hold on;
    plot(T, angle1, 'b', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Amplitude');
    title(['Ideal Sine Wave at ', num2str(df(i)), ' Hz vs Measured Angle']);
    grid on;

    [correlation, lags] = xcorr(angle1, ideal_signal, 'coeff');
    [~, max_idx] = max(correlation);
    sample_lag = lags(max_idx); 
    disp(sample_lag)

    frequency = df(i); % frequency of ideal signal degrees
    phase_lag = 2 * pi * frequency * sample_lag / fs; % phase lag in degrees
    phase_lag = mod(phase_lag + 180, 360) - 180;
    phase_lags = [phase_lags, phase_lag];
    
end

%% Plot bode plots
gains = gains-1;

figure;
subplot(1,2,1)
plot(df, gains, 'LineWidth',2);
title('Bode Plot', 'FontSize',20);
xlabel('Frequency (Hz)', 'FontSize', 15);
ylabel('Gain','FontSize', 15);
ylim([0 1.1])

subplot(1,2,2)
plot(df, phase_lags, 'r', 'LineWidth',2);
title('Bode Plot', 'FontSize',20);
xlabel('Frequency (Hz)', 'FontSize', 15);
ylabel('Phase (degrees)','FontSize', 15);
ylim([-20 60])


