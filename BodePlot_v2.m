%% Bode plot generation
clear all;
close all;
clc;

%% Load data using preferred method

% Get all CSV files in the specified directory
folder = 'tune';
csv_files = dir(fullfile(folder, '*.csv'));

% Extract frequency from filenames and sort them
frequencies = zeros(1, length(csv_files));
for i = 1:length(csv_files)
    filename = csv_files(i).name;
    freq_match = regexp(filename, 'Curve([\d\.]+)Hz', 'tokens');
    if ~isempty(freq_match)
        frequencies(i) = str2double(freq_match{1}{1});
    else
        frequencies(i) = NaN; % Mark invalid files
    end
end

% Remove invalid files and sort by frequency
valid_idx = ~isnan(frequencies);
csv_files = csv_files(valid_idx);
frequencies = frequencies(valid_idx);
[frequencies, sorted_idx] = sort(frequencies);
csv_files = csv_files(sorted_idx);

%% Initialize arrays to store results
gains_db = [];

% Field of view and resolution obtained from camera.py
h_fov = 31.5;
width = 640;
pxl_degrees = h_fov / width;
angle_range = 100;%max:50,min-50
true_pixels = angle_range / pxl_degrees; % approx 1000 pixels

%% Process each CSV file
for i = 1:length(csv_files)
    file = fullfile(folder, csv_files(i).name);
    data = readtable(file);

    % Extract relevant columns
    time = data{:, 1}; 
    error = data{:, 2};   % Input: target deviation (error)
    angle = data{:, 3};   % Absolute servo position
    
    % Convert time to seconds if in milliseconds
    if max(time) > 1000
        time = time / 1000;  
    end
    
    % Shift data to center around zero
    mean_angle = (max(angle) + min(angle)) / 2;
    angle = angle - mean_angle;
    
    % Displays the angle amplitude for each file
    max_angle = max(angle);
    min_angle = min(angle);
    amplitude = (max_angle - min_angle) / pxl_degrees;

    disp(['For file: ', num2str(i)]);
    disp(['max: ', num2str(max_angle)]);
    disp(['min: ', num2str(min_angle)]);
    disp(['Amplitude: ', num2str(amplitude)]);

    % Calculate gain in dB: 20*log10(amplitude / true_pixels)
    gain_db = 20 * log10(amplitude / true_pixels);
    disp(['Gain (dB): ', num2str(gain_db)]);
    gains_db = [gains_db, gain_db];
end

%% Plot Bode Plot
figure;
plot(frequencies, gains_db, 'LineWidth',2);
title('Bode Plot', 'FontSize',20);
xlabel('Frequency (Hz)', 'FontSize', 15);
ylabel('Gain (dB)','FontSize', 15);
grid on;
xticks(frequencies);  % Set x-axis tick marks to the frequency values
xticklabels(string(frequencies));  % Label x-axis with the frequency values
