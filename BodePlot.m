%% Bode plot generation

clear all;
close all;
clc;

%% load data 

folder = 'testing_orignial_framebuffer_disabled';  

csvFiles = dir(fullfile(folder, '*.csv')); % get all csv files in folder
allData = cell(1, numel(csvFiles)); % initialise allData to save csv data here

% loop through each file and obtain the data
for i = 1:numel(csvFiles)
    filePath = fullfile(folder, csvFiles(i).name);
    allData{i} = readtable(filePath);  
    disp(['Loaded: ', csvFiles(i).name]); % to ensure files are loaded
end

%% Plot the camera heading angle against time

% loop through all the csv data
for i = 1:numel(allData)
    time = allData{i}.(1); 
    error = allData{i}.(2);  
    angle = allData{i}.(3);     
    
    % Plot for visualisation purposes
    figure;
    % Time vs Camera Angle
    subplot(1,2,1)
    plot(time, angle);
    title(['Camera angle against Time for file ' num2str(i)]);
    xlabel('Time');
    ylabel('Camera angle');

    % Time vs Angular error
    subplot(1,2,2)
    plot(time, angle);
    title(['Angular error against Time for file ' num2str(i)]);
    xlabel('Time');
    ylabel('Angular error');

end

%% Perfect tracking angle

% Field of view and resolution obtained from camera.py
h_fov = 31.5;
width = 640;

% Therefore, 1 pixel corresponds to this many degrees:
pxl_degrees = h_fov/width;

% So, if target max angle goes from -25 to 25 degrees (from tuning.py)
targetmax_angle = 50;
% this corresponds to an amplitude of this many pixels
true_pixels = targetmax_angle / pxl_degrees; % approx 1000 pixels

%% Comparing Camera Angle to Perfect tracking angle

frequencies = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.2, 1.4, 1.6, 1.8, 2, 2.2, 2.4, 2.6, 2.8, 3];
gains = [];

% loop through all the csv data
for i = 1:numel(allData)
    time = allData{i}.(1); 
    error = allData{i}.(2);  
    angle = allData{i}.(3);     
    
    %t = 0:0.1:length(time)*10;
    %true_angle = 25*sin(frequencies(i)*2*pi*t);

    % Plot for visualisation purposes
    %figure;
    % Time vs Camera Angle
    %plot(time, angle);
    %hold on;
    %plot(t*10, true_angle);
    %title(['Camera angle against Time for file ' num2str(i)]);
    %xlabel('Time');
    %ylabel('Camera angle');
    
    % Displays the angle amplitude for each file
    max_angle = max(angle);
    min_angle = min(angle);
    
    amplitude = (max_angle-min_angle)/pxl_degrees;

    disp(['For file: ', num2str(i)])
    disp(['max: ', num2str(max_angle)])
    disp(['min: ', num2str(min_angle)])
    disp(amplitude)

    % Calculate gain: amplitude normalised by the perfect amplitude (50)
    gain = amplitude / true_pixels;
    disp(['gain: ', num2str(gain)])
    gains = [gains, gain];

end

figure;
plot(frequencies, gains, 'LineWidth',2);
title('Bode Plot', 'FontSize',20);
xlabel('Frequency (Hz)', 'FontSize', 15);
ylabel('Gain','FontSize', 15);

   







