% File path
filename = GetExperiment();
fprintf('Analyzing :\n%s\n\n', filename);
% % filename = "C:\Users\jeeva\Desktop\RISE Lab\experiments\June-12\Test_12.txt"
% Read the data (assumes comma-delimited; adjust if needed)
data = readmatrix(filename);

% Sanity check
if isempty(data)
    error('Data could not be read. Check file path or format.');
end

% --- Extract Columns ---
time = data(:, 1);            % Time column 

pd = data(:, 13:16);            % pd1 to pd4
pm12_16 = data(:, 17:19)
% pm12_16 = data(:, 18);     % pm12 to pm16 (last 5 pressure sensors)
quat_body3 = data(:, 39:42);  % Body 3 quaternion: qx, qy, qz, qw

start_time = 10; % seconds
idx = time >= start_time;

time = time(idx);
pm12_16 = pm12_16(idx, :);


% --- Convert Quaternion to Yaw ---
function yaw = quaternionToYaw(qx, qy, qz, qw)
    % Converts quaternion to yaw angle (in radians)
    yaw = - atan2(2*(qw.*qz + qx.*qy), 1 - 2*(qy.^2 + qz.^2));
end

yaw_body3 = quaternionToYaw(quat_body3(:,1), quat_body3(:,2), quat_body3(:,3), quat_body3(:,4));
% Define bright, high-contrast colors
bright_colors = [
    1.0, 0.4, 0.4;   % bright red
    0.4, 1.0, 0.4;   % bright green
    0.4, 0.8, 1.0;   % bright blue
    1.0, 1.0, 0.4;   % yellow
    1.0, 0.6, 1.0;   % pinkish
];

figure('Color', [0.1 0.1 0.1]);  % Set figure background for dark mode
hold on;
for i = 1:5
    plot(time, pm12_16(:,i), 'LineWidth', 2, 'Color', bright_colors(i,:));
end
hold off;
xlabel('Time (s)', 'Color', 'w');
ylabel('Sensor Pressure', 'Color', 'w');
title('Pressure Sensors: pm12–pm16', 'Color', 'w');
legend({'Sensor 1','Sensor 2','Sensor 3','Sensor 4','Sensor 5'}, 'TextColor', 'w');
set(gca, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w');
grid on;