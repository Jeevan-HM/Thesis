% File path
% filename = 'C:\Users\jeeva\Desktop\RISE Lab\experiments\June-06\Test_2.txt';
clear all;
clc;
base_path = 'C:\Users\jeeva\Desktop\RISE Lab\experiments';
d = dir(base_path); d = d([d.isdir] & ~ismember({d.name},{'.','..'}));
dates = cellfun(@(n) ~isempty(regexp(n,'^[A-Za-z]+-\d{2}$','once'))*datenum(n,'mmmm-dd'), {d.name});
[~,idx] = max(dates); if isnan(dates(idx)), error('No valid date-named folders.'); end
f = dir(fullfile(base_path, d(idx).name, '*.txt'));
tokens = regexp({f.name}, '_(\d+)\.txt', 'tokens', 'once');
valid = ~cellfun('isempty', tokens);
nums = cellfun(@(t) str2double(t{1}), tokens(valid));
ff = f(valid);
[~,j] = max(nums); filename = fullfile(ff(j).folder, ff(j).name);
clearvars -except filename

filename

% filename = "experiments\June-11\Test_6.txt"
% Read the data (assumes comma-delimited; adjust if needed)
data = readmatrix(filename);

% Sanity check
if isempty(data)
    error('Data could not be read. Check file path or format.');
end

% --- Extract Columns ---
time = data(:, 1);            % Time column

pd = data(:, 13:16);            % pd1 to pd4
pm12_16 = data(:, 17:21);     % pm12 to pm16 (last 5 pressure sensors)
quat_body3 = data(:, 39:42);  % Body 3 quaternion: qx, qy, qz, qw

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

% --- Plot 1: pd1–pd4 ---
subplot(3,1,1);
hold on;
for i = 1:4
    plot(time, pd(:,i), 'LineWidth', 2, 'Color', bright_colors(i,:));
end
hold off;
xlabel('Time (s)', 'Color', 'w');
ylabel('Measured Pressure', 'Color', 'w');
title('Desired Pressures: pd1–pd4 vs Time', 'Color', 'w');
legend({'pd1','pd2','pd3','pd4'}, 'TextColor', 'w');
set(gca, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w');
grid on;

% --- Plot 2: Yaw of Mocap Body 3 ---
subplot(3,1,2);
plot(time, yaw_body3, 'LineWidth', 2, 'Color', [1 0.7 0.2]);  % orange
xlabel('Time (s)', 'Color', 'w');
ylabel('Yaw (rad)', 'Color', 'w');
title('Yaw of Mocap Body 3', 'Color', 'w');
set(gca, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w');
grid on;

% --- Plot 3: pm12–pm16 ---
subplot(3,1,3);
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

