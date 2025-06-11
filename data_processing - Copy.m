filename = 'June4th_Test_2.txt';
TIMESTAMP= 1;
PD_START= 2;
PD_END = 5; % 4 desired pressures
PM_START = 14;
PM_END = 21; % This range (14 to 21 inclusive) covers 8 columns, meaning 8 actual pressure sensors
MOCAP_START = 22;
MOCAP_END = 42;

data = dlmread(filename, ',');
timestampData = data(:, TIMESTAMP);
desired_pressure = data(:, PD_START:PD_END);
actual_pressure_sensor = data(:, PM_START:PM_END); % Correctly selecting all 8 sensor columns

% --- Plotting with Subplots ---

figure; % Create a new figure

% Subplot 1: Desired Pressure vs time
subplot(2, 1, 1); % 2 rows, 1 column, first subplot
plot(timestampData, desired_pressure);
title('Desired Pressure vs. Time');
xlabel('Time (s)');
ylabel('Pressure (psi)');
legend({'Desired P1', 'Desired P2', 'Desired P3', 'Desired P4'}, 'Location', 'bestoutside'); % Changed to bestoutside to avoid overlapping
grid on;
set(gca, 'FontSize', 10); % Adjust font size for better readability in subplot

% Subplot 2: Actual Pressure Sensor vs time (all 8 sensors)
subplot(2, 1, 2); % 2 rows, 1 column, second subplot
plot(timestampData, actual_pressure_sensor);
title('Actual Pressure Sensor Readings vs. Time (Sensors 1-8)');
xlabel('Time (s)');
ylabel('Pressure (psi)');
% Create a cell array for the legend entries for all 8 sensors
sensorLegends = cell(1, 8);
for i = 1:8
    sensorLegends{i} = sprintf('Sensor %d', i);
end
legend(sensorLegends, 'Location', 'bestoutside'); % Changed to bestoutside
grid on;
set(gca, 'FontSize', 10); % Adjust font size for better readability in subplot

% Optional: Add a super title for the entire figure
sgtitle('Pressure Data Analysis', 'FontSize', 14, 'FontWeight', 'bold');

% Optional: Adjust overall figure size for better viewing
set(gcf, 'Position', [100, 100, 800, 700]); % [left, bottom, width, height]