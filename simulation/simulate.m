% --- 3D Trajectory Visualization ---
% This script visualizes a 3D trajectory, similar to one that might be
% followed by a soft robot's end-effector.

% Clear workspace, command window, and close all figures
clear;
clc;
close all;

% --- 1. Trajectory Definition ---
% Define a 3D trajectory that approximates the path from the user's image.
% The path is a rough rectangle in 3D space.

% Corner points approximated from the image
p1 = [0.42, 0.82, 0.28]; % Start Point (approx)
p2 = [0.385, 0.82, 0.26];
p3 = [0.38, 0.79, 0.25];
p4 = [0.415, 0.79, 0.27];

% Number of points for each segment
num_points_per_segment = 50;

% Create the segments using linear interpolation (linspace)
% Segment 1: p1 to p2
x1 = linspace(p1(1), p2(1), num_points_per_segment);
y1 = linspace(p1(2), p2(2), num_points_per_segment);
z1 = linspace(p1(3), p2(3), num_points_per_segment);

% Segment 2: p2 to p3
x2 = linspace(p2(1), p3(1), num_points_per_segment);
y2 = linspace(p2(2), p3(2), num_points_per_segment);
z2 = linspace(p2(3), p3(3), num_points_per_segment);

% Segment 3: p3 to p4
x3 = linspace(p3(1), p4(1), num_points_per_segment);
y3 = linspace(p3(2), p4(2), num_points_per_segment);
z3 = linspace(p3(3), p4(3), num_points_per_segment);

% Segment 4: p4 to p1 (to close the loop)
x4 = linspace(p4(1), p1(1), num_points_per_segment);
y4 = linspace(p4(2), p1(2), num_points_per_segment);
z4 = linspace(p4(3), p1(3), num_points_per_segment);

% Combine all segments into a single trajectory
x_d = [x1, x2, x3, x4];
y_d = [y1, y2, y3, y4];
z_d = [z1, z2, z3, z4];

% --- 2. Animation Setup ---
figure('Name', '3D Trajectory Visualization', 'NumberTitle', 'off');
ax = axes;
grid on;
axis equal;
hold on;

% Set plot limits based on the trajectory data, with some padding
padding = 0.02;
axis([min(x_d)-padding max(x_d)+padding min(y_d)-padding max(y_d)+padding min(z_d)-padding max(z_d)+padding]);

xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('Soft Robot End-Effector Trajectory');
view(3); % Set the view to 3D

% Plot the complete desired trajectory for reference
plot3(x_d, y_d, z_d, 'g--', 'LineWidth', 1.5);

% Plot Start (green circle) and End (red square) points
plot3(x_d(1), y_d(1), z_d(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
plot3(x_d(end), y_d(end), z_d(end), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 10);
legend('Desired Path', 'Start', 'End', 'Location', 'northeast');


% --- 3. Main Simulation Loop ---
% Initialize a marker for the "end-effector" position
end_effector_marker = plot3(x_d(1), y_d(1), z_d(1), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);

for i = 1:length(x_d)
    % Update the position of the end-effector marker
    set(end_effector_marker, 'XData', x_d(i), 'YData', y_d(i), 'ZData', z_d(i));
    
    % Pause to create a smooth animation effect
    drawnow;
    pause(0.01);
end

disp('Simulation finished.');

