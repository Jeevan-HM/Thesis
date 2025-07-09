% Main script to find the latest experiment file and generate plots.
% This script mimics the functionality of the provided Python script.

% Housekeeping
clear;
clc;
close all;

% ---- CONFIGURATION ----

% Refactored: Define plot configs as an array of structs, element by element.
% This is a more robust method than the previous struct() call.

% Plots for the first window (Mocap Data)
clear MOCAP_PLOT_CONFIG; % Clear any previous definition to be safe
MOCAP_PLOT_CONFIG(1) = struct(...
    'title',   "Mocap Position (Body 3)", ...
    'ylabel',  "Position (m)", ...
    'columns', {{31, 32, 33}}, ...
    'labels',  {{"X", "Y", "Z"}}, ...
    'colors',  [0.1216, 0.4667, 0.7059; 1.0000, 0.4980, 0.0549; 0.1725, 0.6275, 0.1725] ...
);
MOCAP_PLOT_CONFIG(2) = struct(...
    'title',   "Mocap Yaw Orientation (Body 3)", ...
    'ylabel',  "Yaw (rad)", ...
    'columns', "yaw_body3", ...
    'labels',  "Yaw", ...
    'colors',  [0.8392, 0.1529, 0.1569] ...
);
MOCAP_PLOT_CONFIG(3) = struct(...
    'title',   "Mocap Pitch Orientation (Body 3)", ...
    'ylabel',  "Pitch (rad)", ...
    'columns', "pitch_body3", ...
    'labels',  "Pitch", ...
    'colors',  [0.5804, 0.4039, 0.7412] ...
);
MOCAP_PLOT_CONFIG(4) = struct(...
    'title',   "Mocap Roll Orientation (Body 3)", ...
    'ylabel',  "Roll (rad)", ...
    'columns', "roll_body3", ...
    'labels',  "Roll", ...
    'colors',  [0.0902, 0.7451, 0.8118] ...
);


% Plots for the second window (Sensor and Control Data)
clear SENSOR_CONTROL_CONFIG;
SENSOR_CONTROL_CONFIG(1) = struct(...
    'title',   "Desired Pressures", ...
    'ylabel',  "Desired Pressure", ...
    'columns', {{6, 7}}, ...
    'labels',  [], ...
    'colors',  [0.1216, 0.4667, 0.7059; 1.0000, 0.4980, 0.0549] ...
);
SENSOR_CONTROL_CONFIG(2) = struct(...
    'title',   "Sensing Column (Segment 1)", ...
    'ylabel',  "Sensor Pressure", ...
    'columns', 8:12, ...
    'labels',  [], ...
    'colors',  [0.1216, 0.4667, 0.7059; 1.0000, 0.4980, 0.0549; 0.1725, 0.6275, 0.1725; 0.8392, 0.1529, 0.1569; 0.5804, 0.4039, 0.7412] ...
);
SENSOR_CONTROL_CONFIG(3) = struct(...
    'title',   "Sensing Column (Segment 3)", ...
    'ylabel',  "Sensor Pressure", ...
    'columns', 13:17, ...
    'labels',  [], ...
    'colors',  [0.1216, 0.4667, 0.7059; 1.0000, 0.4980, 0.0549; 0.1725, 0.6275, 0.1725; 0.8392, 0.1529, 0.1569; 0.5804, 0.4039, 0.7412] ...
);

TIME_COLUMN = 1; % MATLAB is 1-based, so column 0 becomes 1
START_TIME_SEC = 10; % Time to start plotting from

% ---- END CONFIGURATION ----


% --- Main Execution ---
try
    % Get the full path to the latest experiment data file
    filename = get_experiment();
    % For testing, you can manually override the filename:
    % filename = "/home/g1/Developer/RISE_Lab/colcon_ws/experiments/June-25/Test_2.csv";

    fprintf('\nAnalyzing:\n%s\n\n', filename);

    % Read the CSV data into a table
    opts = detectImportOptions(filename);
    data = readtable(filename, opts);

    if isempty(data)
        error('Data could not be read. Check file path or format.');
    end

    time = data{:, TIME_COLUMN};

    % Slice data to start from a specific time
    if time(end) >= START_TIME_SEC
        fprintf('Slicing data to start from %d seconds.\n', START_TIME_SEC);
        start_index = find(time >= START_TIME_SEC, 1, 'first');
        time = time(start_index:end);
        data = data(start_index:end, :);
    else
        fprintf('Warning: Total duration is less than %ds. Plotting from beginning.\n', START_TIME_SEC);
    end
    
    % Re-normalize time to start from 0 after slicing
    if ~isempty(time)
        time = time - time(1);
    end

    % Precompute derived columns (Yaw, Pitch, Roll)
    derived = struct();
    % Check if the table has enough columns for quaternion data before trying
    if width(data) >= 37
        try
            % Corrected: Python's 33:37 is MATLAB's 34:37 (4 columns)
            quat_body3 = data{:, 34:37}; 
            qx = quat_body3(:, 1);
            qy = quat_body3(:, 2);
            qz = quat_body3(:, 3);
            qw = quat_body3(:, 4);
            
            derived.yaw_body3 = quaternion_to_yaw(qx, qy, qz, qw);
            derived.pitch_body3 = quaternion_to_pitch(qx, qy, qz, qw);
            derived.roll_body3 = quaternion_to_roll(qx, qy, qz, qw);
        catch E
            fprintf('Could not calculate orientation: %s\n', E.message);
        end
    else
        fprintf('Warning: Data has only %d columns. Skipping orientation calculation.\n', width(data));
    end

    headers = data.Properties.VariableNames;
    [~, base_title, ~] = fileparts(filename);
    base_title = strrep(base_title, '_', ' '); % Make title prettier

    % --- Create the plot windows ---
    
    % Window 1: Mocap Data (2D)
    create_plot_window(1, MOCAP_PLOT_CONFIG, data, derived, time, headers, ...
        sprintf('Mocap Data Analysis (Time Series): %s', base_title));

    % Window 2: Sensor and Control Data (2D)
    create_plot_window(2, SENSOR_CONTROL_CONFIG, data, derived, time, headers, ...
        sprintf('Sensor & Control Data: %s', base_title));
    
    % Window 3: Mocap Position (3D)
    if width(data) >= 33 % Check for position data
        create_3d_mocap_plot(3, data, ...
            sprintf('Mocap 3D Trajectory (Body 3): %s', base_title));
    end

catch ME
    fprintf(2, 'An error occurred: %s\n', ME.message);
    fprintf(2, 'File: %s, Line: %d\n', ME.stack(1).name, ME.stack(1).line);
end


% ---- HELPER FUNCTIONS ----

function filename = get_experiment()
    % Finds the latest experiment CSV file based on directory structure.
    experiments_base_dir = '/home/g1/Developer/RISE_Lab/colcon_ws/experiments';
    
    if ~exist(experiments_base_dir, 'dir')
        error('The specified experiment directory was not found. Please check the ''experiments_base_dir'' path.');
    end

    % Get all items in the base directory
    dir_contents = dir(experiments_base_dir);
    % Filter for directories, excluding '.' and '..'
    dir_contents = dir_contents([dir_contents.isdir] & ~ismember({dir_contents.name}, {'.', '..'}));

    if isempty(dir_contents)
        error('No experiment folders found in the base directory.');
    end
    
    % Filter for folders matching the 'Month-Day' pattern
    date_folder_pattern = '^[A-Za-z]+-\d{2}$';
    valid_folders = {};
    valid_dates = [];
    current_year = year(datetime('now'));

    for i = 1:length(dir_contents)
        folder_name = dir_contents(i).name;
        if ~isempty(regexp(folder_name, date_folder_pattern, 'once'))
            try
                dt = datetime([folder_name, '-', num2str(current_year)], 'InputFormat', 'MMMM-dd-yyyy');
                if month(dt) > month(datetime('now'))
                    dt.Year = dt.Year - 1;
                end
                valid_folders{end+1} = folder_name; %#ok<AGROW>
                valid_dates(end+1) = datenum(dt); %#ok<AGROW>
            catch
                % Ignore folders that can't be parsed
            end
        end
    end

    if isempty(valid_folders)
        error('No valid date-named folders found in experiments.');
    end

    % Sort folders by date to find the latest one
    [~, sort_idx] = sort(valid_dates, 'descend');
    latest_folder = valid_folders{sort_idx(1)};
    latest_folder_path = fullfile(experiments_base_dir, latest_folder);
    
    % Find all 'Test_*.csv' files in the latest folder
    test_files_info = dir(fullfile(latest_folder_path, 'Test_*.csv'));
    if isempty(test_files_info)
        error('Latest date folder (%s) contains no valid ''Test_*.csv'' files.', latest_folder);
    end

    % Extract test numbers to find the latest test
    test_nums = [];
    for i = 1:length(test_files_info)
        num_str = regexp(test_files_info(i).name, 'Test_(\d+)\.csv', 'tokens');
        if ~isempty(num_str)
            test_nums(i) = str2double(num_str{1}{1});
        else
            test_nums(i) = -1; % Invalid name
        end
    end

    [~, max_idx] = max(test_nums);
    latest_test_file = test_files_info(max_idx).name;
    filename = fullfile(latest_folder_path, latest_test_file);
end

function roll = quaternion_to_roll(qx, qy, qz, qw)
    roll = atan2(2 * (qw .* qx + qy .* qz), 1 - 2 * (qx.^2 + qy.^2));
end

function pitch = quaternion_to_pitch(qx, qy, qz, qw)
    arg = 2 * (qw .* qy - qz .* qx);
    arg = max(min(arg, 1.0), -1.0);
    pitch = asin(arg);
end

function yaw = quaternion_to_yaw(qx, qy, qz, qw)
    yaw = -atan2(2 * (qw .* qz + qx .* qy), 1 - 2 * (qy.^2 + qz.^2));
end

function create_plot_window(fig_num, plot_configs, data, derived, time, headers, window_title)
    % This function works with the new struct array config without changes.
    num_plots = length(plot_configs);
    
    figure(fig_num);
    set(gcf, 'Name', window_title, 'NumberTitle', 'off');
    sgtitle(window_title, 'FontSize', 16, 'FontWeight', 'bold');

    for i = 1:num_plots
        ax = subplot(num_plots, 1, i);
        hold(ax, 'on');
        
        plot_cfg = plot_configs(i); % Access the i-th struct in the array
        cols_to_plot = plot_cfg.columns;
        color_matrix = plot_cfg.colors;
        labels_for_plot = plot_cfg.labels;
        
        line_handles = [];
        legend_labels = {};
        
        if ischar(cols_to_plot) || isstring(cols_to_plot) % Case for single derived column 
            if isfield(derived, cols_to_plot) && ~isempty(derived.(cols_to_plot))
                y_data = derived.(cols_to_plot);
                h = plot(ax, time, y_data, 'LineWidth', 2.0, 'Color', color_matrix);
                line_handles(1) = h;
                legend_labels{1} = labels_for_plot;
            end
        else % Case for numeric columns 
            if iscell(cols_to_plot)
                cols_to_plot = cell2mat(cols_to_plot);
            end
            
            for k = 1:length(cols_to_plot)
                col_idx = cols_to_plot(k);
                if col_idx > width(data)
                    fprintf('Warning: Column index %d for plot "%s" is out of range.\n', col_idx, plot_cfg.title);
                    continue;
                end
                
                y_data = data{:, col_idx};
                
                if isempty(labels_for_plot)
                    label = headers{col_idx};
                else
                    label = labels_for_plot{k};
                end
                
                line_color = color_matrix(k, :);
                
                h = plot(ax, time, y_data, 'LineWidth', 2.0, 'Color', line_color);
                line_handles(end+1) = h; %#ok<AGROW>
                legend_labels{end+1} = strrep(label, '_', ' '); %#ok<AGROW>
            end
        end

        title(ax, plot_cfg.title, 'FontSize', 14);
        ylabel(ax, plot_cfg.ylabel, 'FontSize', 12);
        grid(ax, 'on');
        box(ax, 'on');
        set(ax, 'XGrid', 'on', 'YGrid', 'on', 'GridAlpha', 0.5, 'MinorGridAlpha', 0.5);
        if ~isempty(line_handles)
            legend(line_handles, legend_labels, 'FontSize', 10, 'Location', 'northeast');
        end
        hold(ax, 'off');
    end
    
    xlabel(subplot(num_plots, 1, num_plots), 'Time (s)', 'FontSize', 12); % Label last x-axis
end

function create_3d_mocap_plot(fig_num, data, window_title)
    % Creates a 3D plot for the mocap trajectory.
    figure(fig_num);
    set(gcf, 'Name', window_title, 'NumberTitle', 'off');

    % Note: 1-based indexing means columns 30, 31, 32 are 31, 32, 33
    x = data{:, 31};
    y = data{:, 32};
    z = data{:, 33};

    ax = gca;
    plot3(ax, x, y, z, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Trajectory');
    hold(ax, 'on');

    scatter3(ax, x(1), y(1), z(1), 100, 'g', 'filled', 'Marker', 'o', 'DisplayName', 'Start');
    scatter3(ax, x(end), y(end), z(end), 100, 'r', 'filled', 'Marker', 's', 'DisplayName', 'End');
    
    hold(ax, 'off');
    
    xlabel('X Position (m)', 'FontWeight', 'bold');
    ylabel('Y Position (m)', 'FontWeight', 'bold');
    zlabel('Z Position (m)', 'FontWeight', 'bold');
    title(window_title, 'FontSize', 16, 'FontWeight', 'bold');
    legend('show', 'Location', 'northeast');
    grid on;
    axis equal;
    rotate3d on;
end
