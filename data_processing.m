clear;
clc;
close all;
% ---- CONFIGURATION ----
% Configuration for each subplot
% NOTE: All column indices are 1-based for MATLAB.
PLOT_CONFIG = {
    struct(...
        "title", "Desired Pressures", ...
        "ylabel", "Desired Pressure", ...
        "columns", {{2, 3, 4}}, ...      % Python: [1, 2, 3]
        "labels", {{}}, ...              % Use {} for None to use column headers
        "plot_type", "line" ...
    ), ...
    struct(...
        "title", "Mocap Body Yaw Orientation", ...
        "ylabel", "Yaw (rad)", ...
        "columns", {{'yaw_body3'}}, ...  % Special: calculated column
        "labels", {{'Yaw'}}, ...
        "plot_type", "line" ...
    ), ...
    struct(...
        "title", "Sensing Column (Segment 1)", ...
        "ylabel", "Sensor Pressure", ...
        "columns", {{7, 8, 9, 10, 11}}, ... % Python: list(range(6, 11))
        "labels", {{}}, ...
        "plot_type", "line" ...
    ), ...
    struct(...
        "title", "Sensing Column (Segment 3)", ...
        "ylabel", "Sensor Pressure", ...
        "columns", {{12, 13, 14, 15, 16}}, ... % Python: list(range(11, 16))
        "labels", {{}}, ...
        "plot_type", "line" ...
    )
};
TIME_COLUMN = 1; % index of time column (1-based)
% ---- END CONFIGURATION ----
% --- Main Execution ---
main();
% --- Function Definitions ---
function main()
    % Get configuration from the main script's workspace
    PLOT_CONFIG = evalin('base', 'PLOT_CONFIG');
    TIME_COLUMN = evalin('base', 'TIME_COLUMN');
    % 1. Find the latest experiment file
    filename = get_experiment();
    if isempty(filename)
        return;
    end
    fprintf('Latest experiment file: %s\n', filename);
    % 2. Read and validate data
    opts = detectImportOptions(filename);
    data = readtable(filename, opts);
    if isempty(data)
        error("Data could not be read. Check file path or format.");
    end
    fprintf('\nAnalyzing:\n%s\n\n', filename);
    % 3. Precompute derived columns
    derived = struct();
    try
        % Note: Adjust column indices for your specific CSV format
        % Python iloc[:, 30:37] corresponds to MATLAB columns 31 to 38
        quat_body3 = data{:, 31:37};
        derived.yaw_body3 = quaternion_to_yaw(...
            quat_body3(:, 4), quat_body3(:, 5), quat_body3(:, 6), quat_body3(:, 7) ...
        );
    catch ME
        warning('Could not calculate yaw_body3. Check quaternion columns. Error: %s', ME.message);
        derived.yaw_body3 = [];
    end
    time = data{:, TIME_COLUMN};
    headers = data.Properties.VariableNames;
    % 4. Create Plots
    num_plots = numel(PLOT_CONFIG);
    fig = figure();
    tl = tiledlayout(num_plots, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    
    % Set overall figure title
    [~, base_filename, ext] = fileparts(filename);
    sgtitle(tl, sprintf('Experiment Analysis: %s', [base_filename, ext]), ...
        'FontSize', 20, 'FontWeight', 'bold');
    % Loop through each plot configuration
    for p_idx = 1:num_plots
        ax = nexttile(tl);
        hold(ax, 'on'); % Hold on to plot multiple lines on the same axes
        plot_cfg = PLOT_CONFIG{p_idx};
        cols_to_plot = plot_cfg.columns;
        
        for i = 1:numel(cols_to_plot)
            col_ref = cols_to_plot{i};
            y_data = [];
            label = '';
            if ischar(col_ref) % If column is a string (e.g., 'yaw_body3')
                if isfield(derived, col_ref)
                    y_data = derived.(col_ref);
                    label = plot_cfg.labels{i};
                end
            else % If column is a numeric index
                col_idx = col_ref;
                if col_idx > width(data)
                    warning('Warning: Column index %d out of range.', col_idx);
                    continue;
                end
                y_data = data{:, col_idx};
                
                % Use header as label if 'labels' field is empty
                if isempty(plot_cfg.labels)
                    label = headers{col_idx};
                else
                    label = plot_cfg.labels{i};
                end
            end
            
            if ~isempty(y_data)
                % Plot without specifying color to use MATLAB's default color cycle
                plot(ax, time, y_data, ...
                    'DisplayName', strrep(label, '_', ' '), ... % Use DisplayName for legend
                    'LineWidth', 2.0);
            end
        end
        % Configure axes aesthetics
        title(ax, plot_cfg.title, 'FontSize', 16);
        ylabel(ax, plot_cfg.ylabel, 'FontSize', 14);
        legend(ax, 'show', 'Location', 'northeast', 'FontSize', 12);
        grid(ax, 'on');
        ax.GridAlpha = 0.5;
        ax.GridLineStyle = '--';
        ax.FontSize = 12;
        hold(ax, 'off');
    end
    % Set shared X-axis label
    xlabel(tl, 'Time (s)', 'FontSize', 16, 'FontWeight', 'bold');
    
    % Note: Interactive tooltips (Data Cursor) are enabled by default in MATLAB figures.
    % datacursormode on; % Uncomment to ensure it's on
end
function filename = get_experiment()
% Finds the latest 'Test_*.csv' file in the latest date-named folder.
    filename = '';
    experiments_base_dir = '/home/g1/Developer/RISE_Lab/colcon_ws/experiments';
    
    if ~isfolder(experiments_base_dir)
        error('Base experiments directory not found: %s', experiments_base_dir);
    end
    % 1. Find all subdirectories
    dir_contents = dir(experiments_base_dir);
    sub_dirs = dir_contents([dir_contents.isdir]);
    
    % 2. Filter for date-named folders (e.g., 'June-20')
    date_folder_pattern = '^[A-Za-z]+-\d{2}$';
    latest_folder_date = datetime(1,1,1);
    latest_folder_name = '';
    for i = 1:length(sub_dirs)
        folder_name = sub_dirs(i).name;
        if ~isempty(regexp(folder_name, date_folder_pattern, 'once'))
            try
                % Convert folder name to a date for comparison
                current_date = datetime(folder_name, 'InputFormat', 'MMMM-dd');
                % Append current year for proper comparison
                current_date.Year = year(now); 
                if current_date > latest_folder_date
                    latest_folder_date = current_date;
                    latest_folder_name = folder_name;
                end
            catch
                % Ignore folders that don't parse as a date
            end
        end
    end
    if isempty(latest_folder_name)
        error('No valid date-named folders found in experiments directory.');
    end
    
    latest_folder_path = fullfile(experiments_base_dir, latest_folder_name);
    fprintf('Found latest date folder: %s\n', latest_folder_name);
    % 3. Find all 'Test_*.csv' files in the latest folder
    test_files = dir(fullfile(latest_folder_path, 'Test_*.csv'));
    if isempty(test_files)
        error('Latest date folder (%s) contains no valid ''Test_*.csv'' files.', latest_folder_name);
    end
    
    % 4. Extract test numbers and find the latest one
    test_num_pattern = 'Test_(\d+)\.csv';
    latest_test_num = -1;
    latest_test_file = '';
    for i = 1:length(test_files)
        file_name = test_files(i).name;
        tokens = regexp(file_name, test_num_pattern, 'tokens');
        if ~isempty(tokens)
            test_num = str2double(tokens{1}{1});
            if test_num > latest_test_num
                latest_test_num = test_num;
                latest_test_file = file_name;
            end
        end
    end
    if isempty(latest_test_file)
         error('Latest date folder (%s) contains no valid ''Test_*.csv'' files.', latest_folder_name);
    end
    filename = fullfile(latest_folder_path, latest_test_file);
end
function yaw = quaternion_to_yaw(qx, qy, qz, qw)
% Converts a quaternion to a yaw angle.
% Uses element-wise operators to support vector inputs.
    yaw = -atan2(2 * (qw .* qz + qx .* qy), 1 - 2 * (qy.^2 + qz.^2));
end