function analyze_experiment()
% ANALYZE_EXPERIMENT
% MATLAB port of your Python analysis/plotting script.
% Requires R2016b+ (for local functions & string support).

% ============================ SETTINGS (EDITABLE) =========================
EXPERIMENTS_BASE_DIR = "/home/g1/Developer/RISE_Lab/experiments";
START_TIME_OFFSET_SEC = 10;   % seconds to skip at the beginning (0 to disable)

% Column indices in the CSV (NOTE: original were 0-based; MATLAB uses 1-based)
% Python -> MATLAB = +1 offset
TIME_COL = 0 + 1;

% pd_4, pd_7, pd_8
DESIRED_PRESSURE_COLS = [1, 2, 3] + 1;

% Segment-based groups (see your Python comments; +1 for MATLAB)
MEASURED_PRESSURE_SEGMENT1_COLS = [4, 5, 6, 7, 8] + 1;
MEASURED_PRESSURE_SEGMENT2_COLS = [9, 10, 11, 12, 13] + 1;
MEASURED_PRESSURE_SEGMENT3_COLS = [14] + 1;
MEASURED_PRESSURE_SEGMENT4_COLS = [15] + 1;

% Mocap Body 3 position & quaternion (indices +1)
MOCAP_POS_X_COL = 30 + 1;
MOCAP_POS_Y_COL = 31 + 1;
MOCAP_POS_Z_COL = 32 + 1;
MOCAP_QUAT_SLICE = 33 + 1 : 36 + 1;  % qx,qy,qz,qw

% Derived column names
YAW_BODY_NAME   = "yaw_body";
PITCH_BODY_NAME = "pitch_body";
ROLL_BODY_NAME  = "roll_body";

% ----------------------------- PLOT CONFIGS ------------------------------
% Use struct arrays to mimic the Python dicts
MOCAP_PLOT_CONFIG = [ ...
    struct( ...
        'title',"Mocap Position (Body 3 - Trajectory)", ...
        'xlabel',"Time (s)", 'ylabel',"Position (m)", ...
        'columns', [MOCAP_POS_X_COL, MOCAP_POS_Y_COL, MOCAP_POS_Z_COL], ...
        'labels', ["X Position","Y Position","Z Position"], ...
        'colors', ["tab:blue","tab:orange","tab:green"] ) ...
    , struct( ...
        'title',"Mocap Yaw Orientation (Body 3 - Trajectory)", ...
        'xlabel',"Time (s)", 'ylabel',"Yaw (rad)", ...
        'columns', string(YAW_BODY_NAME), ...
        'labels', "Yaw", ...
        'colors', "tab:red" ) ...
    , struct( ...
        'title',"Mocap Pitch Orientation (Body 3 - Trajectory)", ...
        'xlabel',"Time (s)", 'ylabel',"Pitch (rad)", ...
        'columns', string(PITCH_BODY_NAME), ...
        'labels', "Pitch", ...
        'colors', "tab:purple" ) ...
];

SENSOR_CONTROL_CONFIG_1 = [ ...
    struct( ...
        'title',"Desired Pressures", ...
        'xlabel',"Time (s)", 'ylabel',"Desired Pressure (PSI)", ...
        'columns', DESIRED_PRESSURE_COLS, ...
        'labels', ["Desired Pressure 1","Desired Pressure 2","Desired Pressure 3"], ...
        'colors', ["tab:red","tab:orange","tab:blue"] ) ...
    , struct( ...
        'title',"Measured Pressures (Segment 3 - Third Wave, t~82ms)", ...
        'xlabel',"Time (s)", 'ylabel',"Sensor Pressure (PSI)", ...
        'columns', MEASURED_PRESSURE_SEGMENT3_COLS, ...
        'labels', ["Sensor 9 (pm_9)","Sensor 10 (pm_10)"], ... % keep as-is from Python
        'colors', ["tab:blue","tab:cyan"] ) ...
    , struct( ...
        'title',"Measured Pressures (Segment 4 - Final Sensors)", ...
        'xlabel',"Time (s)", 'ylabel',"Sensor Pressure (PSI)", ...
        'columns', MEASURED_PRESSURE_SEGMENT4_COLS, ...
        'labels', ["Sensor 11 (pm_11)","Sensor 12 (pm_12)","Sensor 13 (pm_13)", ...
                   "Sensor 14 (pm_14)","Sensor 15 (pm_15)","Sensor 16 (pm_16)"], ...
        'colors', ["tab:purple","tab:gray","plum","mediumpurple","violet","indigo"] ) ...
];

SENSOR_CONTROL_CONFIG_2 = [ ...
    struct( ...
        'title',"Measured Pressures (Segment 1 - First Wave, t~0ms)", ...
        'xlabel',"Time (s)", 'ylabel',"Sensor Pressure (PSI)", ...
        'columns', MEASURED_PRESSURE_SEGMENT1_COLS, ...
        'labels', ["Sensor 1 (pm_1)","Sensor 2 (pm_2)","Sensor 3 (pm_3)","Sensor 4 (pm_4)"], ...
        'colors', ["tab:red","tab:pink","crimson","tab:brown"] ) ...
    , struct( ...
        'title',"Measured Pressures (Segment 2 - Second Wave, t~41ms)", ...
        'xlabel',"Time (s)", 'ylabel',"Sensor Pressure (PSI)", ...
        'columns', MEASURED_PRESSURE_SEGMENT2_COLS, ...
        'labels', ["Sensor 5 (pm_5)","Sensor 6 (pm_6)","Sensor 7 (pm_7)","Sensor 8 (pm_8)"], ...
        'colors', ["tab:orange","tab:olive","gold","orange"] ) ...
];

% ================================ MAIN ===================================
% filename = get_experiment(EXPERIMENTS_BASE_DIR); % optional auto-discovery
% filename = "/home/g1/Developer/RISE_Lab/colcon_ws/experiments/June-23/Test_5.csv";
% filename = "experiments/October-15/Experiment_8.csv";
filename = "experiments/October-09/Experiment_8.csv";

if ~isfile(filename)
    error('File not found: %s', filename);
end

fprintf('\nAnalyzing:\n%s\n\n', filename);
T = readtable(filename, 'TextType','string');

if isempty(T)
    error('Data could not be read.');
end

% Time vector
time = T{:, TIME_COL};
if ~isvector(time)
    error('TIME_COL does not reference a single column.');
end

% Apply start time offset if applicable
if time(end) >= START_TIME_OFFSET_SEC
    fprintf('Slicing data to start from %.3f seconds.\n', START_TIME_OFFSET_SEC);
    start_idx = find(time >= START_TIME_OFFSET_SEC, 1, 'first');
    T = T(start_idx:end, :);
    time = time(start_idx:end);
end

% Derived signals from quaternions (if present)
derived = containers.Map('KeyType','char','ValueType','any');
try
    quat = T{:, MOCAP_QUAT_SLICE};
    if size(quat,2) ~= 4
        error('Quaternion slice does not have 4 columns.');
    end
    qx = quat(:,1); qy = quat(:,2); qz = quat(:,3); qw = quat(:,4);

    derived(char(YAW_BODY_NAME))   = quaternion_to_yaw(qx,qy,qz,qw);
    derived(char(PITCH_BODY_NAME)) = quaternion_to_pitch(qx,qy,qz,qw);
    derived(char(ROLL_BODY_NAME))  = quaternion_to_roll(qx,qy,qz,qw);
catch ME
    warning('Could not calculate orientation from quaternions: %s', ME.message);
end

headers = string(T.Properties.VariableNames);
base_title = string(get_file_basename(filename));

% Figure 1: Desired + Segments 3 & 4
create_plot_window(1, SENSOR_CONTROL_CONFIG_1, T, derived, time, headers, ...
    "Sensor & Control Data (Desired Pressure, Segments 3 & 4): " + base_title);

% Figure 2: Segments 1 & 2
create_plot_window(2, SENSOR_CONTROL_CONFIG_2, T, derived, time, headers, ...
    "Sensor & Control Data (Segments 1 & 2): " + base_title);

% Figure 3: Mocap time-series
create_plot_window(3, MOCAP_PLOT_CONFIG, T, derived, time, headers, ...
    "Mocap Data (Time Series - Body 3): " + base_title);

% Figure 4: 3D Mocap
create_3d_mocap_plot(4, T, "Mocap 3D Trajectory (Body 3): " + base_title);

end

% ============================== FUNCTIONS ================================

function filename = get_experiment(experiments_base_dir)
% Finds latest date-named folder (e.g., 'June-25'), then latest Test_X_Y.csv
% or Experiment_X.csv inside it, similar to your Python logic.

if ~isfolder(experiments_base_dir)
    error("Error: The specified experiment directory was not found. Check the path.");
end

d = dir(experiments_base_dir);
isValidFolder = [d.isdir] & ~startsWith({d.name}, '.');
names = string({d(isValidFolder).name});

% Regex: "MonthName-DayNumber"
dateMask = ~cellfun(@isempty, regexp(names, '^[A-Za-z]+-\d{1,2}$', 'once'));
dateFolders = names(dateMask);

if isempty(dateFolders)
    error('No valid date-named folders found in experiments.');
end

% Convert folder name (e.g., "June-25") to datetime in current year
thisYear = year(datetime('now'));
folderDates = NaT(size(dateFolders));
for i = 1:numel(dateFolders)
    try
        folderDates(i) = datetime(dateFolders(i), 'InputFormat','MMMM-d');
        folderDates(i).Year = thisYear;
    catch
        % leave as NaT
    end
end

valid = ~isnat(folderDates);
if ~any(valid)
    error('No valid date-named folders found in experiments.');
end

dateFolders = dateFolders(valid);
folderDates = folderDates(valid);

% Sort by most recent; handle year wrapping by assuming future month => previous year
nowDT = datetime('now');
adjYears = folderDates.Year - (month(folderDates) > month(nowDT));
sortKey = folderDates; sortKey.Year = adjYears;
[~, idx] = sort(sortKey, 'descend');
latestFolder = dateFolders(idx(1));

latestPath = fullfile(experiments_base_dir, latestFolder);
files = dir(latestPath);
names = string({files(~[files.isdir]).name});

mask = ~cellfun(@isempty, regexp(names, '^(Test_\d+_\d+|Experiment_\d+)\.csv$', 'once'));
candidates = names(mask);
if isempty(candidates)
    error("Latest date folder (%s) contains no valid 'Test_X_Y.csv' or 'Experiment_X.csv' files.", latestFolder);
end

% Extract final numeric token to rank tests
nums = nan(size(candidates));
for i = 1:numel(candidates)
    tok = regexp(candidates(i), 'Test_\d+_(\d+)\.csv$', 'tokens', 'once');
    if ~isempty(tok)
        nums(i) = str2double(tok{1});
    else
        tok = regexp(candidates(i), 'Experiment_(\d+)\.csv$', 'tokens', 'once');
        if ~isempty(tok)
            nums(i) = str2double(tok{1});
        end
    end
end

if all(isnan(nums))
    error("Latest date folder (%s) contains no valid 'Test_X_Y.csv' or 'Experiment_X.csv' files.", latestFolder);
end

[~, j] = max(nums);
filename = fullfile(latestPath, candidates(j));
fprintf('Latest experiment file: %s\n', filename);
end

function r = quaternion_to_roll(qx,qy,qz,qw)
% Roll (X)
r = atan2( 2*(qw.*qx + qy.*qz), 1 - 2*(qx.^2 + qy.^2) );
end

function p = quaternion_to_pitch(qx,qy,qz,qw)
% Pitch (Y)
arg = 2*(qw.*qy - qz.*qx);
arg = max(min(arg, 1.0), -1.0);
p = asin(arg);
end

function y = quaternion_to_yaw(qx,qy,qz,qw)
% Yaw (Z)
y = -atan2( 2*(qw.*qz + qx.*qy), 1 - 2*(qy.^2 + qz.^2) );
end

function create_plot_window(figNum, plotConfigs, T, derived, time, headers, window_title)
    figure(figNum); clf;
    numPlots = numel(plotConfigs);
    tiledlayout(numPlots,1, 'Padding','compact', 'TileSpacing','loose');
    sgtitle(window_title, 'FontSize', 22, 'FontWeight','bold');

    for k = 1:numPlots
        cfg = plotConfigs(k);
        nexttile; hold on;

        % ---- normalize columns to a cell array ----
        cols = cfg.columns;
        if isnumeric(cols)
            cols = num2cell(cols);
        elseif isstring(cols)
            cols = cellstr(cols);
        elseif ischar(cols)
            cols = {cols};
        elseif ~iscell(cols)
            error('Unsupported type for cfg.columns.');
        end

        for i = 1:numel(cols)
            ci = cols{i};

            if (ischar(ci)) || (isstring(ci) && isscalar(ci))
                % derived series by name
                key = char(ci);
                if isKey(derived, key)
                    ydata = derived(key);
                    lbl   = label_at(cfg.labels, i, key);
                    plot(time, ydata, 'LineWidth', 2.5, ...
                        'Color', getColor(cfg.colors, i), ...
                        'DisplayName', char(lbl));
                else
                    warning('Derived key "%s" not found. Skipping.', key);
                end

            elseif isnumeric(ci) && isscalar(ci)
                % raw column by index
                colIdx = ci;
                if colIdx >= 1 && colIdx <= width(T)
                    ydata = T{:, colIdx};
                    lbl   = label_at(cfg.labels, i, headers(colIdx));
                    plot(time, ydata, 'LineWidth', 2.5, ...
                        'Color', getColor(cfg.colors, i), ...
                        'DisplayName', char(lbl));
                else
                    warning('Column index %d out of range. Skipping.', colIdx);
                end

            else
                warning('Unsupported column entry at index %d. Skipping.', i);
            end
        end

        xlabel(cfg.xlabel, 'FontSize', 14);
        ylabel(cfg.ylabel, 'FontSize', 14);
        title(cfg.title, 'FontSize', 16);
        grid on; box on;
        legend('Location','northeast', 'FontSize', 11);
        hold off;
    end
end


function create_3d_mocap_plot(figNum, T, window_title)
    figure(figNum); clf;

    % Access columns without evalin if these are in scope; otherwise keep as-is
    x = T{:, evalin('caller','MOCAP_POS_X_COL')};
    y = T{:, evalin('caller','MOCAP_POS_Y_COL')};
    z = T{:, evalin('caller','MOCAP_POS_Z_COL')};

    plot3(x, y, z, 'LineWidth', 1.5); hold on;
    scatter3(x(1), y(1), z(1), 100, 'o', 'filled'); % Start
    scatter3(x(end), y(end), z(end), 100, 's', 'filled'); % End
    xlabel('X Position (m)', 'FontWeight','bold');
    ylabel('Y Position (m)', 'FontWeight','bold');
    zlabel('Z Position (m)', 'FontWeight','bold');
    title(window_title, 'FontSize', 20, 'FontWeight','bold');
    legend({'Trajectory','Start','End'}, 'Location','best');
    grid on; box on; axis vis3d;

    % Equalize ranges (fallback without range())
    xr = max(x) - min(x);
    yr = max(y) - min(y);
    zr = max(z) - min(z);
    maxr = max([xr, yr, zr]) / 2;
    if maxr == 0, maxr = 1; end
    midx = (max(x)+min(x))/2;
    midy = (max(y)+min(y))/2;
    midz = (max(z)+min(z))/2;
    xlim([midx-maxr, midx+maxr]);
    ylim([midy-maxr, midy+maxr]);
    zlim([midz-maxr, midz+maxr]);
    view(3);
end

% ---------------------------- Helpers ------------------------------------

function s = get_file_basename(p)
[~, name, ext] = fileparts(p);
s = [name, ext];
end

function c = getColor(colorList, idx)
% Map Python-like color names to MATLAB RGB triples.
% Falls back to lines() colormap if name unknown or out-of-range.
names = string(colorList);
name = names( (mod(idx-1, numel(names)) + 1) );

switch lower(strtrim(name))
    case {'tab:blue'},         c = [0.1216 0.4667 0.7059];
    case {'tab:orange'},       c = [1.0000 0.4980 0.0549];
    case {'tab:green'},        c = [0.1725 0.6275 0.1725];
    case {'tab:red'},          c = [0.8392 0.1529 0.1569];
    case {'tab:purple'},       c = [0.5804 0.4039 0.7412];
    case {'tab:brown'},        c = [0.5490 0.3373 0.2941];
    case {'tab:pink'},         c = [0.8588 0.4196 0.5765];
    case {'tab:gray','tab:grey'}, c = [0.6196 0.6196 0.6196];
    case {'tab:olive'},        c = [0.5490 0.7176 0.3216];
    case {'tab:cyan'},         c = [0.0902 0.7451 0.8118];
    case {'gold'},             c = [1.0000 0.8431 0.0000];
    case {'orange'},           c = [1.0000 0.6471 0.0000];
    case {'crimson'},          c = [0.8627 0.0784 0.2353];
    case {'plum'},             c = [0.8667 0.6275 0.8667];
    case {'mediumpurple'},     c = [0.5765 0.4392 0.8588];
    case {'violet'},           c = [0.9333 0.5098 0.9333];
    case {'indigo'},           c = [0.2941 0.0000 0.5098];
    otherwise
        L = lines(8);
        c = L( mod(idx-1, size(L,1)) + 1, : );
end
end

function lbl = label_at(labels, i, fallback)
lbls = string(labels);
if i <= numel(lbls) && strlength(lbls(i))>0
    lbl = lbls(i);
else
    lbl = string(fallback);
end
end
