function filename = GetExperiment()
%GetExperiment Finds the correct experiment file to analyze.
%
%   WORKFLOW:
%   1. Checks the Downloads folder for new 'Test_*.txt' files.
%   2. If found, moves the latest one to the appropriate dated
%      'experiments' subfolder and returns its new path.
%   3. If NOT found, searches the 'experiments' directory for the
%      most recent existing file and returns its path.
%
%   Returns:
%       filename (string): The full path to the file that should be
%                          analyzed.
%
%   Throws:
%       An error if no files are found in Downloads AND no valid files
%       exist in the experiments directory.

% --- Configuration ---
experiments_base_dir = '/home/g1/Developer/RISE_Lab/experiments';
downloads_dir = '/home/g1/Downloads';

fprintf('Step 1: Checking for new files in Downloads folder...\n');
file_list_downloads = dir(fullfile(downloads_dir, 'Test_*.txt'));

% --- SCENARIO 1: New file(s) exist in Downloads ---
if ~isempty(file_list_downloads)
    fprintf('-> Found new file(s) in Downloads. Organizing the latest one.\n');
    
    % Find the file with the highest two-digit number
    tokens = regexp({file_list_downloads.name}, 'Test_(\d{2})\.txt', 'tokens', 'once');
    valid_indices = ~cellfun('isempty', tokens);
    
    file_list_valid = file_list_downloads(valid_indices);
    tokens_valid = tokens(valid_indices);
    
    nums = cellfun(@(t) str2double(t{1}), tokens_valid);
    [~, max_idx] = max(nums);
    
    latest_file_info = file_list_valid(max_idx);
    
    % Determine destination and move the file
    date_folder_name = datestr(now, 'mmmm-dd');
    destination_folder = fullfile(experiments_base_dir, date_folder_name);
    
    if ~isfolder(destination_folder)
        mkdir(destination_folder);
    end
    
    source_path = fullfile(latest_file_info.folder, latest_file_info.name);
    destination_path = fullfile(destination_folder, latest_file_info.name);
    
    [status, msg] = movefile(source_path, destination_path);
    
    if status
        filename = destination_path; % Return the path of the moved file
    else
        error('organizer:MoveFailed', 'File could not be moved. Error: %s', msg);
    end

% --- SCENARIO 2: No new files in Downloads, use latest from experiments ---
else
    fprintf('-> No new files in Downloads. Finding latest existing file in experiments folder.\n');
    
    base_path = experiments_base_dir;
    d = dir(base_path);
    d = d([d.isdir] & ~ismember({d.name},{'.','..'}));
    
    % Find the latest date-named folder robustly
    folder_names = {d.name};
    is_date_folder = cellfun(@(n) ~isempty(regexp(n, '^[A-Za-z]+-\d{2}$', 'once')), folder_names);
    
    date_nums = nan(size(folder_names));
    for k = 1:numel(folder_names)
        if is_date_folder(k)
            try
                date_nums(k) = datenum(folder_names{k}, 'mmmm-dd');
            catch
                date_nums(k) = NaN;
            end
        end
    end
    
    if all(isnan(date_nums))
        error('organizer:NoFilesFound', ...
            'No files in Downloads AND no valid date-folders found in experiments.');
    end
    
    [~, idx] = max(date_nums);
    latest_folder = folder_names{idx};
    
    % Find the latest Test_*.txt file within that folder
    f = dir(fullfile(base_path, latest_folder, 'Test_*.txt'));
    if isempty(f)
        error('organizer:NoTestFiles', ...
            'Latest date folder (%s) contains no valid ''Test_*.txt'' files.', latest_folder);
    end
    
    tokens = regexp({f.name}, 'Test_(\d{2})\.txt', 'tokens', 'once');
    valid = ~cellfun('isempty', tokens);
    
    if ~any(valid)
        error('organizer:NoTestFiles', ...
            'Latest date folder (%s) contains no valid ''Test_*.txt'' files.', latest_folder);
    end
    
    nums = cellfun(@(t) str2double(t{1}), tokens(valid));
    ff = f(valid);
    [~,j] = max(nums);
    
    filename = fullfile(ff(j).folder, ff(j).name); % Return path of existing file
end

end