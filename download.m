function success = downloadLatestFromSlack()
%downloadLatestFromSlack Downloads the latest Test_*.txt file from a DM.
%   This function connects to the Slack API, uses a specific DM
%   Conversation ID, and downloads the latest 'Test_*.txt' file found
%   within that conversation's recent history.

% --- Configuration (IMPORTANT: FILL THIS IN) ---
% Paste your NEW, SECRET token here.
api_token = 'xoxb-12670066832-9027067729191-FIEv9tQocbgDbGEydUVzuDt3'; 

% Paste the Conversation ID from the Slack URL.
dm_channel_id = 'D08VDLWHJMB'; 

% --- Setup ---
success = false;
if strcmp(api_token, 'xoxb-YOUR-NEW-TOKEN-HERE')
    error('SlackAPI:NoToken', 'Please enter your Slack API token in the downloadLatestFromSlack.m script.');
end

options = weboptions(...
    'HeaderFields', {'Authorization', ['Bearer ' api_token]}, ...
    'ContentType', 'json', 'Timeout', 30);

% --- 1. Find the Latest File Directly in the DM History ---
fprintf('Searching for files in DM conversation: %s\n', dm_channel_id);
history_url = 'https://slack.com/api/conversations.history';
history_params = {'channel', dm_channel_id, 'limit', 50};

try
    response = webread(history_url, history_params{:}, options);
catch ME
    error('SlackAPI:RequestFailed', 'Failed to get DM history. Check API token, channel ID, and internet. Details: %s', ME.message);
end

if ~response.ok
    error('SlackAPI:ResponseError', 'Slack API error getting history: %s', response.error);
end

latest_file_info = [];
max_num = -1;

% Loop through all messages to find the latest matching file
for i = 1:length(response.messages)
    msg = response.messages(i);
    if isfield(msg, 'files')
        for j = 1:length(msg.files)
            file = msg.files(j);
            tokens = regexp(file.name, 'Test_(\d+)\.txt', 'tokens', 'once');
            if ~isempty(tokens)
                num = str2double(tokens{1});
                if num > max_num
                    max_num = num;
                    latest_file_info = file;
                end
            end
        end
    end
end

if isempty(latest_file_info)
    fprintf('-> No ''Test_*.txt'' files found in the recent DM history.\n');
    return; % Exit function, success remains false
end

fprintf('-> Found latest file on Slack: %s\n', latest_file_info.name);

% --- 2. Download the File ---
user_home = char(java.lang.System.getProperty('user.home'));
downloads_dir = fullfile(user_home, 'Downloads');
local_filepath = fullfile(downloads_dir, latest_file_info.name);

fprintf('Downloading to: %s\n', local_filepath);
try
    websave(local_filepath, latest_file_info.url_private, options);
    fprintf('✅ Download complete.\n');
    success = true;
catch ME
    error('SlackAPI:DownloadFailed', 'Failed to download the file. Details: %s', ME.message);
end

end