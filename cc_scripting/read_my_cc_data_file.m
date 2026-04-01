folder_path = "/home/sebastian/Documents/Primitives/ConformanceChecking/CC/CC-Data/Exported_Recordings";
rec_id = 6;

%% Hints:
%% - Recs 5 and 9 contain driver intervention

%% TODO first:
%% - debug into function below
%% - plot y and y_ref with time
%% - check in which time frame they roughly overlap

rec_id_2_start_time_idx = containers.Map( ...
    [3,    4,    5, 6,    7,   8,    9, 10,   11], ...
    [1497, 1634, 0, 1000, 772, 1048, 0, 1092, 954]);

rec_id_2_end_time_idx = containers.Map( ...
    [3,    4,    5, 6,    7,    8,    9, 10,   11], ...
    [2866, 3181, 0, 2111, 2166, 2559, 1, 2601, 2381]);

% 6: 1643

start_time = rec_id_2_start_time_idx(rec_id);
end_time = rec_id_2_end_time_idx(rec_id);

shift_ref = 0; % 80; %50;

res = read_file(folder_path, rec_id, [start_time, end_time], shift_ref);

out_file_path = sprintf('out/cc_data/record_opt_2_%d', rec_id);
if exist(out_file_path, 'file')
    error('Output file %s already exists', out_file_path);
end

save(out_file_path, 'res');
