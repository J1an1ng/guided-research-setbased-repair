folder_path = "/home/sebastian/Documents/Primitives/ConformanceChecking/CC/CC-Data/Exported_Recordings";
rec_id = 6;

%% TODO first:
%% - debug into function below
%% - plot y and y_ref with time
%% - check in which time frame they roughly overlap

rec_id_2_time_idcs = containers.Map( ...
    [6], ...
    [[1000, 1643], ... % [, 2108]
    ]);


res = read_file(folder_path, rec_id, rec_id_2_time_idcs(rec_id));

out_file_path = sprintf('out/cc_data/record_small_%d', rec_id);
if exist(out_file_path, 'file')
    error('Output file %s already exists', out_file_path);
end

save(out_file_path, 'res');