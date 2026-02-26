classdef PrimitiveSaver < handle
    properties (Access = private)
        to_mat logical
        to_cr logical

        mat_out_path string
        cr_out_path string

        interval_str string
        num_primitives_per_file uint32

        occ_computer OccupancyComputer

        bulk_sub_counter uint32
        bulk_counter uint32
        bulk cell

        callback
    end

    methods

        function obj = PrimitiveSaver(to_mat, to_cr, ...
                interval_str, num_primitives_per_file, out_path, ...
                occ_computer, callback)
            if ~to_mat && ~to_cr
                warning("The generated primitives will not be saved");
            end

            obj.to_mat = to_mat;
            obj.to_cr = to_cr;

            obj.interval_str = interval_str;
            obj.num_primitives_per_file = num_primitives_per_file;

            p = primitives_folder_path(out_path);
            obj.mat_out_path = strcat(p, "mat/");
            obj.cr_out_path = strcat(p, "commonroad/");
            obj.create_dirs();

            obj.occ_computer = occ_computer;

            obj.bulk_sub_counter = 1;
            obj.bulk_counter = 1;
            obj.bulk = cell(num_primitives_per_file, 1);

            obj.callback = callback;
        end

        function add(obj, primitive)
            obj.bulk{obj.bulk_sub_counter} = primitive;
            obj.bulk_sub_counter = obj.bulk_sub_counter + 1;

            if obj.bulk_sub_counter > obj.num_primitives_per_file
                obj.save();
                obj.bulk_sub_counter = 1;
                obj.bulk_counter = obj.bulk_counter + 1;
                obj.bulk = cell(obj.num_primitives_per_file, 1);
            end
        end

        function finish(obj)
            if obj.bulk_sub_counter > 1
                obj.save();
            end
        end

    end

    methods (Access = private)

        function create_dirs(obj)
            if obj.to_mat
                [~, ~] = mkdir(obj.mat_out_path);
            end

            if obj.to_cr
                [~, ~] = mkdir(obj.cr_out_path);
            end
        end

        function res = mat_file_path(obj, file_name)
            res = strcat(obj.mat_out_path, file_name, '.mat');
        end

        function res = json_file_path(obj, file_name)
            res = strcat(obj.cr_out_path, file_name, '.json');
        end

        function res = out_files_exist(obj, file_name)
            res = isfile(obj.mat_file_path(file_name)) || isfile(obj.json_file_path(file_name));
        end

        function save(obj)
            obj.bulk = remove_empty_values(obj.bulk);
            file_name = strcat(obj.interval_str, '--', int2str(obj.bulk_counter));

            while obj.out_files_exist(file_name)
                %% TODO maybe log that this happened
                obj.bulk_counter = obj.bulk_counter + 1;
                file_name = strcat(obj.interval_str, '--', int2str(obj.bulk_counter));
            end

            if obj.to_mat
                obj.save_bulk_to_mat_file(file_name);
            end

            if obj.to_cr
                obj.save_bulk_to_cr_file(file_name);
            end

            obj.callback(obj.bulk);
        end

        function save_bulk_to_mat_file(obj, file_name)
            primitives_bulk = obj.bulk;
            out_file = obj.mat_file_path(file_name);
            if isfile(out_file)
                warning("Should not happen: Overwrite existing mat-file...");
            end
            save(out_file, 'primitives_bulk');
            clear('primitives_bulk');
        end

        function save_bulk_to_cr_file(obj, file_name)
            res.motion_primitives = cellfun(@(p) p.export_to_commonroad(obj.occ_computer), obj.bulk);
            json = jsonencode(res, PrettyPrint = true);

            out_file = obj.json_file_path(file_name);
            if isfile(out_file)
                warning("Should not happen: Overwrite existing json-file...");
            end
            fid = fopen(out_file, 'wt');
            fprintf(fid, json);
            fclose(fid);
        end

    end
end

function res = primitives_folder_path(out_path)
    res = strcat(out_path, "/primitives/");
end
