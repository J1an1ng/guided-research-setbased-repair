classdef SinglePrimitiveIO < handle
    properties (Constant)
        available_export_formats = ["json"; "hdf5"]
        default_export_format = "hdf5"
    end

    properties (Access = private)
        result_dir ResultDir
        final_set_out_path string

        occ_computer OccupancyComputer
        export_format string
        max_half_width double
    end

    methods

        function obj = SinglePrimitiveIO(result_dir, occ_computer, max_half_width, export_format)
            obj.result_dir = result_dir;
            obj.occ_computer = occ_computer;
            obj.max_half_width = max_half_width;
            if ~exist('export_format', 'var')
                export_format = obj.default_export_format;
            end

            obj.export_format = export_format;
            assert(ismember(export_format, obj.available_export_formats), ...
                "Export format not supported: %s", export_format);
        end

        function save(obj, primitive)
            if primitive.is_valid()
                obj.save_primitive(primitive);
                obj.save_final_set_to_mat(primitive);
            else
                obj.save_errors(primitive);
            end
        end

        function res = final_set_exists(obj, primitive)
            in_file = obj.get_final_set_file_path(primitive);
            res = isfile(in_file);
        end

        function load_final_set(obj, primitive)
            in_file = obj.get_final_set_file_path(primitive);
            if ~isfile(in_file)
                error("File does not exist: %s", in_file);
            end
            load(in_file, 'p_final_set');
            primitive.set_final_set(p_final_set);
        end

        function save_final_set_to_mat(obj, primitive)
            out_file = obj.get_final_set_file_path(primitive);
            check_if_file_exists(out_file);
            p_final_set = primitive.final_set;
            save(out_file, 'p_final_set');
        end

        function save_primitive(obj, primitive)
            switch obj.export_format
                case "json"
                    obj.save_to_json(primitive);
                case "hdf5"
                    obj.save_to_hdf5(primitive);
                otherwise
                    error("Export format not supported: %s", obj.export_format);
            end
        end

        function save_to_json(obj, primitive)
            res = primitive.export_to_struct(obj.occ_computer, obj.max_half_width);
            res = round_all_numbers_in_struct(res, 2);
            json = jsonencode(res, PrettyPrint = true);

            % Sanitize filename by replacing invalid characters
            safe_filename = primitive.json;
            safe_filename = strrep(safe_filename, ':', '_');
            safe_filename = strrep(safe_filename, '"', '');
            safe_filename = strrep(safe_filename, '{', '');
            safe_filename = strrep(safe_filename, '}', '');
            safe_filename = strrep(safe_filename, '/', '_');
            safe_filename = strrep(safe_filename, '\', '_');
            safe_filename = strrep(safe_filename, '*', '_');
            safe_filename = strrep(safe_filename, '?', '_');
            safe_filename = strrep(safe_filename, '<', '_');
            safe_filename = strrep(safe_filename, '>', '_');
            safe_filename = strrep(safe_filename, '|', '_');
            safe_filename = strrep(safe_filename, '[', '_');
            safe_filename = strrep(safe_filename, ']', '_');
            safe_filename = strrep(safe_filename, ',', '_');
            out_file = strcat(obj.result_dir.primitives_out_path, "/", safe_filename, '.json');
            check_if_file_exists(out_file);
            fid = fopen(out_file, 'wt');
            fprintf(fid, json);
            fclose(fid);
            % TODO: maybe delete reachsets of primitive
            % for the sake of saving memory
        end

        function save_to_hdf5(obj, primitive)
            res = primitive.export_to_struct(obj.occ_computer, obj.max_half_width);
            % Sanitize filename by replacing invalid characters
            safe_filename = primitive.json;
            safe_filename = strrep(safe_filename, ':', '_');
            safe_filename = strrep(safe_filename, '"', '');
            safe_filename = strrep(safe_filename, '{', '');
            safe_filename = strrep(safe_filename, '}', '');
            safe_filename = strrep(safe_filename, '/', '_');
            safe_filename = strrep(safe_filename, '\', '_');
            safe_filename = strrep(safe_filename, '*', '_');
            safe_filename = strrep(safe_filename, '?', '_');
            safe_filename = strrep(safe_filename, '<', '_');
            safe_filename = strrep(safe_filename, '>', '_');
            safe_filename = strrep(safe_filename, '|', '_');
            safe_filename = strrep(safe_filename, '[', '_');
            safe_filename = strrep(safe_filename, ']', '_');
            safe_filename = strrep(safe_filename, ',', '_');
            out_file = strcat(obj.result_dir.primitives_out_path, "/", safe_filename, '.h5');
            check_if_file_exists(out_file);
            fid = H5F.create(out_file);
            export_struct_to_hdf5(out_file, fid, res);

            H5F.close(fid);
        end

    end

    methods (Access = private)

        function file_path = get_final_set_file_path(obj, primitive)
            file_path = strcat(obj.result_dir.primitives_final_sets_out_path, ...
                "/", string(primitive.final_set_id), '.mat');
        end

        function save_errors(obj, primitive)
            res = primitive.export_errors();
            json = jsonencode(res, PrettyPrint = true);

            % Sanitize filename by replacing invalid characters
            safe_filename = primitive.json;
            safe_filename = strrep(safe_filename, ':', '_');
            safe_filename = strrep(safe_filename, '"', '');
            safe_filename = strrep(safe_filename, '{', '');
            safe_filename = strrep(safe_filename, '}', '');
            safe_filename = strrep(safe_filename, '/', '_');
            safe_filename = strrep(safe_filename, '\', '_');
            safe_filename = strrep(safe_filename, '*', '_');
            safe_filename = strrep(safe_filename, '?', '_');
            safe_filename = strrep(safe_filename, '<', '_');
            safe_filename = strrep(safe_filename, '>', '_');
            safe_filename = strrep(safe_filename, '|', '_');
            safe_filename = strrep(safe_filename, '[', '_');
            safe_filename = strrep(safe_filename, ']', '_');
            safe_filename = strrep(safe_filename, ',', '_');
            out_file = strcat(obj.result_dir.primitives_out_path, "/", safe_filename, '.json');
            check_if_file_exists(out_file);
            fid = fopen(out_file, 'wt');
            fprintf(fid, json);
            fclose(fid);
        end

    end
end

function check_if_file_exists(file_path)
    if isfile(file_path)
        warning("Should not happen: Overwrite existing. %s", file_path);
        delete(file_path);
    end
end
