classdef (Abstract) AbstractStateSpaceDivider < handle & matlab.mixin.Heterogeneous
    properties (Abstract, Constant)
        property_names_to_dump (:, 1) string
    end

    properties (Constant, Access = private)
        property_names_to_dump_default = ["set_id_manager"; ...
                                          "current_batch_idx"; ...
                                          "current_batch"; ...
                                          "subset_relation"]
    end

    properties (GetAccess = public, SetAccess = protected)
        set_id_manager SetContainerIdManager = SetContainerIdManager()
    end

    properties (Access = protected)
        result_dir ResultDir

        current_batch_idx = 1
        current_batch (:, 1) cell
        subset_relation dictionary = dictionary(uint32.empty, uint32.empty)

        use_checkpoint_next = false
    end

    methods (Abstract, Access = protected)
        batch = generate_next_batch(obj)
    end

    methods (Abstract, Access = public)
        max_final_set = get_max_final_set(obj, center_ref)

        after_primitives_generated(obj, primitives)
    end

    methods (Access = public)

        function obj = AbstractStateSpaceDivider(result_dir)
            obj.result_dir = result_dir;
        end

        function [batch_idx, batch] = next_batch(obj)
            batch_idx = obj.current_batch_idx;
            if obj.use_checkpoint_next
                fprintf('Using checkpoint for next batch.\n');
                obj.use_checkpoint_next = false;
                batch = obj.current_batch;
            else
                batch = obj.generate_next_batch();
                obj.current_batch = batch;
                obj.save_batch_to_json();
                obj.dump();
            end
            obj.current_batch_idx = obj.current_batch_idx + 1;
        end

        function set_set_id_manager(obj, set_id_manager)
            obj.set_id_manager = set_id_manager;
        end

        function res = get_subset_relation(obj)
            res = dictionary(obj.subset_relation);
        end

    end

    methods (Access = protected)

        function dump(obj)
            file_path = obj.get_current_checkpoint_file_path();
            if isfile(file_path)
                fprintf('Checkpoint will be overwritten.\n');
            end
            dump_obj = [];
            fprintf('Dump...\n');
            dump_obj = obj.transfer_dump_props(obj, dump_obj);
            save(file_path, 'dump_obj');
            fprintf('...done.\n');
        end

        function load_dump(obj)
            latest_file_path = obj.get_latest_checkpoint_file_path();
            fprintf('Latest checkpoint found: %s\n', latest_file_path);
            if ~isfile(latest_file_path)
                fprintf('State space divider checkpoint does not exist, skipping.\n');
                return
            end
            fprintf('Load dump...\n');
            load(latest_file_path, 'dump_obj');
            obj.transfer_dump_props(dump_obj, obj);
            fprintf('...done.\n');
            obj.use_checkpoint_next = true;
        end

    end

    methods (Access = private)

        function res = get_base_checkpoint_file_path(obj, idx)
            res = strcat(obj.result_dir.out_path_internal, ...
                '/checkpoint_for_', class(obj), '_', string(idx), '.mat');
        end

        function res = get_current_checkpoint_file_path(obj)
            res = obj.get_base_checkpoint_file_path(obj.current_batch_idx);
        end

        function res = get_latest_checkpoint_file_path(obj)
            idx = 1;
            file_path = obj.get_base_checkpoint_file_path(idx);
            while isfile(file_path)
                idx = idx + 1;
                file_path = obj.get_base_checkpoint_file_path(idx);
            end
            res = obj.get_base_checkpoint_file_path(idx - 1);
        end

        function prop_names = get_prop_names_to_dump(obj)
            prop_names = num2cell([obj.property_names_to_dump; obj.property_names_to_dump_default]);
        end

        function target = transfer_dump_props(obj, source, target)
            prop_names = obj.get_prop_names_to_dump();
            for i = 1:length(prop_names)
                prop_name = prop_names{i};
                target.(prop_name) = source.(prop_name);
            end
        end

        function save_batch_to_json(obj)
            out_file = strcat(obj.result_dir.initial_sets_out_path, ...
                '/batch_', num2str(obj.current_batch_idx), '.json');
            if isfile(out_file)
                warning("Should not happen: Overwrite existing json-file...");
            end

            % we only need to export sets that have an inner base set, as they are
            % necessary for covering the whole state space
            % has_inner_base_set = cellfun(@(x) x.has_inner_base_set, obj.current_batch);
            % current_batch_to_export = obj.current_batch(has_inner_base_set);
            % TODO possibly only export :current_batch_to_export instead of obj.current_batch;
            % currently, the sets without an inner base set are still exported, but have [] as value

            set_ids = collect_set_ids(obj.current_batch);
            set_ids = string(set_ids); % necessary for jsonencode
            % jsonencode does not support dictionaries
            if isempty(set_ids)
                sets_map = containers.Map();
            else
                sets_map = containers.Map(set_ids, obj.current_batch);
            end

            json = jsonencode(sets_map, PrettyPrint = true);

            fid = fopen(out_file, 'wt');
            fprintf(fid, json);
            fclose(fid);
        end

    end
end
