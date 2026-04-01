classdef PrimitiveSetGenerator < handle
    properties (Access = private)
        settings PrimitiveGenerationSettings
        reach_performer

        existing_primitives dictionary = dictionary()
        traj_gen ValidInpTrajGenerator

        sim {must_be_of_type_or_empty(sim, 'RandomCoraSimulator')} = []
        sim_sampler {must_be_of_type_or_empty(sim_sampler, 'SimulationSampler')} = []
        num_sims uint32 = 10
    end

    methods (Access = public)

        function obj = PrimitiveSetGenerator( ...
                settings, reach_performer, ...
                params, inp_traj_gen, model_for_ref_traj, ...
                sim, sim_sampler, num_sims)
            obj.settings = settings;
            obj.reach_performer = reach_performer;

            deactivate_cora_warning();

            obj.existing_primitives = read_primitive_id_set(settings.result_dir.primitive_ids_file_path);

            %% TODO maybe use slightly stronger :constraints for trajectories,
            %% i.e., tighter state and input bounds and also smaller maximum
            %% friction forces.
            obj.traj_gen = ValidInpTrajGenerator(inp_traj_gen, settings.terminal_cond, ...
                settings.primitive_duration, ...
                settings.dt_control, ...
                settings.time_decimal_places, ...
                model_for_ref_traj, settings.k, settings.i, params, ...
                settings.constraints, ...
                settings.extended_horizon, settings.process_final_state);

            fprintf('Existing primitives: %d \n', numEntries(obj.existing_primitives));

            assert(isempty(sim) == isempty(sim_sampler), "Sim and sim_sampler must be either both empty or both non-empty");
            if ~isempty(sim) && ~isempty(sim_sampler)
                obj.sim = sim;
                obj.sim_sampler = sim_sampler;

                if exist('num_sims', 'var')
                    obj.num_sims = num_sims;
                end
            end
        end

        function append_to_primitive_ids_file(obj, p)
            fid = fopen(obj.settings.result_dir.primitive_ids_file_path, 'a+');
            fprintf(fid, '%s\n%s\n', p.json, string(p.final_set_id));
            fclose(fid);
        end

        function start_generation(obj, num_workers)
            if ~exist('num_workers', 'var')
                fprintf("Number of workers not specified. Taking maximum...\n");
                myCluster = parcluster('local');
                num_workers = myCluster.NumWorkers;
            end
            fprintf("Number of workers: %d.\n", num_workers);

            [batch_idx, start_sets] = obj.settings.state_space_divider.next_batch();

            % TODO only for hierarchical-single-path
            % start_sets = start_sets(1:1);

            while ~isempty(start_sets)
                fprintf('#Current batch size: %d. \n', length(start_sets));

                primitives_checkpoint_file_path = strcat( ...
                    obj.settings.result_dir.primitive_scaffolds_out_path, ...
                    '/', string(batch_idx), '.mat');
                if isfile(primitives_checkpoint_file_path)
                    fprintf('Loading primitives from checkpoint file: %s\n', primitives_checkpoint_file_path);
                    load(primitives_checkpoint_file_path, 'primitives');
                else
                    primitives = obj.build_primitive_scaffolds(start_sets, batch_idx, num_workers);
                    save(primitives_checkpoint_file_path, 'primitives', '-v7.3');
                end

                % TODO: dump primitives. Generate primitives in parallel for non-hierarchical and test it
                % for one primitive and other things...(export, subset-relation, restarting, etc.)

                for i = 1:length(primitives)
                    p_final_set_id = obj.settings.state_space_divider.set_id_manager.gen_next_id();
                    primitives{i}.set_final_set_id(p_final_set_id);
                end

                fprintf('#Primitives: %d. \n', length(primitives));
                fprintf('#Primitives/start_set in avg: %d. \n', length(primitives) / length(start_sets));

                % TODO only for hierarchical-single-path
                % primitives = primitives(1:1);

                primitives = obj.perform_reach(primitives, num_workers);

                obj.settings.state_space_divider.after_primitives_generated(primitives);

                [batch_idx, start_sets] = obj.settings.state_space_divider.next_batch();
            end

            save_subset_relation(obj.settings.state_space_divider.get_subset_relation(), ...
                obj.settings.result_dir.out_path_export);
        end

    end

    methods (Access = private)

        function primitives = build_primitive_scaffolds(obj, start_sets, batch_idx, num_workers)
            settings_ = obj.settings;
            traj_gen_ = obj.traj_gen;

            primitives = cell(length(start_sets), 1);

            sub_out_path = strcat(obj.settings.result_dir.out_path_internal, ...
                "/batch_", num2str(batch_idx));
            [~, ~] = mkdir(sub_out_path);

            if num_workers == 1
                for i = 1:length(start_sets)
                    primitives{i} = build_primitive_scaffolds_for_start_set( ...
                        start_sets{i}, sub_out_path, settings_, traj_gen_);
                end
            else
                parfor (i = 1:length(start_sets), num_workers)
                    primitives{i} = build_primitive_scaffolds_for_start_set( ...
                        start_sets{i}, sub_out_path, settings_, traj_gen_);
                end
            end
            primitives = vertcat(primitives{:});
        end

        function results = perform_reach(obj, primitives, num_workers)
            % These handles are broadcasted to all workers. Note that they
            % only perform read access, thus this should not impede performance
            % re-declare here for avoiding broadcast-variable-warning for obj
            settings_ = parallel.pool.Constant(obj.settings);
            reach_performer_ = parallel.pool.Constant(obj.reach_performer);
            existing_primitives_ = parallel.pool.Constant(obj.existing_primitives);

            sim_ = parallel.pool.Constant(obj.sim);
            sim_sampler_ = parallel.pool.Constant(obj.sim_sampler);
            num_sims_ = parallel.pool.Constant(obj.num_sims);

            q = parallel.pool.DataQueue;
            afterEach(q, @(p) obj.append_to_primitive_ids_file(p));

            results = cell(length(primitives), 1);

            if num_workers == 1
                for i = 1:length(primitives)
                    results{i} = perform_reach_for_primitive(primitives{i}, ...
                        settings_.Value, reach_performer_.Value, ...
                        existing_primitives_.Value, q, sim_.Value, sim_sampler_.Value, num_sims_.Value);
                end
            else
                chunk_size = 64 * 30;
                num_chunks = ceil(length(primitives) / chunk_size);
                for c = 1:num_chunks
                    fprintf('Chunk: %d/%d\n', c, num_chunks);
                    % we create a parallel pool if no exists
                    parallel_pool = gcp;

                    chunk_start = (c - 1) * chunk_size + 1;
                    chunk_end = min(c * chunk_size, length(primitives));
                    parfor (i = chunk_start:chunk_end, num_workers)
                        results{i} = perform_reach_for_primitive(primitives{i}, ...
                            settings_.Value, reach_performer_.Value, ...
                            existing_primitives_.Value, q, sim_.Value, sim_sampler_.Value, num_sims_.Value);
                    end
                    % we shut down the parallel pool to free memory
                    % if more than 70% is used
                    [total_mem, used_mem] = get_memory_usage();
                    fprintf('Mem: %d/%d\n', used_mem, total_mem);
                    if used_mem / total_mem > 0.7
                        delete(parallel_pool);
                    end
                end
            end
        end

    end
end
