classdef BoxLikeStateSpaceDivider < AbstractOnceStateSpaceDivider
    properties (Constant)
        property_names_to_dump = cell(1, 0)
    end

    properties (Access = private)
        % number of state dimensions
        state_length uint32

        % lower and upper bound for sampling state dimensions
        sampling_dims_bounds dictionary = dictionary()

        % state dimension where only half of the full range of possible
        % value needs to be considered for the initial intervals
        mirrored_dim uint32

        % resulting interval size of sampling state dimensions
        range_sizes dictionary = dictionary()

        step_sizes dictionary = dictionary()

        invert_sampling_order dictionary = dictionary()

        overlap_fractions dictionary = dictionary()
        overlap_abs dictionary = dictionary()

        % intervals for other state dimensions
        other_dims_intervals dictionary = dictionary()

        max_samples_per_dim dictionary = dictionary()

        constraints AbstractConstraints

        terminal_cond AbstractTerminalCondition

        lb_sampler LazySampler

        interval_scaffold (:, 2) double

        final_interval_scaffold (:, 2) double
    end

    properties (Access = public)
        % state dimensions that are subject to sampling
        sampling_dims (1, :) uint32

        % state dimensions that are not subject to sampling
        other_dims (1, :) uint32
    end

    methods

        function obj = BoxLikeStateSpaceDivider( ...
                result_dir, num_workers, constraints, terminal_cond, ...
                state_length, sampling_dims, mirrored_dim, ...
                sampling_dims_bounds, ...
                range_sizes, ...
                invert_sampling_order, ...
                other_dims_intervals, ...
                overlap_fractions, overlap_abs, ...
                max_samples_per_dim)
            obj = obj@AbstractOnceStateSpaceDivider(result_dir, num_workers, ...
                sampling_dims);
            if ~exist("max_samples_per_dim", "var")
                max_samples_per_dim = dictionary(uint32.empty, uint32.empty);
            end
            % Validation of parameters:
            num_sampling_dims = length(sampling_dims);
            assert (overlap_fractions.numEntries + overlap_abs.numEntries == num_sampling_dims);
            assert(num_sampling_dims < state_length);
            assert(any(sampling_dims == mirrored_dim) || isnan(mirrored_dim));
            assert(numEntries(sampling_dims_bounds) == num_sampling_dims);

            assert(numEntries(invert_sampling_order) == num_sampling_dims);

            num_other_dims = state_length - num_sampling_dims;
            assert(numEntries(other_dims_intervals) == num_other_dims);

            obj.constraints = constraints;
            obj.terminal_cond = terminal_cond;
            obj.state_length = state_length;
            obj.sampling_dims = sampling_dims;
            obj.mirrored_dim = mirrored_dim;
            obj.range_sizes = range_sizes;
            obj.other_dims = setdiff(1:state_length, sampling_dims);

            obj.sampling_dims_bounds = sampling_dims_bounds;
            obj.range_sizes = range_sizes;
            obj.invert_sampling_order = invert_sampling_order;
            obj.overlap_fractions = overlap_fractions;
            obj.overlap_abs = overlap_abs;
            obj.other_dims_intervals = other_dims_intervals;
            obj.max_samples_per_dim = max_samples_per_dim;

            if any(obj.sampling_dims == mirrored_dim)
                %% We reduce the lower bound of considered values by half of the
                %% range size
                range_size = obj.range_sizes{mirrored_dim};
                b = obj.sampling_dims_bounds{mirrored_dim};
                b(1) = b(1) - range_size / 2;
                obj.sampling_dims_bounds{mirrored_dim} = b;
            end

            obj.final_interval_scaffold = create_final_interval_scaffold(state_length, ...
                obj.sampling_dims, obj.range_sizes, ...
                obj.other_dims, obj.other_dims_intervals, overlap_fractions, overlap_abs);
            [obj.lb_sampler, step_sizes] = create_lazy_sampler_for_initial_intervals( ...
                obj.sampling_dims, obj.sampling_dims_bounds, ...
                obj.range_sizes, ...
                obj.overlap_fractions, overlap_abs, obj.invert_sampling_order, obj.max_samples_per_dim);
            obj.step_sizes = step_sizes;
            obj.interval_scaffold = obj.create_interval_scaffold();

            obj.load_dump();
        end

        function max_final_set = get_max_final_set(obj, ~)
            final_interval = obj.final_interval_scaffold;
            max_final_set = interval(final_interval(:, 1), final_interval(:, 2));
        end

        function state_sets = all_batches(obj)
            state_sets = obj.sample_all();
            state_sets = box_arrs2set_cell_arr(state_sets, obj.set_id_manager);
        end

    end

    methods (Access = private)

        function state_sets = sample_all(obj)
            valid_samples = ListOf2Ds([obj.state_length 2]);
            num_discarded = 0;
            discarded_itvs = SetOfIntervals(obj.state_length);

            i = 1;
            next_lb = obj.lb_sampler.next();
            while ~isempty(next_lb)
                itv = obj.interval_scaffold;
                itv = update_interval(itv, obj.sampling_dims, ...
                    obj.sampling_dims_bounds, next_lb, obj.range_sizes);

                already_exists = false;
                is_valid = obj.constraints.check_interval(itv);

                if ~is_valid
                    itv = obj.constraints.repair_interval(itv);
                    is_valid = ~is_itv_empty(itv) && obj.constraints.check_interval(itv);
                    if is_valid
                        % currently, it never happens that a repaired interval
                        % already exists. The reason for this is most likely the small
                        % overlap of intervals (only size of measurement uncertainty),
                        % such that - when some interval is invalid first - the next
                        % invalid one already exceeds the friction circle on its lower
                        % bound, so the repaired interval is empty (see line above).
                        already_exists = discarded_itvs.add(itv);
                    end
                end

                is_valid = is_valid && ~obj.terminal_cond.check_for_itv_soft(itv);

                if is_valid && ~already_exists
                    valid_samples.add(itv);
                else
                    num_discarded = num_discarded + 1;
                end

                if mod(i, 1000) == 0
                    fprintf("# Generated %d. Discarded: %d \n", i, num_discarded);
                end

                i = i + 1;

                next_lb = obj.lb_sampler.next();
            end

            state_sets = valid_samples.to_arr();
            fprintf("# Valid %d/%d \n", length(state_sets), i);
        end

        function res = create_interval_scaffold(obj)
            res = zeros(obj.state_length, 2);
            res(obj.other_dims, :) = [ ...
                                      extract_from(obj.other_dims_intervals, 1, obj.other_dims) ...
                                      extract_from(obj.other_dims_intervals, 2, obj.other_dims) ...
                                     ];
        end

    end
end
