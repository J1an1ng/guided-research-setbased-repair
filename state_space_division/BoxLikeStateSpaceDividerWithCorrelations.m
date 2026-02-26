classdef BoxLikeStateSpaceDividerWithCorrelations < AbstractOnceStateSpaceDivider
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

        % contains an entry A -> B if the interval size of A
        % depends on the value of B
        sampling_dims_correlations dictionary = dictionary()

        % for each entry A -> B in :correlated_sampling_dims, the following
        % dict contains an entry A -> fun, where fun maps a value of B to a range size of A
        get_range_sizes_of_correlated_sampling_dims dictionary = dictionary()

        % resulting interval size of sampling state dimensions that are
        % not a key in :correlated_sampling_dims
        range_sizes_of_uncorrelated_sampling_dims dictionary = dictionary()

        step_sizes_of_uncorrelated_sampling_dims dictionary = dictionary()

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

        correlated_sampling_dims (1, :) uint32
        uncorrelated_sampling_dims (1, :) uint32

        % state dimensions that are not subject to sampling
        other_dims (1, :) uint32
    end

    methods

        function obj = BoxLikeStateSpaceDividerWithCorrelations( ...
                result_dir, num_workers, constraints, terminal_cond, ...
                state_length, sampling_dims, mirrored_dim, ...
                sampling_dims_bounds, ...
                sampling_dims_correlations, get_range_sizes_of_correlated_sampling_dims, ...
                range_sizes_of_uncorrelated_sampling_dims, ...
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

            obj.correlated_sampling_dims = keys(sampling_dims_correlations);
            obj.uncorrelated_sampling_dims = keys(range_sizes_of_uncorrelated_sampling_dims);

            assert(isempty(intersect(obj.correlated_sampling_dims, obj.uncorrelated_sampling_dims)));
            assert(isempty(setdiff(sampling_dims, ...
                union(obj.correlated_sampling_dims, obj.uncorrelated_sampling_dims))));
            assert(length(obj.correlated_sampling_dims) + ...
                length(obj.uncorrelated_sampling_dims) == num_sampling_dims);

            assert(numEntries(invert_sampling_order) == num_sampling_dims);

            num_other_dims = state_length - num_sampling_dims;
            assert(numEntries(other_dims_intervals) == num_other_dims);

            obj.constraints = constraints;
            obj.terminal_cond = terminal_cond;
            obj.state_length = state_length;
            obj.sampling_dims = sampling_dims;
            obj.mirrored_dim = mirrored_dim;
            obj.other_dims = setdiff(1:state_length, sampling_dims);

            obj.sampling_dims_bounds = sampling_dims_bounds;
            obj.sampling_dims_correlations = sampling_dims_correlations;
            obj.get_range_sizes_of_correlated_sampling_dims = get_range_sizes_of_correlated_sampling_dims;
            obj.range_sizes_of_uncorrelated_sampling_dims = range_sizes_of_uncorrelated_sampling_dims;
            obj.invert_sampling_order = invert_sampling_order;
            obj.overlap_fractions = overlap_fractions;
            obj.overlap_abs = overlap_abs;
            obj.other_dims_intervals = other_dims_intervals;
            obj.max_samples_per_dim = max_samples_per_dim;

            if any(obj.uncorrelated_sampling_dims == mirrored_dim)
                %% We reduce the lower bound of considered values by half of the
                %% range size
                range_size = obj.range_sizes_of_uncorrelated_sampling_dims{mirrored_dim};
                b = obj.sampling_dims_bounds{mirrored_dim};
                b(1) = b(1) - range_size / 2;
                obj.sampling_dims_bounds{mirrored_dim} = b;
            end

            obj.final_interval_scaffold = create_final_interval_scaffold(state_length, ...
                obj.uncorrelated_sampling_dims, obj.range_sizes_of_uncorrelated_sampling_dims, ...
                obj.other_dims, obj.other_dims_intervals, overlap_fractions, overlap_abs);
            [obj.lb_sampler, step_sizes] = create_lazy_sampler_for_initial_intervals( ...
                obj.uncorrelated_sampling_dims, obj.sampling_dims_bounds, ...
                obj.range_sizes_of_uncorrelated_sampling_dims, ...
                obj.overlap_fractions, overlap_abs, obj.invert_sampling_order, obj.max_samples_per_dim);
            obj.step_sizes_of_uncorrelated_sampling_dims = step_sizes;
            obj.interval_scaffold = obj.create_interval_scaffold();

            obj.load_dump();
        end

        function max_final_set = get_max_final_set(obj, center_ref)
            final_interval = obj.final_interval_scaffold;
            corr_range_sizes = obj.get_corraleted_range_sizes(center_ref, 1);
            corr_range_sizes = corr_range_sizes{obj.correlated_sampling_dims};
            % FIXME also consider overlap_abs
            overlap_fracs = obj.overlap_fractions(obj.correlated_sampling_dims);
            ranges = repmat(overlap_fracs', 1, 2) .* [-corr_range_sizes / 2 corr_range_sizes / 2];
            final_interval(obj.correlated_sampling_dims, :) = ranges;
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
            next_lb_uncorr = obj.lb_sampler.next();
            while ~isempty(next_lb_uncorr)
                itv = obj.interval_scaffold;
                itv = update_interval(itv, obj.uncorrelated_sampling_dims, ...
                    obj.sampling_dims_bounds, next_lb_uncorr, obj.range_sizes_of_uncorrelated_sampling_dims);

                corr_range_sizes = obj.get_corraleted_range_sizes(mean(itv, 2), 0);
                bounds_for_corr = obj.sampling_dims_bounds;
                if any(obj.correlated_sampling_dims == obj.mirrored_dim)
                    b = bounds_for_corr{obj.mirrored_dim};
                    b(1) = b(1) - corr_range_sizes{obj.mirrored_dim} / 2;
                    bounds_for_corr{obj.mirrored_dim} = b;
                end
                sampler_for_corr = create_lazy_sampler_for_initial_intervals( ...
                    obj.correlated_sampling_dims, bounds_for_corr, ...
                    corr_range_sizes, obj.overlap_fractions, obj.overlap_abs, ...
                    obj.invert_sampling_order, obj.max_samples_per_dim);

                next_lb_corr = sampler_for_corr.next();
                while ~isempty(next_lb_corr)
                    itv = update_interval(itv, obj.correlated_sampling_dims, ...
                        obj.sampling_dims_bounds, next_lb_corr, corr_range_sizes);

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
                    next_lb_corr = sampler_for_corr.next();
                end
                next_lb_uncorr = obj.lb_sampler.next();
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

        function corr_range_sizes = get_corraleted_range_sizes(obj, center_ref, underapproximate)
            corr_range_sizes = dictionary();
            for k = 1:length(obj.correlated_sampling_dims)
                state_idx = obj.correlated_sampling_dims(k);
                corr_dim = obj.sampling_dims_correlations(state_idx);

                get_range_size = obj.get_range_sizes_of_correlated_sampling_dims(state_idx);

                corr_dim_value = center_ref(corr_dim, :);
                range_size = get_range_size(corr_dim_value);

                if underapproximate
                    %% we need to shift the value of the correlated dimension
                    step = obj.step_sizes_of_uncorrelated_sampling_dims(corr_dim);
                    corr_dim_value_ub = corr_dim_value + step;
                    corr_dim_value_lb = corr_dim_value - step;
                    range_size = min([get_range_size(corr_dim_value_lb), ...
                                      range_size, get_range_size(corr_dim_value_ub)]);
                end
                corr_range_sizes{state_idx} = range_size;
            end
        end

    end
end

function itv = update_interval(itv, dims, total_bounds, lb, range_sizes)
    ub = lb + extract_from(range_sizes, 1, dims);
    ub = min(ub, extract_from(total_bounds, 2, dims));
    itv(dims, :) = [lb ub];
end

function res = extract_from(dict, idx, dims)
    values = dict(dims);
    res = cellfun(@(v) v(idx), values);
    res = res';
end

function res = box_arrs2set_cell_arr(boxes, id_manager)
    n = size(boxes, 1);
    res = cell(n, 1);
    for i = 1:n
        b = squeeze(boxes(i, :, :));
        itv = interval(b(:, 1), b(:, 2));
        res{i} = SetContainer(id_manager.gen_next_id(), itv, true);
    end
end
