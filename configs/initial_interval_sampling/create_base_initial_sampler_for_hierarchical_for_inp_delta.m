function [sampler, v_range_size] = create_base_initial_sampler_for_hierarchical_for_inp_delta( ...
    result_dir, num_workers, state_order, params, constraints, terminal_cond, initial_rads)
    arguments
        result_dir ResultDir
        num_workers uint32
        state_order KSTTrailerInpDeltaStateOrder
        params KSTTrailerParams
        constraints KSTTrailerInpDeltaConstraints
        terminal_cond AbstractTerminalCondition
        initial_rads dictionary
    end

    k = state_order;

    sampling_dims = [k.V k.HA];
    other_dims = setdiff(1:k.get_size(), sampling_dims);

    sampling_bounds = dictionary();
    invert_sampling_order = dictionary();

    range_sizes = dictionary();

    sampling_bounds{k.V} = [0 params.v_max];
    invert_sampling_order{k.V} = 1; % we start with high velocities
    v_range_size = 0.4; % 0.37;
    range_sizes{k.V} = v_range_size;

    sampling_bounds{k.HA} = [0, params.max_hitch_angle];
    invert_sampling_order{k.HA} = 0;
    range_sizes{k.HA} = 0.053; % 0.039;

    %% FIXME
    overlap_abs = dictionary(sampling_dims, 2 * initial_rads(sampling_dims));

    %% Initial intervals for transformable state dimensions:
    other_dims_intervals = dictionary();
    for i = other_dims
        other_dims_intervals{i} = [-initial_rads(i), initial_rads(i)];
    end

    sampler = BoxLikeStateSpaceDivider(result_dir, ...
        num_workers, ...
        constraints, terminal_cond, k.get_size(), ...
        sampling_dims, k.HA, sampling_bounds, ...
        range_sizes,  ...
        invert_sampling_order, ...
        other_dims_intervals, dictionary(uint32.empty, double.empty), overlap_abs);
end
