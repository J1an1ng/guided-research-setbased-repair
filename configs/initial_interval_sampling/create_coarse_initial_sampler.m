function [sampler, v_range_size] = create_coarse_initial_sampler( ...
    result_dir, num_workers, state_order, params, constraints, terminal_cond, initial_sizes)
    arguments
        result_dir ResultDir
        num_workers uint32
        state_order KSTTrailerStateOrder
        params KSTTrailerParams
        constraints KSTTrailerConstraints
        terminal_cond AbstractTerminalCondition
        initial_sizes dictionary
    end

    k = state_order;

    sampling_dims = [k.DELTA k.V k.HA];
    other_dims = setdiff(1:k.get_size(), sampling_dims);

    sampling_bounds = dictionary();
    invert_sampling_order = dictionary();

    range_sizes_of_uncorrelated_sampling_dims = dictionary();

    %% Hint: we can mirror the delta-intervals, i.e., we only have to sample
    %% ranges into one direction.
    sampling_bounds{k.DELTA} = [0 params.steering_max];
    invert_sampling_order{k.DELTA} = 0;

    sampling_bounds{k.V} = [0 params.v_max];
    invert_sampling_order{k.V} = 1; % we start with high velocities
    v_range_size = 3;
    range_sizes_of_uncorrelated_sampling_dims{k.V} = v_range_size;

    sampling_bounds{k.HA} = [-0.005, 0.005]; % [-params.max_hitch_angle, params.max_hitch_angle];
    invert_sampling_order{k.HA} = 0;
    % this value results in an acceptable occupancy and can be controlled to a
    % smaller value in the end
    range_sizes_of_uncorrelated_sampling_dims{k.HA} = 0.01;

    overlap_abs = dictionary(sampling_dims, 2 * initial_sizes(sampling_dims));

    %% Correlations
    sampling_dims_correlations = dictionary();
    sampling_dims_correlations(k.DELTA) = k.V;
    get_range_sizes_of_correlated_sampling_dims = dictionary();
    f = create_get_delta_rad_for_v(constraints, initial_sizes(k.DELTA));
    get_range_sizes_of_correlated_sampling_dims(k.DELTA) = @(v) 2 * f(v);

    %% Initial intervals for transformable state dimensions:
    other_dims_intervals = dictionary();
    for i = other_dims
        other_dims_intervals{i} = [-initial_sizes(i), initial_sizes(i)];
    end

    sampler = BoxLikeStateSpaceDividerWithCorrelations(result_dir, ...
        num_workers, ...
        constraints, terminal_cond, k.get_size(), ...
        sampling_dims, k.DELTA, sampling_bounds, ...
        sampling_dims_correlations, get_range_sizes_of_correlated_sampling_dims, ...
        range_sizes_of_uncorrelated_sampling_dims,  ...
        invert_sampling_order, ...
        other_dims_intervals, dictionary(uint32.empty, double.empty), overlap_abs);
end
