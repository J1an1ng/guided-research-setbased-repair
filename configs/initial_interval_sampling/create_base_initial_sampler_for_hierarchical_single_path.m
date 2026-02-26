function sampler = create_base_initial_sampler_for_hierarchical_single_path( ...
    result_dir, num_workers, state_order, params, constraints, terminal_cond, state_meas_dist)
    arguments
        result_dir ResultDir
        num_workers uint32
        state_order KSTTrailerStateOrder
        params KSTTrailerParams
        constraints KSTTrailerConstraints
        terminal_cond AbstractTerminalCondition
        state_meas_dist (:, 1) double
    end

    k = state_order;

    %% Sampling ranges
    % Indices of state components, that cannot be set to 0 for the initial
    % set of a motion primitive
    sampling_state_dims = [k.DELTA k.V k.HA];

    sampling_bounds = dictionary();
    invert_sampling_order = dictionary();

    range_sizes_of_uncorrelated_sampling_dims = dictionary();

    %% Hint: we can mirror the delta-intervals, i.e., we only have to sample
    %% ranges into one direction.
    get_delta_rad_for_v = create_get_delta_rad_for_v(constraints, state_meas_dist(k.DELTA));
    sampling_bounds{k.DELTA} = [0 2 * get_delta_rad_for_v(params.v_max)];
    invert_sampling_order{k.DELTA} = 0;

    sampling_bounds{k.V} = [params.v_max - 1.4, params.v_max];
    invert_sampling_order{k.V} = 1; % we start with high velocities
    range_sizes_of_uncorrelated_sampling_dims{k.V} = 1.4;

    ha_rad = 0.0359 / 2.0;
    sampling_bounds{k.HA} = [-ha_rad, ha_rad];
    invert_sampling_order{k.HA} = 0;
    range_sizes_of_uncorrelated_sampling_dims{k.HA} = 2 * ha_rad;

    overlap_abs = dictionary(sampling_state_dims, state_meas_dist(sampling_state_dims)');

    %% Correlations
    sampling_dims_correlations = dictionary();
    sampling_dims_correlations(k.DELTA) = k.V;
    get_range_sizes_of_correlated_sampling_dims = dictionary();
    delta_range = 2 * get_delta_rad_for_v(params.v_max);
    get_range_sizes_of_correlated_sampling_dims(k.DELTA) = @(v) delta_range;

    %% Initial intervals for transformable state dimensions:
    other_dims_intervals = get_kst_initial_itv_size(k, state_meas_dist);

    sampler = BoxLikeStateSpaceDividerWithCorrelations(result_dir, ...
        num_workers, ...
        constraints, terminal_cond, k.get_size(), ...
        sampling_state_dims, k.DELTA, sampling_bounds, ...
        sampling_dims_correlations, get_range_sizes_of_correlated_sampling_dims, ...
        range_sizes_of_uncorrelated_sampling_dims,  ...
        invert_sampling_order, ...
        other_dims_intervals, dictionary(uint32.empty, double.empty), overlap_abs);
end
