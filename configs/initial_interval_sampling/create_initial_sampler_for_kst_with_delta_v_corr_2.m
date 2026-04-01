function sampler = create_initial_sampler_for_kst_with_delta_v_corr_2( ...
    result_dir, num_workers, state_order, params, constraints, terminal_cond, state_meas_dist)
    arguments
        result_dir ResultDir
        num_workers uint32
        state_order AbstractKSTTrailerStateOrder
        params AbstractKSTTrailerParams
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
    sampling_bounds{k.DELTA} = [0 params.steering_max];
    invert_sampling_order{k.DELTA} = 0;

    sampling_bounds{k.V} = [0 params.v_max];
    invert_sampling_order{k.V} = 1; % we start with high velocities
    range_sizes_of_uncorrelated_sampling_dims{k.V} = 1.;

    sampling_bounds{k.HA} = [-params.max_hitch_angle, params.max_hitch_angle];
    invert_sampling_order{k.HA} = 0;
    range_sizes_of_uncorrelated_sampling_dims{k.HA} = 0.003; % 0.02;

    overlap_abs = dictionary(sampling_state_dims, state_meas_dist(sampling_state_dims)');

    %% Correlations
    sampling_dims_correlations = dictionary();
    sampling_dims_correlations(k.DELTA) = k.V;
    get_range_sizes_of_correlated_sampling_dims = dictionary();
    get_range_sizes_of_correlated_sampling_dims(k.DELTA) = @(v) max( ...
        2.0 * 0.1 * constraints.delta_max_for_v(v), 1.2 * state_meas_dist(k.DELTA));

    %% Initial intervals for transformable state dimensions:
    other_dims_intervals = dictionary();

    other_dims_intervals{k.X} = [-state_meas_dist(k.X), state_meas_dist(k.X)];
    other_dims_intervals{k.Y} = [-2 * state_meas_dist(k.Y), 2 * state_meas_dist(k.Y)];
    other_dims_intervals{k.PSI} = [-state_meas_dist(k.PSI), state_meas_dist(k.PSI)];

    sampler = BoxLikeStateSpaceDividerWithCorrelations(result_dir, ...
        num_workers, constraints, terminal_cond, k.SIZE, ...
        sampling_state_dims, k.DELTA, sampling_bounds, ...
        sampling_dims_correlations, get_range_sizes_of_correlated_sampling_dims, ...
        range_sizes_of_uncorrelated_sampling_dims,  ...
        invert_sampling_order, ...
        other_dims_intervals, dictionary(uint32.empty, double.empty), overlap_abs);
end
