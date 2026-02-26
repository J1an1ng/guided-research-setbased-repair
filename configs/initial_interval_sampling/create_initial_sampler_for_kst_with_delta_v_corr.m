function sampler = create_initial_sampler_for_kst_with_delta_v_corr( ...
    out_path, num_workers, state_order, params, constraints, terminal_cond)
    arguments
        out_path string
        num_workers uint32
        state_order AbstractKSTTrailerStateOrder
        params AbstractKSTTrailerParams
        constraints KSTTrailerConstraints
        terminal_cond AbstractTerminalCondition
    end

    k = state_order;

    %% Sampling ranges
    % Indices of state components, that cannot be set to 0 for the initial
    % set of a motion primitive
    sampling_state_dims = [k.DELTA k.V k.HA];

    sampling_bounds = dictionary();
    overlap_fractions = dictionary();
    invert_sampling_order = dictionary();

    range_sizes_of_uncorrelated_sampling_dims = dictionary();

    %% Hint: we can mirror the delta-intervals, i.e., we only have to sample
    %% ranges into one direction.
    sampling_bounds{k.DELTA} = [0 params.steering_max];
    overlap_fractions(k.DELTA) = 0.5;
    invert_sampling_order{k.DELTA} = 0;

    sampling_bounds{k.V} = [0 params.v_max];
    overlap_fractions(k.V) = 0.5;
    invert_sampling_order{k.V} = 1; % we start with high velocities
    range_sizes_of_uncorrelated_sampling_dims{k.V} = 1.5;

    sampling_bounds{k.HA} = [-params.max_hitch_angle, params.max_hitch_angle];
    overlap_fractions(k.HA) = 0.9;
    invert_sampling_order{k.HA} = 0;
    range_sizes_of_uncorrelated_sampling_dims{k.HA} = 0.016; % 0.02;

    %% Correlations
    sampling_dims_correlations = dictionary();
    sampling_dims_correlations(k.DELTA) = k.V;
    get_range_sizes_of_correlated_sampling_dims = dictionary();
    get_range_sizes_of_correlated_sampling_dims(k.DELTA) = @(v) 2.0 * 0.3 * constraints.delta_max_for_v(v);

    %% Initial intervals for transformable state dimensions:
    other_dims_intervals = dictionary();

    other_dims_intervals{k.X} = [-0.15, 0.15];
    other_dims_intervals{k.Y} = [-0.1, 0.1];
    other_dims_intervals{k.PSI} = [-0.008, 0.008];

    sampler = BoxLikeStateSpaceDividerWithCorrelations(out_path, num_workers, ...
        constraints, terminal_cond, k.SIZE, ...
        sampling_state_dims, k.DELTA, sampling_bounds, ...
        sampling_dims_correlations, get_range_sizes_of_correlated_sampling_dims, ...
        range_sizes_of_uncorrelated_sampling_dims,  ...
        invert_sampling_order, ...
        other_dims_intervals, overlap_fractions, dictionary(uint32.empty, double.empty));
end
