function sampler = create_initial_sampler_probme_int( ...
    state_order, params, constraints)
    arguments
        state_order AbstractKSTTrailerStateOrder
        params AbstractKSTTrailerParams
        constraints KSTTrailerConstraints
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
    sampling_bounds{k.DELTA} = [0.2484 0.4967]; % [0 params.steering_max];
    overlap_fractions(k.DELTA) = 0.5;
    invert_sampling_order{k.DELTA} = 0;

    sampling_bounds{k.V} = [0 3]; % [0 params.v_max];
    overlap_fractions(k.V) = 0.5;
    invert_sampling_order{k.V} = 1; % we start with high velocities
    range_sizes_of_uncorrelated_sampling_dims{k.V} = 1.5;

    sampling_bounds{k.HA} = [-0.3312 -0.3152]; % [-params.max_hitch_angle, params.max_hitch_angle];
    overlap_fractions(k.HA) = 0.9;
    invert_sampling_order{k.HA} = 0;
    range_sizes_of_uncorrelated_sampling_dims{k.HA} = 0.016; % 0.02;

    %% Correlations
    sampling_dims_correlations = dictionary();
    sampling_dims_correlations(k.DELTA) = k.V;
    range_sizes_of_correlated_sampling_dims = dictionary();
    delta_total_range = params.steering_max -  params.steering_min;
    range_sizes_of_correlated_sampling_dims{k.DELTA} = [delta_total_range / 4 0.2395];
    % [delta_total_range / 4 0.012];
    %% range_sizes_of_correlated_sampling_dims{k.DELTA} = [0.012 0.012];
    %% range_sizes_of_correlated_sampling_dims{k.DELTA} = [delta_total_range / 4 0.216];
    %% range_sizes_of_correlated_sampling_dims{k.DELTA} = [delta_total_range / 4 0.157];

    %% Initial intervals for transformable state dimensions:
    other_dims_intervals = dictionary();

    other_dims_intervals{k.X} = [-0.15, 0.15];
    other_dims_intervals{k.Y} = [-0.1, 0.1];
    other_dims_intervals{k.PSI} = [-0.008, 0.008];

    sampler = BoxLikeStateSpaceDividerWithCorrelations(constraints, k.SIZE, ...
        sampling_state_dims, k.DELTA, sampling_bounds, ...
        sampling_dims_correlations, range_sizes_of_correlated_sampling_dims, ...
        range_sizes_of_uncorrelated_sampling_dims,  ...
        invert_sampling_order, ...
        overlap_fractions, other_dims_intervals);
end
