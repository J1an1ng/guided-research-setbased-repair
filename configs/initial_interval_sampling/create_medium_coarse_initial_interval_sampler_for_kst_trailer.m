function sampler = create_medium_coarse_initial_interval_sampler_for_kst_trailer( ...
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
    sampling_range_sizes = dictionary();
    invert_sampling_order = dictionary();
    sampling_dims_min_overlap = dictionary(uint32.empty, double.empty);

    delta_range_size = 0.1;
    sampling_bounds{k.DELTA} = [-delta_range_size / 2, params.steering_max];
    sampling_range_sizes{k.DELTA} = delta_range_size;
    invert_sampling_order{k.DELTA} = 0;

    sampling_bounds{k.V} = [0 params.v_max];
    sampling_range_sizes{k.V} = 1;
    invert_sampling_order{k.V} = 1; % we start with high velocities

    sampling_bounds{k.HA} = [-params.max_hitch_angle, params.max_hitch_angle];
    sampling_range_sizes{k.HA} = 0.1;
    invert_sampling_order{k.HA} = 0;

    %% Initial intervals for transformable state dimensions:
    other_dims_intervals = dictionary();

    other_dims_intervals{k.X} = [-0.15, 0.15];
    other_dims_intervals{k.Y} = [-0.05, 0.05];
    other_dims_intervals{k.PSI} = [-0.008, 0.008];

    overlap_fraction = 0;

    sampler = StateIntervalSampler(constraints, k.SIZE, ...
        sampling_state_dims, sampling_bounds, ...
        sampling_range_sizes, invert_sampling_order, ...
        overlap_fraction, other_dims_intervals, sampling_dims_min_overlap);
end
