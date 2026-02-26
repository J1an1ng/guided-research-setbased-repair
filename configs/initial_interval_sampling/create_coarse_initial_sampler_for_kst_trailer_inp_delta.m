function [sampler, v_range_size] = create_coarse_initial_sampler_for_kst_trailer_inp_delta( ...
    result_dir, num_workers, state_order, params, constraints, terminal_cond, state_meas_dist)
    arguments
        result_dir ResultDir
        num_workers uint32
        state_order KSTTrailerInpDeltaStateOrder
        params KSTTrailerParams
        constraints KSTTrailerInpDeltaConstraints
        terminal_cond AbstractTerminalCondition
        state_meas_dist (:, 1) double
    end

    k = state_order;

    %% Sampling ranges
    % Indices of state components, that cannot be set to 0 for the initial
    % set of a motion primitive
    sampling_state_dims = [k.V k.HA];

    sampling_bounds = dictionary();
    invert_sampling_order = dictionary();

    range_sizes = dictionary();

    sampling_bounds{k.V} = [0 params.v_max];
    invert_sampling_order{k.V} = 1; % we start with high velocities
    v_range_size = 10.0;
    range_sizes{k.V} = v_range_size;

    sampling_bounds{k.HA} = [-params.max_hitch_angle, params.max_hitch_angle];
    invert_sampling_order{k.HA} = 0;
    range_sizes{k.HA} = 0.25;

    % the overlap needs to be twice the state_meas_dist, such that during online
    % planning, each possible state interval is enclosed in one of the sampled boxes.
    overlap_abs = dictionary(sampling_state_dims, 2 * state_meas_dist(sampling_state_dims)');

    %% Initial intervals for transformable state dimensions:
    other_dims_intervals = get_kst_initial_itv_size(k, state_meas_dist);

    sampler = BoxLikeStateSpaceDivider(result_dir, ...
        num_workers, ...
        constraints, terminal_cond, k.get_size(), ...
        sampling_state_dims, NaN, sampling_bounds, ...
        range_sizes,  ...
        invert_sampling_order, ...
        other_dims_intervals, dictionary(uint32.empty, double.empty), overlap_abs);
end
