function primitives = sample_primitives_for_controller_opt(settings, model, ...
    model_for_ref_traj, bounds, desired_duration, accs, exp_num_primitives, plotter)
    arguments
        settings
        model
        model_for_ref_traj
        bounds Bounds
        desired_duration double
        accs (1, :) double
        exp_num_primitives uint32
        plotter
    end

    k = settings.k;
    params = settings.params;

    sampling_state_dims = [k.DELTA k.V k.HA];

    v_size = 1.0;

    ha_size = 0.003;

    get_delta_range_size_for_v = @(v) max( ...
        2.0 * 0.1 * settings.constraints.delta_max_for_v(v), ...
        1.2 * settings.state_meas_dist(k.DELTA));

    % We define the intervals to sample from for the initial sampling
    itv_scaffold = zeros(k.get_size(), 2);
    itv_scaffold([k.X k.Y k.PSI], :) = ...
            [-settings.state_meas_dist(k.X), settings.state_meas_dist(k.X); ...
             -settings.state_meas_dist(k.Y), settings.state_meas_dist(k.Y); ...
             -settings.state_meas_dist(k.PSI), settings.state_meas_dist(k.PSI)];

    function p_list = sample_for_v_max(v_max, delta_scale, ~, target_point_dist) % delta_scale ha_scale
        v = [v_max - v_size, v_max];
        delta_size = get_delta_range_size_for_v(v_max);
        normal_delta = [-delta_size / 2 delta_size / 2];
        normal_ha = [-ha_size / 2, ha_size / 2];
        % ha_scaled_start = ha_scale * ha_size;
        % scaled_ha = [ha_scaled_start, ha_scaled_start + ha_size / 2];
        delta_scaled_start = delta_scale * delta_size;
        scaled_delta = [delta_scaled_start delta_scaled_start + delta_size / 2];

        configs = [normal_delta normal_ha
                   % normal_delta scaled_ha; ...
                   scaled_delta normal_ha
                   % scaled_delta scaled_ha; ...
                   % scaled_delta flip(-scaled_ha)];
                  ];

        p_list = ListLeaf();
        for j = 1:size(configs, 1)
            config = configs(j, :);
            next_itv = itv_scaffold;
            next_itv(sampling_state_dims, :) = [config([1 2]); v; config([3 4])];
            if ~settings.constraints.check_interval(next_itv) ...
                    || settings.terminal_cond.check_for_itv_soft(next_itv)
                error('Invalid interval');
            end

            new_p_list = sample_primitives_for_itv(next_itv, settings, model, ...
                model_for_ref_traj, bounds, desired_duration, accs, ...
                target_point_dist, exp_num_primitives, plotter);
            p_list = ListNode(new_p_list, p_list);
        end
    end

    v_max_list = [params.v_max, 15.0, 10.0, 5.0, 4.0, 2.5];
    delta_scale_list = [2 2 2 2 2 2];
    ha_scale_list = [1, 5, 9, 9, 9, 9];

    v_max2target_point_dist = dictionary(v_max_list, [1.0, 2.0, 3.0, 3.0, 3.0, 3.0]);

    p_list_outer = ListLeaf();
    for i = 1:length(v_max_list)
        v_max = v_max_list(i);
        target_point_dist = v_max2target_point_dist(v_max);
        new_p_list_outer = sample_for_v_max(v_max, delta_scale_list(i), ...
            ha_scale_list(i), target_point_dist);
        p_list_outer = ListNode(new_p_list_outer, p_list_outer);
    end
    primitives = p_list_outer.recursive_reduce(@(acc, el) [acc; el], []);
end

function p_list = sample_primitives_for_itv(itv, settings, model, ...
    model_for_ref_traj, bounds, desired_duration, accs, ...
    target_point_dist, ~, ~) % exp_num_primitives, plotter)
    inp_traj_gen = KSTInpTrajTargetPointGenerator(settings.k, settings.i, ...
        settings.params, model, bounds, ...
        desired_duration, accs, target_point_dist, 0.01);
    traj_gen = ValidInpTrajGenerator(inp_traj_gen, settings.terminal_cond, ...
        settings.primitive_duration, ...
        settings.dt_control, ...
        settings.time_decimal_places, ...
        model_for_ref_traj, settings.k, settings.i, settings.params, ...
        settings.constraints, ...
        settings.extended_horizon, settings.process_final_state);

    itv = interval(itv(:, 1), itv(:, 2));
    fprintf('Interval: %s\n', clean_interval_json_string(jsonencode(itv)));

    % plot_trajectories(itv, plotter, traj_gen, settings.k, false);

    [trajectories, discarded_trajs] = traj_gen.generate(center(itv));
    % assert(length(trajectories) == exp_num_primitives, 'Expected number of primitives failed');
    fprintf(['#Valid trajs: %d.\n ' ...
             '#Discarded trajs: %d.\n'], ...
            length(trajectories), length(discarded_trajs));
    p_list = ListLeaf();
    for t = 1:length(trajectories)
        next_id = settings.state_space_divider.set_id_manager.gen_next_id();
        itv_container = SetContainer(next_id, itv, true);
        p = Primitive(itv_container, trajectories{t}, settings.dt_control, settings.occ_dt);
        p_list = ListNode(p, p_list);
    end
end
