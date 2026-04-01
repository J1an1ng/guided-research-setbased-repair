function primitives = create_primitive_scaffolds( ...
    inp_traj_gen, ...
    start_itv, settings, ...
    model_for_ref_traj)

    arguments
        inp_traj_gen AbstractInpTrajGenerator
        start_itv
        settings
        model_for_ref_traj
    end

    traj_gen = ValidInpTrajGenerator(inp_traj_gen, settings.terminal_cond, ...
        settings.primitive_duration, ...
        settings.dt_control, ...
        settings.time_decimal_places, ...
        model_for_ref_traj, settings.k, settings.i, settings.params, ...
        settings.constraints, ...
        settings.extended_horizon, settings.process_final_state);

    fprintf('Interval: %s\n', clean_interval_json_string(jsonencode(start_itv)));

    % plot_trajectories(start_itv, plotter, traj_gen, settings.k, false);

    [trajectories, discarded_trajs] = traj_gen.generate(center(start_itv));
    % assert(length(trajectories) == exp_num_primitives, 'Expected number of primitives failed');
    fprintf(['#Valid trajs: %d.\n ' ...
             '#Discarded trajs: %d.\n'], ...
            length(trajectories), length(discarded_trajs));

    primitives = cell(length(trajectories), 1);

    for t = 1:length(trajectories)
        next_id = settings.state_space_divider.set_id_manager.gen_next_id();
        start_itv_container = SetContainer(next_id, start_itv, false);
        p = Primitive(start_itv_container, trajectories{t}, settings.dt_control, settings.occ_dt);
        primitives{t} = p;
    end
    primitives = [primitives{:}]';
end
