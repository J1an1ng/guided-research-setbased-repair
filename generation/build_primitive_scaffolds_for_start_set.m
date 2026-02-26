function primitives = build_primitive_scaffolds_for_start_set(start_set, batch_out_path, settings, traj_gen)
    log_file_path = strcat(batch_out_path, "/", string(start_set.id), ".txt");

    log_fid = fopen(log_file_path, 'wt');
    [trajectories, discarded_trajs] = traj_gen.generate(center(start_set.cora_set));

    my_log(log_fid, settings.debug, 'Building primitive scaffolds for start set: %d. ', start_set.id);

    my_log(log_fid, settings.debug, ['#Valid trajs: %d. ' ...
                                     '#Discarded trajs: %d.\n'], ...
                     length(trajectories), length(discarded_trajs));

    counter = 0;

    primitives = cell(length(trajectories), 1);

    for i = 1:length(trajectories)
        traj = trajectories{i};

        counter = counter + 1;
        my_log(log_fid, settings.debug, 'Counter: %d. ', counter);
        my_log(log_fid, settings.debug, 'Ref traj: %s \n', traj.json);

        p = Primitive(start_set, traj, settings.dt_control, settings.occ_dt);

        primitives{i} = p;
    end
end
