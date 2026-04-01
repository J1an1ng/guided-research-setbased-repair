function p_fragment = perform_reach_for_primitive(p, ...
        settings, reach_performer, existing_primitives, q, sim, sim_sampler, num_sims)
    deactivate_cora_warning();

    log_file_path = strcat(settings.result_dir.reach_log_file_path, "/", p.json, ".txt");
    log_fid = fopen(log_file_path, 'wt');

    my_log(log_fid, settings.debug, 'Start reach for: %s. \n', p.json);

    dyn = settings.create_dynamics();

    saver = SinglePrimitiveIO(settings.result_dir, settings.occ_comp, settings.max_half_width);

    primitive_exists = isKey(existing_primitives, p.json);
    if primitive_exists
        my_log(log_fid, settings.debug, 'Primitive already exists. Skip. \n');
        final_set_id_from_last_run = existing_primitives(p.json);
        assert(final_set_id_from_last_run == p.final_set_id, ...
            'Final set id from last run is different from the one in the primitive. Inconsistency detected.');
        if saver.final_set_exists(p)
            saver.load_final_set(p);
        else
            my_log(log_fid, true, 'Primitive generated but final set missing.');
            primitive_exists = false;
        end
    end

    if ~primitive_exists
        res = reach_performer.generate(settings, p, dyn, log_fid);
        p.apply_reach_result(res, settings.norm_final_set, settings.mirroring, settings.k);

        if settings.create_plot
            sim_res = [];
            if settings.do_random_simulations
                initial_itv_rads = rad(p.start_set.enclosing_itv);
                sim_res = sim.simulate(num_sims, p.ref_traj, initial_itv_rads, settings.dt_control, [], sim_sampler);
            end
            plot_primitive(settings, p, p.reachsets, sim_res);
        end

        saver.save(p);
        p_for_queue = p.to_fragment_for_logging();
        send(q, p_for_queue);
    end

    fclose(log_fid);

    % Hint: we only return the fragment to save memory
    p_fragment = p.to_fragment_for_processing();
    clear p;
    clear dyn;
    clear saver;
    clear res;
end
