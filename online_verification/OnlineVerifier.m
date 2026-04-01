classdef OnlineVerifier < handle
    properties
        dyn
        k AbstractStateOrder
        r_init_computer AbstractRInitComputer
        state_meas_dist
        cora_options
        state_proc
        dyn_conf
        dyn_conf_values
        simulator
        occ_comp OccupancyComputer
        norm_final_set
        mirroring
        dt_control double
        occ_dt double
        plot_result logical
    end

    methods

        function obj = OnlineVerifier(dyn, k, ...
                r_init_computer, ...
                state_meas_dist, ...
                cora_options, ...
                state_proc, dyn_conf, dyn_conf_values, ....
                simulator, occ_comp, norm_final_set, mirroring, ...
                dt_control, occ_dt, plot_result)
            obj.dyn = dyn;
            obj.k = k;
            obj.r_init_computer = r_init_computer;
            obj.state_meas_dist = state_meas_dist;
            obj.cora_options = cora_options;
            obj.state_proc = state_proc;
            obj.dyn_conf = dyn_conf;
            obj.dyn_conf_values = dyn_conf_values;
            obj.simulator = simulator;
            obj.occ_comp = occ_comp;
            obj.norm_final_set = norm_final_set;
            obj.mirroring = mirroring;
            obj.dt_control = dt_control;
            obj.occ_dt = occ_dt;
            obj.plot_result = plot_result;
        end

        function [R_final, occs] = compute_reachset(obj, y_0, u_ref, time_per_input)
            arguments
                obj OnlineVerifier
                y_0 (:, 1) double
                u_ref (:, :) double
                time_per_input double
            end

            R_result = [];

            num_inputs = size(u_ref, 2);
            duration = num_inputs * time_per_input;

            R_init = obj.r_init_computer.compute_initial_set(y_0, obj.state_meas_dist);
            x_0 = center(R_init);

            ref_traj = PredefinedInputTraj(u_ref, time_per_input);
            success = obj.simulator.simulate(ref_traj, x_0, duration, obj.dt_control);

            if ~success
                error("Simulation failed");
            end

            R_0 = obj.state_proc.process_initial_set(R_init);

            num_steps = size(ref_traj.inputs, 2);
            t = 0;
            i = 1;
            while i <= num_steps
                params.tStart = t;
                params.tFinal = t + obj.dt_control;

                [params.u, params.U] = obj.dyn_conf.create_params(ref_traj, i, obj.dyn_conf_values);

                params.R0 = obj.state_proc.process_initial_set_before_reach(R_0);

                R_current = reach(obj.dyn, params, obj.cora_options);
                R_final = R_current(end).timePoint.set{end};
                R_0 = obj.state_proc.process_final_set_after_reach(R_final);
                R_current_proj = obj.state_proc.project_final_set(R_current);

                R_result = add(R_result, R_current_proj);

                t = t + obj.dt_control;
                i = i + 1;

            end

            %% TODO for usage in Python-code: wrap this into a class that transforms the reachsets to occupancies and
            %% returns only the last reachset (normalized!) and the occupancies

            if obj.plot_result
                plot_reachsets(ref_traj, R_result, obj.k, 3, []);
            end

            R_final = R_result(end).timePoint.set{end};
            R_final = obj.norm_final_set(R_final, obj.k);
            R_final = reduce(R_final, 'pca', 100);
            [R_final, was_mirrored] = obj.mirroring.mirror(R_final);

            fprintf('Final set mirrored: %d\n', was_mirrored);

            num_occs_to_merge = obj.occ_dt / obj.dt_control;
            occs = reachsets_to_occupancies(R_result, obj.occ_comp, num_occs_to_merge);
        end

    end
end
