classdef RandomCoraSimulator < handle & matlab.mixin.Heterogeneous
    properties (SetAccess = private, GetAccess = public)
        dyn
        param_conf ControlledModelParamConfig
        dyn_conf_values ControlledModelParamValues
        state_proc StateProcessor
    end

    methods

        function obj = RandomCoraSimulator(dyn, param_conf, dyn_conf_values, state_proc)
            obj.dyn = dyn;
            obj.param_conf = param_conf;
            obj.dyn_conf_values = dyn_conf_values;
            obj.state_proc = state_proc;
        end

        function sim_res = simulate(obj, num_sims, ref_traj, initial_itv_rads, dt_control, errors_to_catch, sim_sampler, new_dyn_conf_values)
            if ~exist('new_dyn_conf_values', 'var')
                new_dyn_conf_values = obj.dyn_conf_values;
            end

            initial_ref_state = ref_traj.states(:, 1);
            initial_set = zonotope(interval(initial_ref_state - initial_itv_rads, initial_ref_state + initial_itv_rads));

            sim_res = cell(num_sims, 1);

            for k = 1:num_sims
                x0 = sim_sampler.sample_initial_state(initial_set);
                sim_res{k} = obj.simulate_single_trace(x0, ref_traj, dt_control, errors_to_catch, new_dyn_conf_values, sim_sampler);
            end
            sim_res = [sim_res{:}]';
        end

        function sim_res = simulate_single_trace(obj, x0, ref_traj, dt_control, errors_to_catch, new_dyn_conf_values, sim_sampler)
            max_steps = size(ref_traj.inputs, 2);
            x_res = cell(max_steps + 1, 1);
            t_res = cell(max_steps + 1, 1);
            x_res{1} = x0';
            t_res{1} = [0];
            t = 0;
            for i = 1:max_steps
                [x_res{i + 1}, t_res{i + 1}, err] = obj.simulate_single_step(x_res{i}', i, t, dt_control, ref_traj, ...
                    errors_to_catch, new_dyn_conf_values, sim_sampler);
                if ~isempty(err)
                    break
                end
                t = t + dt_control;
            end
            % Convert cell arrays to matrices for simResult
            x_mat = cell2mat(x_res);
            t_vec = cell2mat(t_res);
            sim_res = simResult({x_mat}, {t_vec});
        end

        function [x_next, t_next, err] = simulate_single_step(obj, x, i, t_start, dt_control, ref_traj, errors_to_catch, ...
                new_dyn_conf_values, sim_sampler)
            err = [];
            params.tStart = t_start;
            params.tFinal = t_start + dt_control;
            max_steps = size(ref_traj.inputs, 2);

            ref_state = ref_traj.states(:, i);
            meas_dist = sim_sampler.sample_meas_dist(x, ref_state);

            params.x0 = obj.state_proc.process_state_before_sim(x, meas_dist);

            if obj.param_conf.include_model_add_dist
                model_add_dist = sim_sampler.sample_model_add_dist(x, ref_state);
            else
                model_add_dist = [];
            end

            if obj.param_conf.include_model_inp_dist
                inp_dist = sim_sampler.sample_inp_dist(x, ref_state);
            else
                inp_dist = [];
            end

            params.u = obj.param_conf.create_params_for_sim(ref_traj, min(i, max_steps), new_dyn_conf_values, meas_dist, model_add_dist, inp_dist);

            try
                [t_res, x_res] = simulate(obj.dyn, params);
                x_res = obj.state_proc.process_state_after_sim(x_res(end, :));
                t_res = t_res(end);
                x_next = x_res';
                t_next = t_res;
            catch ME
                % only catch errors that are in the list of errors to catch
                if any(strcmp(ME.identifier, errors_to_catch))
                    x_next = [];
                    t_next = [];
                    err = ME;
                else
                    rethrow(ME);
                end
            end
        end

    end
end
