classdef (Abstract) WorstCaseSimulator < RandomSimulator
    properties (Access = private)
        num_sims uint32
        dyn
        param_conf ControlledModelParamConfig
        dyn_conf_values ControlledModelParamValues
    end

    properties (Access = protected)
        meas_dist (:, 1) double
        meas_dist_itv
        model_add_dist (:, 1) double
        model_add_dist_itv
        inp_dist (:, 1) double
        inp_dist_itv
    end

    methods

        function obj = WorstCaseSimulator(num_sims, dyn, param_conf, dyn_conf_values, meas_dist, model_add_dist, inp_dist)
            obj.num_sims = num_sims;
            obj.dyn = dyn;
            obj.param_conf = param_conf;
            obj.dyn_conf_values = dyn_conf_values;
            obj.meas_dist = meas_dist;
            obj.meas_dist_itv = interval(zeros(size(meas_dist)), meas_dist);
            obj.model_add_dist = model_add_dist;
            obj.model_add_dist_itv = interval(zeros(size(model_add_dist)), model_add_dist);
            obj.inp_dist = inp_dist;
            obj.inp_dist_itv = interval(zeros(size(inp_dist)), inp_dist);
        end

        function res = simulate(obj, initial_set, ref_traj, t_start, t_end)

            assert(isa(initial_set, 'interval'));
            
            res = simResult([], []);
            
            %% TODO make initial states more extreme
            X_0 = randPoint(initial_set, obj.num_sims, 'extreme');
            
            simParams.tStart = t_start;
            simParams.tFinal = t_end;
            for k = 1:obj.num_sims
                x0 = X_0(:, k);
                x0 = obj.get_initial_state(x0, initial_set);
                simParams.x0 = x0;
                initial_state_center = center(initial_set);

                if obj.param_conf.include_meas_dist
                    current_meas_dist = obj.get_meas_dist(x0, initial_state_center);
                else
                    current_meas_dist = [];
                end
                
                %% FIXME also include state_proc here
                %% important: when calling the process_state method: 
                %% only add meas_dist if not included in system!!
                % state = state_proc.process_state(x0, current_meas_dist);

                if obj.param_conf.include_model_add_dist
                    current_model_add_dist = obj.get_model_add_dist(x0, initial_state_center);
                else
                    current_model_add_dist = [];
                end

                if obj.param_conf.include_model_inp_dist
                    current_inp_dist = obj.get_inp_dist(x0, ref_state);
                else
                    current_inp_dist = [];
                end

                max_steps = size(ref_traj.inputs, 2);

                simParams.u = obj.param_conf.create_params_for_sim(ref_traj, 1:max_steps, ...
                    obj.dyn_conf_values, current_meas_dist, current_model_add_dist, current_inp_dist);
                [t_res, x_res] = simulate(obj.dyn, simParams);

                res(k) = simResult({x_res}, {t_res});
            end
        end

        function sim_res = simulate_step(obj, sim_res, i, t_start, t_end, initial_set, ref_traj, state_proc)
            %% TODO es kommt sim_res rein und neues sim_res raus; anfangs ist sim_res leer, dann
            %% werden Anfangszustände gesamplet
            assert(isempty(sim_res) == (i == 1));

            if i == 1
                % we only support interval initial sets
                assert(isa(initial_set, 'interval'));
                x_0 = randPoint(initial_set, obj.num_sims, 'extreme');

                %% TODO create specific methods for simulators; in y-wc: take
                %% extreme value in y and psi
                %% TODO process initial states; add methods to state processor for this
            end

            simParams.tStart = t_start;
            simParams.tFinal = t_end;
            for k = 1:obj.num_sims
                if i == 1
                    x0 = x_0(k, :);
                else
                    r = sim_res(k);
                    x0 = r.x{i - 1};
                end
                ref_state = ref_traj.states(:, i);
                current_meas_dist = obj.get_meas_dist(x0(end, :), ref_state);
                state = state_proc.process_state(x0, current_meas_dist);
                simParams.x0 = state;
                if obj.param_conf.include_model_add_dist
                    current_model_add_dist = obj.get_model_add_dist(state, ref_state);
                else
                    current_model_add_dist = [];
                end

                if obj.param_conf.include_model_inp_dist
                    current_inp_dist = obj.get_inp_dist(state, ref_state);
                else
                    current_inp_dist = [];
                end

                max_steps = size(ref_traj.times, 2);

                simParams.u = obj.param_conf.create_params_for_sim(ref_traj, min(i, max_steps), ...
                    obj.dyn_conf_values, current_meas_dist, current_model_add_dist, current_inp_dist);
                [t_res, x_res] = simulate(obj.dyn, simParams);
                
                if i == 1
                    sim_res(k) = simResult({x_res}, {t_res});
                else
                    sim_res(k) = add(sim_res(k), simResult({x_res}, {t_res}));
                end
            end
        end

    end

    methods (Abstract, Access = protected)
        x = get_initial_state(obj, x, initial_set)
        current_meas_dist = get_meas_dist(obj, state, ref_state)
        current_model_add_dist = get_model_add_dist(obj, state, ref_state)
        current_inp_dist = get_inp_dist(obj, state, ref_state)
    end
end
