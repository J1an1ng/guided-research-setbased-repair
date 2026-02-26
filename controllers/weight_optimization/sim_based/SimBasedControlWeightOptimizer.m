classdef SimBasedControlWeightOptimizer
    properties
        ref_trajs (:, 1) AbstractInpTraj
        simulator RandomCoraSimulator
        sim_sampler SimulationSampler
        dt_control double
        num_sims int32
        initial_itv_rads (:, 1) double

        control_params_lb (:, 1) double
        control_params_ub (:, 1) double

        ref_state_args_lb
        ref_state_args_ub

        check_point_file_path string

        cost_def SimCostDef

        % If true, the max cost of the simulations is used, otherwise the mean cost is used
        take_max_cost_of_sims logical
    end

    methods

        function obj = SimBasedControlWeightOptimizer(ref_trajs, simulator, sim_sampler, dt_control, ...
                num_sims, initial_itv_rads, ...
                control_params_lb, control_params_ub, ...
                ref_state_args_lb, ref_state_args_ub, ...
                check_point_file_path, cost_def, take_max_cost_of_sims)
            obj.ref_trajs = ref_trajs;
            obj.simulator = simulator;
            obj.sim_sampler = sim_sampler;
            obj.dt_control = dt_control;
            obj.num_sims = num_sims;
            obj.initial_itv_rads = initial_itv_rads;

            obj.control_params_lb = control_params_lb;
            obj.control_params_ub = control_params_ub;
            obj.ref_state_args_lb = ref_state_args_lb;
            obj.ref_state_args_ub = ref_state_args_ub;
            assert(all(obj.ref_state_args_lb.keys() == obj.ref_state_args_ub.keys()), ...
                'Ref state args keys must be the same');
            obj.check_point_file_path = check_point_file_path;
            obj.cost_def = cost_def;
            obj.take_max_cost_of_sims = take_max_cost_of_sims;
        end

        function opt_ctrl_params = optimize(obj)

            function y = f(x)
                fprintf("Evaluate function...");

                control_params = x(1:length(obj.control_params_lb));
                ref_state_args_array = x(length(obj.control_params_lb) + 1:end);
                ref_state_args = dictionary();
                ref_state_args_keys = obj.ref_state_args_lb.keys();
                for i = 1:length(ref_state_args_keys)
                    k = ref_state_args_keys(i);
                    ref_state_args(k) = ref_state_args_array(i);
                end

                new_values = obj.simulator.dyn_conf_values.with(control_weights = control_params', ref_state_args = ref_state_args);

                errs = zeros(length(obj.ref_trajs), 1);
                for i = 1:length(obj.ref_trajs)
                    errs(i) = obj.run_simulations_and_get_cost(obj.ref_trajs(i), new_values);
                    if errs(i) == Inf
                        y = Inf;
                        return
                    end
                end
                y = mean(errs);
            end

            % Hint: for MaxFunctionEvaluations, higher values than the currently
            % one are recommended
            opts = optimoptions('surrogateopt', 'CheckpointFile', obj.check_point_file_path, ...
                'UseParallel', true, 'PlotFcn', 'surrogateoptplot', ...
                'MaxFunctionEvaluations', 10000, ...
                 'MinSurrogatePoints', 60);

            all_params_lb = [obj.control_params_lb; obj.ref_state_args_lb.values()];
            all_params_ub = [obj.control_params_ub; obj.ref_state_args_ub.values()];

            opt_ctrl_params = surrogateopt(@f, all_params_lb, all_params_ub, opts);
            display(opt_ctrl_params);
        end

    end

    methods (Access = private)

        function cost = run_simulations_and_get_cost(obj, ref_traj, new_values)
            fprintf("Run simulations...");

            % Important: use a recent enough version of CORA that supports simResult.error
            errorsToCatch = [InputOutOfBoundsException.ID];

            sim_res = obj.simulator.simulate(obj.num_sims, ref_traj, obj.initial_itv_rads, obj.dt_control, ...
                errorsToCatch, obj.sim_sampler, new_values);

            if obj.take_max_cost_of_sims
                reduce_fun = @max;
            else
                reduce_fun = @mean;
            end

            cost = reduce_fun(arrayfun(@(sim) obj.cost_def.evaluate(ref_traj, sim), sim_res));
        end

    end
end
