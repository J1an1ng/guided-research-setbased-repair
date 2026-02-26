classdef SetBasedControlWeightOptimizer
    properties
        settings PrimitiveGenerationSettings
        dyn_conf ControlledModelParamConfig
        control_weights_fun_creator
        single_primitive_generator

        primitives (:, 1) Primitive

        params

        control_params_lb (:, 1) double
        control_params_ub (:, 1) double

        check_point_file_path string

        cost_def CostDef
    end

    methods

        function obj = SetBasedControlWeightOptimizer(settings, dyn_conf, control_weights_fun_creator, ...
                single_primitive_generator, primitives, control_params_lb, control_params_ub, ...
                check_point_file_path, cost_def)
            obj.settings = settings;
            obj.dyn_conf = dyn_conf;
            obj.control_weights_fun_creator = control_weights_fun_creator;
            obj.single_primitive_generator = single_primitive_generator;

            obj.primitives = primitives;
            fprintf('#primitives used for optimization: %d.\n', length(primitives));

            obj.params = settings.params;

            obj.control_params_lb = control_params_lb;
            obj.control_params_ub = control_params_ub;
            obj.check_point_file_path = check_point_file_path;
            obj.cost_def = cost_def;
        end

        function opt_ctrl_params = optimize(obj)
            dyn = obj.settings.create_dynamics();

            function y = f(x)
                fprintf("Evaluate function...");
                errs = zeros(length(obj.primitives), 1);
                for i = 1:length(obj.primitives)
                    errs(i) = obj.perform_reach_and_get_cost(obj.primitives(i), dyn, x);
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
                'MaxFunctionEvaluations', 3000, ...
                 'MinSurrogatePoints', 60);
            opt_ctrl_params = surrogateopt(@f, obj.control_params_lb, obj.control_params_ub, opts);
            display(opt_ctrl_params);
        end

    end

    methods (Access = private)

        function cost = perform_reach_and_get_cost(obj, p, dyn, control_params)
            fprintf("Reachability analysis primitive...");
            [~, control_weights_fun] = obj.control_weights_fun_creator(obj.settings.k, control_params');
            obj.dyn_conf.set_control_weights(control_weights_fun);
            reach_res = obj.single_primitive_generator.generate(obj.settings, p, dyn, -1);
            if any(reach_res.cora_error) % || reach_res.constraint_violation
                cost = Inf;
                return
            end

            cost = obj.cost_def.evaluate(p, reach_res);
        end

    end
end
