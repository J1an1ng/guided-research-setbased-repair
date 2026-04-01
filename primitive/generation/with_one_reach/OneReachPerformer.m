classdef OneReachPerformer < handle
    properties (Access = private)
        state_processor_class
        dyn_conf ControlledModelParamConfig
        dyn_conf_values ControlledModelParamValues
        sim RandomSimulator
    end

    methods

        function obj = OneReachPerformer(state_processor_class, dyn_conf, dyn_conf_values, sim)
            obj.state_processor_class = state_processor_class;
            obj.dyn_conf = dyn_conf;
            obj.dyn_conf_values = dyn_conf_values;
            obj.sim = sim;
        end

        function res = generate(obj, settings, p, dyn, ~)
            arguments
                obj OneReachPerformer
                settings PrimitiveGenerationSettings
                p Primitive
                dyn
                ~
            end
            res.terminal = false;
            res.connectable = ~settings.do_connectivity_check;
            res.constraint_violation = false;
            res.reachsets = [];
            res.cora_error = [];

            sim_res = [];

            state_proc = obj.state_processor_class(settings.state_meas_dist, settings.k);
            R_0 = state_proc.process_initial_set(p.start_set.cora_set);
            R_0 = state_proc.process_initial_set_before_reach(R_0);

            max_steps = size(p.ref_traj.inputs, 2);

            [params.u, params.U] = obj.dyn_conf.create_params(p.ref_traj, 1:max_steps, obj.dyn_conf_values);

            params.tStart = 0;
            params.tFinal = p.ref_traj.duration;
            params.R0 = R_0;

            options = settings.cora_options;
            options.timeStep = settings.dt_control;

            % try
            R_primitive = reach(dyn, params, options);
            % catch e
            %    res.cora_error = e.message;
            %    fprintf("Cora error: %s\n", e.message);
            %    my_log(log_fid, settings.debug, 'CORA error:%s \n', e.message);
            %    return
            % end

            if settings.create_plot && settings.do_random_simulations
                sim_res = obj.sim.simulate(p.start_set.cora_set, p.ref_traj, 0, p.ref_traj.duration);
            end

            res.reachsets = R_primitive; % project(R_primitive, 1:settings.k.SIZE);

            if settings.create_plot
                plot_primitive(settings, p, R_primitive, sim_res);
            end
        end

    end
end
