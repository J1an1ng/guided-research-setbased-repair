classdef StandardSimulator < RandomSimulator
    properties (Access = private)
        dyn
        param_conf ControlledModelParamConfig
    end

    methods

        function obj = StandardSimulator(dyn, param_conf)
            obj.dyn = dyn;
            obj.param_conf = param_conf;
        end

        function res = simulate(obj, p, res, i, params, state_proc, current_reachset)
            if i == 1
                res = obj.simulate_first(params, current_reachset);
            else
                res = obj.simulate_step(p, res, i, params, state_proc);
            end
        end

        function res = simulate_first(obj, params, current_reachset)
            sim_options = create_random_sim_options(length(current_reachset.timePoint.set) - 1);
            res = simulateRandom(obj.dyn, params, sim_options);
        end

    end

    methods (Access = private)

        function res = simulate_step(obj, p, res, i, params, state_proc)
            simParams.tStart = params.tStart;
            simParams.tFinal = params.tFinal;
            max_steps = size(p.ref_traj.inputs, 2);
            for k = 1:length(res)
                r = res(k);
                x0 = r.x{i - 1};
                simParams.x0 = state_proc.process_state(x0);
                simParams.u = obj.param_conf.create_params_for_sim(p.ref_traj, min(i, max_steps));
                [t_res, x_res] = simulate(obj.dyn, simParams);
                res(k) = add(r, simResult({x_res}, {t_res}));
            end
        end

    end
end

function options = create_random_sim_options(nr_const_inp)
    options.points = 100;
    options.fracVert = 0;
    % options.fracInpVert = 1;
    % options.type = 'gaussian';
    options.nrConstInp = nr_const_inp; % FIXME do not hardcode
    % options.p_conf = 0.99;

end
