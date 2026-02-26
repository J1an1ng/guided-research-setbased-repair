classdef ArocPrimitiveGenerator < handle
    properties (Access = private)
        max_num_tries uint32
    end

    methods

        function obj = ArocPrimitiveGenerator(max_num_tries)
            obj.max_num_tries = max_num_tries;
        end

        function res = generate(obj, settings, p, ~, ~, log_fid)
            arguments
                obj ArocPrimitiveGenerator
                settings PrimitiveGenerationSettings
                p Primitive
                ~
                log_fid
            end
            res.terminal = false;
            res.connectable = false;
            res.reachsets = [];
            res.cora_error = [];

            Params = create_aroc_params(settings, p);
            Opts = create_aroc_opts(settings, p);

            approx_final_int = settings.interval_sampler.get_max_final_set(Params.xf);
            q_values = 1 ./ (approx_final_int.sup - approx_final_int.inf);
            q_values = q_values ./ mean(q_values);
            Opts.Q = diag(q_values);

            for i = 1:obj.max_num_tries
                [contr_res, R_primitive] = convexInterpolationControl('truck', Params, Opts);
                % [contr_res, R_primitive] = generatorSpaceControl('truck', Params, Opts);
                % [contr_res, R_primitive] = polynomialControl('truck', Params, Opts);
                R_primitive = R_primitive.reachSet;

                % We simulate random trajectories for the primitive
                if settings.create_plot && settings.do_random_simulations
                    [sim_res, ~, ~] = simulateRandom(contr_res);
                end

                if settings.terminal_cond.check_for_set_soft(contr_res.Rfin)
                    res.terminal = true;
                    break
                end

                if ~p.ref_traj.is_terminal
                    % we need to take the center before shifting, as shifting will
                    % recenter all state dimensions; but - if e.g. the delta range size
                    % depends on the velocity, we want to consider the real velocity
                    c = center(contr_res.Rfin);
                    R_final_proj = settings.transform_initial_set(contr_res.Rfin, settings.k);
                    final_interval_bound = settings.interval_sampler.get_max_final_set(c);
                    if contains(final_interval_bound, R_final_proj)
                        res.connectable = true;
                        break
                    elseif i == obj.max_num_tries
                        my_log(log_fid, settings.debug, "Primitive unconnectable \n");
                    else
                        % update weighting matrix
                        scale = rad(interval(R_final_proj)) ./ rad(final_interval_bound);
                        scale = scale ./ scale(1);
                        Opts.Q = diag(scale);
                    end
                end
            end

            res.reachsets = R_primitive;

            if settings.create_plot
                plot_primitive(settings.k, R_primitive, sim_res);
            end
        end

    end
end

function params = create_aroc_params(settings, p)
    arguments
        settings PrimitiveGenerationSettings
        p Primitive
    end

    params.R0 = p.start_interval;
    b = settings.constraints.inp_bounds;
    params.U = interval(b.lb, b.ub);
    params.W = interval(-settings.input_dist, settings.input_dist);
    params.V = interval(-settings.state_meas_dist, settings.state_meas_dist);
    params.tFinal = p.ref_traj.duration;
    params.xf = p.ref_traj.states(:, end);
end

function opts = create_aroc_opts(settings, p)
    arguments
        settings PrimitiveGenerationSettings
        p Primitive
    end
    opts.nx = settings.k.SIZE;
    opts.nu = settings.i.SIZE;

    % opts.controller = 'quadratic';
    opts.N = p.ref_traj.num_steps;
    opts.Ninter = 1;

    opts.reachSteps = 20;
    % opts.reachStepsFin = 20;

    opts.refInput = 1;
    opts.refTraj.x = p.ref_traj.states;
    opts.refTraj.u = p.ref_traj.inputs;
    % opts.cora = settings.cora_options;

    opts.extHorizon.active = 1;
    opts.extHorizon.horizon = 10; % 0.1s
    opts.extHorizon.decay = 'rise';

    opts.cora.taylorTerms = 20;
    opts.cora.zonotopeOrder = 30;
    opts.cora.tensorOrder = 3;
    % Opts.cora.lagrangeRem.simplify = 'optimize';
    opts.cora.errorOrder = 25;
    opts.cora.intermediateOrder = 20;
end
