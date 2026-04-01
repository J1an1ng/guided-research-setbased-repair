classdef RepeatingReachPerformer < handle
    properties (Access = private)
        state_proc StateProcessor
        dyn_conf ControlledModelParamConfig
        dyn_conf_values ControlledModelParamValues
    end

    methods

        function obj = RepeatingReachPerformer(state_proc, dyn_conf, dyn_conf_values)
            obj.state_proc = state_proc;
            obj.dyn_conf = dyn_conf;
            obj.dyn_conf_values = dyn_conf_values;
        end

        function res = generate(obj, settings, p, dyn, log_fid)
            arguments
                obj RepeatingReachPerformer
                settings PrimitiveGenerationSettings
                p Primitive
                dyn
                log_fid
            end
            res.terminal = false;
            res.connectable = ~settings.do_connectivity_check;
            res.constraint_violation = false;
            res.reachsets = [];
            res.cora_error = [];

            R_primitive = [];

            R_0 = obj.state_proc.process_initial_set(p.start_set.cora_set);

            t = 0;
            max_steps = size(p.ref_traj.inputs, 2);
            i = 1;
            while i <= max_steps || p.ref_traj.is_terminal
                params.tStart = t;
                params.tFinal = t + settings.dt_control;

                [params.u, params.U] = obj.dyn_conf.create_params(p.ref_traj, min(i, max_steps), obj.dyn_conf_values);

                %% Hint: I tried this for innerReach with timeStepInner = 0.001;
                %% it makes a certain error disappear, but results in another error
                % params.u = repmat(params.u, 1, 10);

                params.R0 = obj.state_proc.process_initial_set_before_reach(R_0);
                % FIXME
                % params.R0 = polyZonotope(params.R0);

                % try
                %R_current = reachInner(dyn, params, settings.cora_options);
                R_current = reach(dyn, params, settings.cora_options);

                % R_current = [];
                % catch e
                %    res.cora_error = e.message;
                %    res.reachsets = R_primitive;
                %    res.sim_res = sim_res;
                %    fprintf("Cora Error: %s\n", e.message);
                %    my_log(log_fid, settings.debug, 'CORA error:%s \n', e.message);
                %    return
                % end

                R_final = R_current(end).timePoint.set{end};
                %% FIXME reduce R_final here as it is not done in Cora anymore

                %% FIXME if necessary, pass the step to methods on the state processor
                %% state_proc.update(i);
                R_0 = obj.state_proc.process_final_set_after_reach(R_final);

                R_current_proj = obj.state_proc.project_final_set(R_current);

                if settings.check_constraints
                    for j_1 = 1:length(R_current_proj)
                        for j_2 = 1:length(R_current_proj(j_1).timeInterval.set)
                            itv = interval(R_current_proj(j_1).timeInterval.set{j_2});
                            if ~settings.constraints.check_interval([itv.inf itv.sup])
                                res.constraint_violation = true;
                                return
                            end
                        end
                    end
                end

                R_primitive = add(R_primitive, R_current_proj);

                t = t + settings.dt_control;

                R_final_proj = R_current_proj(end).timePoint.set{end};

                if settings.terminal_cond.check_for_set_soft(R_final_proj) % t >= 1.0 %||
                    res.terminal = true;
                    break
                end

                %% TODO: improve the dynamic stopping: take the duration with the smallest
                %% reachset in [0.5, 0.75] (not the last one if containment is
                %% never reached)
                %% TODO make this more general: introduce a general primitive-validation, which
                %% can be either the connectivity check or some other check for the
                %% hierarchical generation (e.g. occupancy check etc.)
                %% The validation check to be used can the be configured in the settings.

                % TODO change the connectivity check to a general primitive validation
                if settings.do_connectivity_check ...
                        && ~p.ref_traj.is_terminal ...
                        && t >= settings.min_primitive_duration
                    % we need to take the center befoore shifting, as shifting will
                    % recenter all state dimensions; but - if e.g. the delta range size
                    % depends on the velocity, we want to consider the real velocity
                    c = center(R_final_proj);
                    R_final_proj = settings.transform_initial_set(R_final_proj, settings.k);
                    final_interval_bound = settings.state_space_divider.get_max_final_set(c);
                    if contains(final_interval_bound, R_final_proj)
                        res.connectable = true;
                        break
                    elseif i == max_steps
                        my_log(log_fid, settings.debug, "Primitive unconnectable \n");
                    end
                end
                %% TODO: for monolithic primitives we do not want a containment check,
                %% thus it should be possible to disable it.

                i = i + 1;
            end

            res.reachsets = R_primitive;
        end

    end
end


