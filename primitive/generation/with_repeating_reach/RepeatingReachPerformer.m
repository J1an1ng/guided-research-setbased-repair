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
                % Each reach() call advances exactly one timeStep, so params.u
                % must have exactly 1 column. create_params already returns a
                % single column when given a scalar time_step index.
                assert(size(params.u, 2) == 1, ...
                    'params.u must have exactly 1 column per reach step, got %d', size(params.u, 2));

                my_log(log_fid, settings.debug, ...
                    ['Reach step i=%d, max_steps=%d, tStart=%.6f, tFinal=%.6f, timeStep=%.6f, ' ...
                     'size(params.u)=[%d,%d]\n'], ...
                    i, max_steps, params.tStart, params.tFinal, settings.cora_options.timeStep, ...
                    size(params.u, 1), size(params.u, 2));

                %% Hint: I tried this for innerReach with timeStepInner = 0.001;
                %% it makes a certain error disappear, but results in another error
                % params.u = repmat(params.u, 1, 10);

                params.R0 = obj.state_proc.process_initial_set_before_reach(R_0);
                % FIXME
                % params.R0 = polyZonotope(params.R0);

                [r0_dim, r0_gens] = get_set_shape(params.R0);
                [u_dim, u_gens] = get_set_shape(params.U);

                if has_invalid_numeric(params.u) || has_invalid_numeric_set(params.R0)
                    res.cora_error = "Invalid numeric input (NaN/Inf) before reach";
                    my_log(log_fid, settings.debug, 'CORA error: %s\n', res.cora_error);
                    return
                end

                my_log(log_fid, settings.debug, ...
                    'Set stats: R0(dim=%d, gens=%d), U(dim=%d, gens=%d)\n', ...
                    r0_dim, r0_gens, u_dim, u_gens);

                try
                    % R_current = reachInner(dyn, params, settings.cora_options);
                    R_current = reach(dyn, params, settings.cora_options);
                catch e
                    res.cora_error = e.message;
                    res.reachsets = R_primitive;

                    my_log(log_fid, settings.debug, 'CORA error identifier: %s\n', e.identifier);
                    my_log(log_fid, settings.debug, 'CORA error message: %s\n', e.message);
                    my_log(log_fid, settings.debug, ...
                        ['Context at error: i=%d, max_steps=%d, tStart=%.6f, tFinal=%.6f, timeStep=%.6f, ' ...
                         'size(params.u)=[%d,%d], size(params.R0 center)=[%d,%d]\n'], ...
                        i, max_steps, params.tStart, params.tFinal, settings.cora_options.timeStep, ...
                        size(params.u, 1), size(params.u, 2), size(center(params.R0), 1), size(center(params.R0), 2));

                    if ~isempty(e.stack)
                        my_log(log_fid, settings.debug, 'CORA error stack:\n');
                        for s = 1:length(e.stack)
                            st = e.stack(s);
                            my_log(log_fid, settings.debug, '  #%d %s (line %d) in %s\n', ...
                                s, st.file, st.line, st.name);
                        end
                    end
                    return
                end

                R_final = R_current(end).timePoint.set{end};

                % --- Generator-explosion guard ---
                % CORA no longer reduces internally after each reach step,
                % so we must reduce R_final before feeding it back as R_0.
                % Use the same zonotopeOrder that CORA uses internally
                % (default 50) and Girard reduction.  A hard cap triggers
                % an additional aggressive reduction if generators still
                % exceed the threshold after the first pass.
                ZONO_ORDER  = 50;   % matches options.zonotopeOrder
                GEN_HARD_CAP = 120; % absolute generator count ceiling

                gens_before = get_num_gens(R_final);
                R_final = reduce(R_final, 'girard', ZONO_ORDER);
                gens_after_first = get_num_gens(R_final);

                if gens_after_first > GEN_HARD_CAP
                    % Emergency reduction — shrink to a tighter order
                    emergency_order = max(2, floor(GEN_HARD_CAP / dim(R_final)));
                    R_final = reduce(R_final, 'girard', emergency_order);
                end
                gens_after = get_num_gens(R_final);

                if i <= 5
                    my_log(log_fid, settings.debug, ...
                        ['  R_final reduction: gens %d -> %d -> %d ' ...
                         '(order=%d, cap=%d)\n'], ...
                        gens_before, gens_after_first, gens_after, ...
                        ZONO_ORDER, GEN_HARD_CAP);
                end

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

function res = has_invalid_numeric(x)
    res = any(isnan(x(:)) | isinf(x(:)));
end

function res = has_invalid_numeric_set(S)
    try
        itv = interval(S);
        res = any(isnan(itv.inf(:)) | isinf(itv.inf(:)) | isnan(itv.sup(:)) | isinf(itv.sup(:)));
    catch
        try
            c = center(S);
            res = any(isnan(c(:)) | isinf(c(:)));
        catch
            res = true;
        end
    end
end

function [set_dim, num_gens] = get_set_shape(S)
    set_dim = -1;
    num_gens = -1;

    try
        set_dim = dim(S);
    catch
        try
            c = center(S);
            set_dim = size(c, 1);
        catch
        end
    end

    % For zonotope specifically, check generator matrix
    try
        if isa(S, 'zonotope')
            num_gens = size(S.G, 2);
        end
    catch
    end
end

function S_fixed = fix_zonotope_dimension_mismatch(~, ~, ~) 
    % REMOVED: This hotfix is no longer needed.
    % The root cause (split.m accessing G columns beyond size(G,2)) is now
    % fixed directly in CORA's split.m and priv_select.m.
    S_fixed = [];
    error('fix_zonotope_dimension_mismatch:removed', ...
        'This hotfix has been removed. The fix is now in CORA split.m.');
end

function n = get_num_gens(S)
    % Return the number of generators for a zonotope-like set.
    n = 0;
    try
        if isa(S, 'zonotope')
            n = size(S.G, 2);
        elseif isa(S, 'polyZonotope')
            n = size(S.G, 2) + size(S.GI, 2);
        elseif isa(S, 'conZonotope')
            n = size(S.G, 2);
        else
            % fallback: try generators property
            n = size(generators(S), 2);
        end
    catch
        n = -1;
    end
end


