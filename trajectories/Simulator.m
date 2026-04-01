classdef Simulator < handle
    properties (SetAccess = immutable, GetAccess = private)
        terminal_cond AbstractTerminalCondition
        time_decimal_places uint32

        model

        state_order AbstractStateOrder
        inp_order AbstractInpOrder
        params
        constraints AbstractConstraints

        extended_horizon double
        process_final_state
    end

    methods

        function obj = Simulator(terminal_cond, ...
                time_decimal_places, model, state_order, inp_order, params, constraints, ...
                extended_horizon, process_final_state)
            obj.terminal_cond = terminal_cond;
            obj.time_decimal_places = time_decimal_places;

            obj.model = model;
            obj.state_order = state_order;
            obj.inp_order = inp_order;
            obj.params = params;
            obj.constraints = constraints;

            obj.extended_horizon = extended_horizon;
            obj.process_final_state = process_final_state;
        end

        function success = simulate(obj, traj, initial_state, duration, dt_step)
            arguments
                obj Simulator
                traj AbstractInpTraj
                initial_state (:, 1) double
                duration double
                dt_step double
            end

            % norm(traj.target_pos - [2.75; -13.14]) <= 0.1
            num_steps = ceil(duration / dt_step);
            success = true;
            times = cell(num_steps + 1, 1);
            inps = cell(num_steps, 1);
            states = cell(num_steps + 1, 1);

            t = 0;
            i = 1;
            states{1} = initial_state;
            times{1} = 0;
            violation = false;
            while t < duration - 1e-9
                if obj.terminal_cond.check_for_state_hard(states{i})
                    break
                end

                u = traj.inp_at(states{i}, t, dt_step);

                tspan = [0 dt_step];
                fun = @(t, x) obj.model(x, u, ...
                    obj.state_order, obj.inp_order, obj.params);
                % TODO use discrete-time version of model here
                [~, x] = ode45(fun, tspan, states{i});
                x = x(end, :)';

                % TODO it would be better to check x + some minimum set size
                % (e.g. measurment uncertainty)
                if ~obj.constraints.check(x, u)
                    if x(obj.state_order.V) < 0
                        throw(Exception("This should not happen!"));
                    end
                    violation = true;
                    break
                end

                t = round(t + dt_step, obj.time_decimal_places);
                times{i + 1} = t;
                inps{i} = u;
                states{i + 1} = x;
                i = i + 1;
            end

            %% Hint: if a constraint violation occurs, it is better to
            %% discard the trajectory, even if the minimum duration was
            %% already reached, as:
            %%  - this short trajectory is likely to be basically the first
            %%    piece of some other trajectory
            %%  - it is likely that there is no connectable primitive without
            %%    a state violation itself.
            if violation
                success = false;
            end

            times = remove_empty_values(times);
            inps = remove_empty_values(inps);
            states = remove_empty_values(states);

            times = cell2mat(times');
            inps = cell2mat(inps');
            states = cell2mat(states');
            %% Hint: we filter out initial sets if they fulfill the soft
            %% terminal condition, therefore we must do the same here, as otw
            %% we might have primitives without successor primitives
            is_terminal = obj.terminal_cond.check_for_state_soft(states(:, end));

            ext_horizon_states = [];

            if ~violation && obj.extended_horizon > 0
                last_state = obj.process_final_state(x, obj.state_order);
                u = zeros(obj.inp_order.SIZE, 1);
                tspan = 0:dt_step:obj.extended_horizon;
                fun = @(t, x) obj.model(x, u, ...
                        obj.state_order, obj.inp_order, obj.params);
                [~, x] = ode45(fun, tspan, last_state);
                x = x';
                ext_horizon_states = x(:, 2:end);
            end

            %% FIXME this is bad style
            traj.set_simulation_result(times, inps, states, is_terminal, ext_horizon_states);
        end

    end
end
