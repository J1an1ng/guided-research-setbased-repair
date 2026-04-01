classdef (Abstract) AbstractInpTraj < handle & JsonEncodable & matlab.mixin.Heterogeneous
    properties (SetAccess = private)
        times (1, :) double
        states (:, :) double
        inputs (:, :) double
        is_terminal logical

        ext_horizon_states (:, :) double
    end

    properties (Dependent)
        duration double
        num_steps uint32
    end

    methods

        function value = get.duration(obj)
            value = obj.times(end);
        end

        function value = get.num_steps(obj)
            value = length(obj.inputs);
        end

    end

    methods

        function set_simulation_result(obj, times, inputs, states, ...
                is_terminal, ext_horizon_states)
            obj.times = times;
            obj.inputs = inputs;
            obj.states = states;
            obj.is_terminal = is_terminal;
            obj.ext_horizon_states = ext_horizon_states;
        end

        function res = export_to_struct(obj)
            % we transpose states and inputs as we want a row to be a state or input
            res = struct('states', obj.states', 'inputs', obj.inputs');
        end

        function params = create_cora_sim_params(obj, dyn_conf, dyn_conf_values, initial_itv_rads)
            %% TODO remove this method again
            arguments
                obj AbstractInpTraj
                dyn_conf ControlledModelParamConfig
                dyn_conf_values ControlledModelParamValues
                initial_itv_rads (:, 1) double
            end
            [params.u, params.U] = dyn_conf.create_params_for_random_sim(obj, 1:size(obj.inputs, 2), dyn_conf_values);
            params.tStart = 0;
            params.tFinal = obj.duration;
            initial_state = obj.states(:, 1);
            params.R0 = zonotope(interval(initial_state - initial_itv_rads, initial_state + initial_itv_rads));
        end

    end

    methods (Abstract)
        inp = inp_at(obj, state, t, dt)
    end
end
