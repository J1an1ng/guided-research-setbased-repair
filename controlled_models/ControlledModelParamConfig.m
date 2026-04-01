classdef ControlledModelParamConfig < handle & matlab.mixin.Heterogeneous
    properties (Access = private)
        state_order AbstractStateOrder
        inp_order AbstractInpOrder

        ref_state_size uint32
        include_ref_inp logical = false
        control_weights_size uint32
    end

    properties (Access = public)
        is_discrete_time_system logical = false

        include_meas_dist logical
        include_model_add_dist logical
        include_model_inp_dist logical
    end

    methods

        function obj = ControlledModelParamConfig(is_discrete_time_system, ...
                state_order, inp_order, ...
                include_meas_dist, ...
                include_model_add_dist, ...
                include_model_inp_dist, ...
                ref_state_size, include_ref_inp, control_weights_size)
            obj.is_discrete_time_system = is_discrete_time_system;

            obj.state_order = state_order;
            obj.inp_order = inp_order;
            obj.include_meas_dist = include_meas_dist;

            obj.include_model_add_dist = include_model_add_dist;

            obj.include_model_inp_dist = include_model_inp_dist;

            obj.ref_state_size = ref_state_size;
            obj.include_ref_inp = include_ref_inp;

            obj.control_weights_size = control_weights_size;
        end

        function U = create_scaffold_for_param_uncertainties(obj, values)
            U = zonotope(interval([], []));

            if obj.include_meas_dist
                gen_1 = eye(obj.state_order.get_size());
                % gen_1(3, 3) = 0;
                U = zonotope(cartProd(U, zonotope([zeros(obj.state_order.get_size(), 1) ...
                                                   gen_1 ...
                                                  ])));
                % ones(obj.state_order.get_size(), 1)
            end

            if obj.include_model_add_dist
                gen_1 = eye(obj.state_order.get_size());
                % gen_1(3, 3) = 0;
                model_add_dist_U = zonotope([zeros(obj.state_order.get_size(), 1) ...
                                             gen_1 ...
                                            ]);
                % ones(obj.state_order.get_size(), 1)
                U = zonotope(cartProd(U, model_add_dist_U));
            end

            if obj.include_model_inp_dist
                U = zonotope(cartProd(U, zonotope([zeros(obj.inp_order.SIZE, 1) ...
                                                   eye(obj.inp_order.SIZE) ...
                                                   ones(obj.inp_order.SIZE, 1)])));
            end

            U = zonotope(cartProd(U, zeros(values.ref_state_builder.get_ref_state_size(), 1)));

            if obj.include_ref_inp
                U = zonotope(cartProd(U, zeros(obj.inp_order.SIZE, 1)));
            end
            U = zonotope(cartProd(U, zeros(obj.control_weights_size, 1)));
        end

        function [u, U] = create_params_for_random_sim(obj, ref_traj, time_steps, values)
            arguments
                obj ControlledModelParamConfig
                ref_traj AbstractInpTraj
                time_steps (1, :) uint32
                values ControlledModelParamValues
            end

            [u, U] = obj.create_params(ref_traj, time_steps, values);

            if obj.is_discrete_time_system
                %% for simulation of a discrete time system,
                %% the last input is not used due to a bug in Cora
                u = u(:, 1:end - 1);
            end
        end

        function U = create_params_uncertainties(obj, values)
            arguments
                obj ControlledModelParamConfig
                values ControlledModelParamValues
            end

            U = zonotope(interval([], []));

            if ~isempty(values.combined_dist_set)
                U = zonotope(cartProd(U, values.combined_dist_set));
            else
                %% FIXME: this is only a temporary hack; better:
                %% always create combined dist set in run-script already
                if obj.include_meas_dist
                    U = zonotope(cartProd(U, values.meas_dist_itv));
                end

                if obj.include_model_add_dist
                    U = zonotope(cartProd(U, values.model_add_dist_set));
                end

                if obj.include_model_inp_dist
                    U = zonotope(cartProd(U, values.model_inp_dist_set));
                end
            end

            U = zonotope(cartProd(U, zeros(obj.ref_state_size, 1)));

            if obj.include_ref_inp
                U = zonotope(cartProd(U, zeros(obj.inp_order.SIZE, 1)));
            end
            U = zonotope(cartProd(U, zeros(obj.control_weights_size, 1)));
        end

        function [u, U] = create_params(obj, ref_traj, time_steps, values)
            arguments
                obj ControlledModelParamConfig
                ref_traj AbstractInpTraj
                time_steps (1, :) uint32
                values ControlledModelParamValues
            end

            n = length(time_steps);

            if obj.is_discrete_time_system
                %% for discrete time systems, an input for the 0-th time step is needed;
                %% I know this is somewhat hacky...
                n = n + 1;
            end

            u = [];

            if obj.include_meas_dist
                u = [u; zeros(obj.state_order.get_size(), n)];
            end

            if obj.include_model_add_dist
                u = [u; zeros(obj.state_order.get_size(), n)];
            end

            if obj.include_model_inp_dist
                u = [u; zeros(obj.inp_order.SIZE, n)];
            end

            ref_states = arrayfun(@(i) values.ref_state_builder.build_ref_state(ref_traj, values.ref_state_args, i), ...
                    time_steps, 'UniformOutput', false);
            ref_states = cell2mat(ref_states);

            if obj.is_discrete_time_system
                ref_states = [ref_states, zeros(obj.ref_state_size, 1)];
            end

            u = [u; ref_states];

            if obj.include_ref_inp
                ref_inputs = ref_traj.inputs(:, time_steps);

                if obj.is_discrete_time_system
                    ref_inputs = [ref_inputs, zeros(obj.inp_order.SIZE, 1)];
                end

                u = [u; ref_inputs];
            end

            w = values.get_control_weights(ref_traj, time_steps);

            %% FIXME hack
            % w(1) = (1 + time_steps * 0.2) * w(1); %% 0.05
            %% w(1) = (1 + min([time_steps * 0.1, 5])) * w(1); %% 0.5

            u = [u; repmat(w, 1, n)];
            U = obj.create_params_uncertainties(values);
        end

        function u = create_params_for_sim(obj, ref_traj, time_steps, values, current_meas_dist, ...
                current_model_add_dist, current_model_inp_dist)
            n = length(time_steps);

            u = [];

            if obj.include_meas_dist
                if ~exist('current_meas_dist', 'var')
                    current_meas_dist = randPoint(values.meas_dist_itv);
                end

                u = [u; repmat(current_meas_dist, 1, n)];
            end

            if obj.include_model_add_dist
                if ~exist('current_model_add_dist', 'var')
                    current_model_add_dist = randPoint(values.model_add_dist_set);
                end

                u = [u; repmat(current_model_add_dist, 1, n)];
            end

            if obj.include_model_inp_dist
                if ~exist('current_model_inp_dist', 'var')
                    current_model_inp_dist = randPoint(values.model_inp_dist_itv);
                end

                u = [u; repmat(current_model_inp_dist, 1, n)];
            end

            ref_states = arrayfun(@(i) values.ref_state_builder.build_ref_state(ref_traj, values.ref_state_args, i), ...
                    time_steps, 'UniformOutput', false);
            ref_states = cell2mat(ref_states);
            u = [u; ref_states];

            if obj.include_ref_inp
                ref_inputs = ref_traj.inputs(:, time_steps);
                u = [u; ref_inputs];
            end

            w = values.get_control_weights(ref_traj, time_steps);

            %% FIXME hack
            %% w(1) = (1 + min([time_steps * 0.1, 5])) * w(1); %% 0.5

            u = [u; repmat(w, 1, n)];
        end

        function current_meas_dist = get_current_meas_dist(obj, current_params)
            if obj.include_meas_dist
                state_sep1 = obj.state_order.get_size();
                current_meas_dist = current_params(1:state_sep1, :);
            else
                current_meas_dist = zeros(obj.state_order.get_size(), 1);
            end
        end

        function [current_meas_dist, current_model_add_dist, current_inp_dist, ...
            ref_state, ref_inp, weights] = split(obj, current_params)

            k_size = obj.state_order.SIZE;

            if obj.include_meas_dist
                state_sep1 = k_size;
                current_meas_dist = current_params(1:state_sep1, :);
            else
                state_sep1 = 0;
                current_meas_dist = zeros(k_size, 1);
            end

            if obj.include_model_add_dist
                state_sep2 = state_sep1 + k_size;
                current_model_add_dist = current_params(state_sep1 + 1:state_sep2, :);
            else
                state_sep2 = state_sep1;
                current_model_add_dist = zeros(k_size, 1);
            end

            if obj.include_model_inp_dist
                state_sep3 = state_sep2 + obj.inp_order.SIZE;
                current_inp_dist = current_params(state_sep2 + 1:state_sep3, :);
            else
                state_sep3 = state_sep2;
                current_inp_dist = zeros(obj.inp_order.SIZE, 1);
            end

            state_sep4 = state_sep3 + obj.ref_state_size;
            ref_state = current_params(state_sep3 + 1:state_sep4, :);

            if obj.include_ref_inp
                state_sep5 = state_sep4 + obj.inp_order.SIZE;
                ref_inp = current_params(state_sep4 + 1:state_sep5, :);
            else
                state_sep5 = state_sep4;
                ref_inp = zeros(obj.inp_order.SIZE, 1);
            end

            state_sep6 = state_sep5 + obj.control_weights_size;
            weights = current_params(state_sep5 + 1:state_sep6, :);
        end

        function res = get_params_size(obj)
            res = double(obj.include_meas_dist) * obj.state_order.get_size() + ...
                double(obj.include_model_add_dist) * obj.state_order.get_size() + ...
                double(obj.include_model_inp_dist) * obj.inp_order.SIZE + ...
                obj.ref_state_size + ...
                double(obj.include_ref_inp) * obj.inp_order.SIZE + ...
                obj.control_weights_size;
        end

    end

    methods (Access = private)

        function check_values(obj, values)
            arguments
                obj ControlledModelParamConfig
                values ControlledModelParamValues
            end

            assert(~obj.include_meas_dist || values.meas_dist_itv.dim == obj.state_order.get_size(), ...
                'Measured distance does not match the expected size.');
            assert(~obj.include_model_add_dist || values.model_add_dist_set.dim == obj.state_order.get_size(), ...
                'Model add distance does not match the expected size.');
            assert(~obj.include_model_inp_dist || values.model_inp_dist_itv.dim == obj.inp_order.SIZE, ...
                'Model input distance does not match the expected size.');
            assert(values.ref_state_builder.get_ref_state_size() == obj.ref_state_size, ...
                'Ref state size does not match the expected size.');
            assert(~isempty(values.control_weights_fun) || length(values.control_weights) == obj.control_weights_size, ...
                'Control weights size does not match the expected size.');
        end

    end
end
