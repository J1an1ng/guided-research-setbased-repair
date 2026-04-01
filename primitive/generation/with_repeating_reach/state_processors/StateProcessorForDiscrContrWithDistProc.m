classdef StateProcessorForDiscrContrWithDistProc < StateProcessor
    properties (Access = private)
        state_order AbstractStateOrder
        meas_dist % zonotope
        dist_proc AbstractDistProcessor
        actual_state_dims (1, :) uint32
    end

    methods (Access = public)

        function obj = StateProcessorForDiscrContrWithDistProc(state_order, meas_dist, dist_proc)
            k = state_order;
            obj.state_order = k;

            meas_dist_itv = interval(-meas_dist, meas_dist);
            obj.meas_dist = zonotope(meas_dist_itv);
            obj.dist_proc = dist_proc;
            obj.actual_state_dims = 1:k.get_size();
        end

        function r_0 = process_initial_set(obj, initial_set)
            r_0 = ZonotopeConverter.convert_to_zono(initial_set);
            % We copy the state (as the control input is computed for the last state measurement);
            % we denote this copy as "init_state" throughout this class:
            r_0 = project(r_0, [1:dim(r_0) obj.actual_state_dims]);
            % We concatenate each possible measurement disturbance to each state:
            r_0 = cartProd(r_0, obj.meas_dist);
        end

        function r_0 = process_initial_set_before_reach(~, r_0)
            r_0 = reduce(r_0, 'pca', 100); % 20
        end

        function r_0 = process_final_set_after_reach(obj, r_final)
            r_0 = ZonotopeConverter.convert_to_zono(r_final);
            % We only take the actual state and copy the it (for the same reason as in process_initial_set):
            r_0 = project(r_0, [obj.actual_state_dims obj.actual_state_dims]);
            % We concatenate each possible measurement disturbance to each state:
            r_0 = cartProd(r_0, obj.meas_dist);
        end

        function r = project_final_set(obj, r)
            r = project(r, obj.actual_state_dims);
        end

        function state = process_state_before_sim(obj, state, meas_dist)
            if ~exist('meas_dist', 'var')
                meas_dist = randPoint(obj.meas_dist);
            end

            state = [state; state; meas_dist];
            state = state';
        end

        function state = process_state_after_sim(obj, state)
            state = state(obj.actual_state_dims)';
        end

        function state_size = get_state_size(obj)
            state_size = 2 * obj.state_order.get_size() + obj.meas_dist.dim;
        end

        function [state, measured_state] = split_and_process(obj, state_collection)
            k = obj.state_order;
            k_size = k.SIZE;
            state_sep1 = k_size;
            state_sep2 = state_sep1 + k_size;

            state = state_collection(1:state_sep1);

            init_state = state_collection(state_sep1 + 1:state_sep2);
            current_meas_dist = state_collection(state_sep2 + 1:end);
            measured_state = obj.dist_proc.process_meas_dist(init_state, current_meas_dist);
        end

        function dx = create_derivative(obj, state, derivative_state, model_add_dist)
            derivative_state = obj.dist_proc.process_model_add_dist(state, derivative_state, model_add_dist);

            k = obj.state_order;
            k_size = k.SIZE;
            state_sep1 = k_size;
            state_sep2 = state_sep1 + k_size;
            state_sep3 = state_sep2 + k_size;

            dx(1:state_sep1, 1) = derivative_state;
            dx(state_sep1 + 1:state_sep2, 1) = zeros(k_size, 1);
            dx(state_sep2 + 1:state_sep3, 1) = zeros(k_size, 1);
        end

    end
end
