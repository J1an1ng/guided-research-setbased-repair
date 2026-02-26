classdef StateProcessorForDiscretelyControlledWithObserver < StateProcessor

    properties (Access = private)
        state_order AbstractStateOrder
        state_meas_dist (:, 1) double
        V % interval
        V_2 % interval

        actual_state_dims (1, :) uint32
        propagated_observed_state_dims (1, :) uint32

        current_transform_matrix (:, :) double
    end

    methods (Access = public)

        function obj = StateProcessorForDiscretelyControlledWithObserver(state_meas_dist, state_order)
            k = state_order;
            obj.state_order = k;
            obj.state_meas_dist = state_meas_dist;

            err_interval = [-state_meas_dist state_meas_dist];
            err_interval = [zeros(k.get_size(), 2); err_interval];
            obj.V = interval(err_interval(:, 1), err_interval(:, 2));

            err_interval = [zeros(k.get_size(), 2); err_interval];
            obj.V_2 = interval(err_interval(:, 1), err_interval(:, 2));

            obj.actual_state_dims = 1:k.get_size();
            obj.propagated_observed_state_dims = 2 * k.get_size() + 1:3 * k.get_size();
        end

        function r_0 = process_initial_set(obj, initial_set)
            r_0 = ZonotopeConverter.convert_to_zono(initial_set);
            r_0 = project(r_0, [1:dim(r_0) 1:obj.state_order.get_size()]);
            % ... and add the disturbance to the state copy; initially, the
            % observed state equals the measured state
            r_0 = r_0 + obj.V;
        end

        function r_0 = process_initial_set_before_reach(obj, r_0)
            k = obj.state_order;
            r_0 = ZonotopeConverter.convert_to_zono(r_0);
            %% FIXME make target order customizable
            % r_0 = reduce(r_0, 'pca', 5);
            duplicate_start = k.get_size() + 1;
            r_0 = project(r_0, [1:dim(r_0) duplicate_start:duplicate_start + k.get_size() - 1]);
        end

        function update(obj, i_step)
            obj.compute_transform_matrix(i_step);
        end

        function r_0 = process_final_set_after_reach(obj, r_final)
            r_0 = ZonotopeConverter.convert_to_zono(r_final);
            r_0 = project(r_0, [obj.actual_state_dims ...
                                obj.propagated_observed_state_dims obj.actual_state_dims]);

            %% TODO: maybe crop all v <= 0 away --> create method in terminal-cond
            %% doing that if necessary
            % k = obj.state_order;
            % r_0_itv = interval(r_0);
            % r_0_itv.sup(k.V)

            r_0 = r_0 + obj.V_2;
            % We apply the transformation to r_0 (i.e. we take the linear
            % combination of the observed and measured value for each
            % dimension); the result contains the actual state and the new
            % observer.
            r_0 = obj.current_transform_matrix * r_0;
        end

        function state = process_state(obj, state, meas_dist)
            state = state(end, [obj.actual_state_dims obj.propagated_observed_state_dims])';

            meas_dist_with_correct_dims = randPoint(obj.V_2);

            if exist('meas_dist', 'var')
                meas_dist_with_correct_dims(2 * obj.state_order.get_size() + 1:end) = meas_dist;
            end

            state = [state; state(obj.actual_state_dims)] + meas_dist_with_correct_dims;
            state = obj.current_transform_matrix * state;
            state = [state; state(obj.state_order.get_size() + 1:end)];
        end

        function r = project_final_set(obj, r)
            r = project(r, obj.actual_state_dims);
        end

    end

    methods (Access = private)

        function compute_transform_matrix(obj, i_step)
            k = obj.state_order;

            % We create the transformation matrix for combining the
            % observer and the new measurement to the new observer:
            q = 1 / (double(i_step) + 1); % fraction of currently measured value
            p = 1 - q; % fraction of observer
            combine_obs_and_measured = [diag(p * ones(1, k.get_size())) diag(q * ones(1, k.get_size()))];
            combine_obs_and_measured = blkdiag(eye(k.get_size()), combine_obs_and_measured);
            obj.current_transform_matrix = combine_obs_and_measured;
        end

    end
end
