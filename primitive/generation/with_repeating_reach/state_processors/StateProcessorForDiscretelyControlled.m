classdef StateProcessorForDiscretelyControlled < StateProcessor
    properties (Access = private)
        state_order AbstractStateOrder
        V % interval

        actual_state_dims (1, :) uint32
    end

    methods (Access = public)

        function obj = StateProcessorForDiscretelyControlled(state_meas_dist, state_order)
            k = state_order;
            obj.state_order = k;

            err_interval = [-state_meas_dist state_meas_dist];
            err_interval = [zeros(k.get_size(), 2); err_interval];
            obj.V = interval(err_interval(:, 1), err_interval(:, 2));

            obj.actual_state_dims = 1:k.get_size();
        end

        function r_0 = process_initial_set(obj, initial_set)
            r_0 = ZonotopeConverter.convert_to_zono(initial_set);
            r_0 = project(r_0, [1:dim(r_0) obj.actual_state_dims]);
            % ... and add the disturbance to the state copy:
            r_0 = r_0 + obj.V;
        end

        function r_0 = process_initial_set_before_reach(~, r_0)
            r_0 = reduce(r_0, 'pca', 100); % 20
        end

        %% FIXME remove this method; keep this class immutable;
        %% create the object already in the runner script; pass the step to
        %% the other method that uses the matrix and compute it on demand only
        %% (and recompute whenever needed)
        function update(~, ~)
            % nothing to do
        end

        function r_0 = process_final_set_after_reach(obj, r_final)
            r_0 = ZonotopeConverter.convert_to_zono(r_final);
            r_0 = project(r_0, [obj.actual_state_dims obj.actual_state_dims]);
            r_0 = r_0 + obj.V;
        end

        function state = process_state(obj, state, meas_dist)
            state = state(end, obj.actual_state_dims)';

            meas_dist_with_correct_dims = randPoint(obj.V);

            if exist('meas_dist', 'var')
                meas_dist_with_correct_dims(obj.state_order.get_size() + 1:end) = meas_dist;
            end

            state = [state; state] + meas_dist_with_correct_dims;
        end

        function r = project_final_set(obj, r)
            r = project(r, obj.actual_state_dims);
        end

    end
end
