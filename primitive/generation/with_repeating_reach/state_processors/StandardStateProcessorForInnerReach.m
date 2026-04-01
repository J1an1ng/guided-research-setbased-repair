classdef StandardStateProcessorForInnerReach < StateProcessor
    properties (Access = private)
        V
    end

    methods (Access = public)

        function obj = StandardStateProcessorForInnerReach(state_meas_dist, ~)
            err_interval = [-state_meas_dist state_meas_dist];
            obj.V = interval(err_interval(:, 1), err_interval(:, 2));
        end

        function r_0 = process_initial_set(~, initial_set)
            r_0 = polytope(initial_set);
        end

        function r_0 = process_initial_set_before_reach(~, r_0)
        end

        function update(~, ~)
            % nothing to do
        end

        function r_0 = process_final_set_after_reach(~, r_final)
            r_0 = polytope(r_final);
        end

        function state = process_state(obj, state, meas_dist)
            state = state(end, :)';
            if ~exist('meas_dist', 'var')
                meas_dist = randPoint(obj.V);
            end
            state = state + meas_dist;
        end

        function r = project_final_set(~, r)
        end

    end
end
