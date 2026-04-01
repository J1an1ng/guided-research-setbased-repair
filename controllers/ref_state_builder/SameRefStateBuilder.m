classdef SameRefStateBuilder < AbstractRefStateBuilder
    properties (Access = private)
        k AbstractStateOrder
    end

    methods

        function obj = SameRefStateBuilder(k)
            obj.k = k;
        end

        function ref_state_size = get_ref_state_size(obj)
            ref_state_size = obj.k.get_size();
        end

        function ref_state = build_ref_state(~, ref_states, ~, ~, time_step)
            ref_state = ref_states(:, time_step);
        end

    end
end
