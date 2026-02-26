classdef LookaheadRefStateBuilder < AbstractRefStateBuilder
    properties (Access = private)
        k AbstractStateOrder

        current_state_dims (:, 1) uint32
        lookahead_state_dims (:, 1) uint32

        ref_state_size uint32
    end

    methods

        function obj = LookaheadRefStateBuilder(k, ...
                current_state_dims, lookahead_state_dims)
            obj.k = k;
            obj.current_state_dims = current_state_dims;
            obj.lookahead_state_dims = lookahead_state_dims;
            obj.ref_state_size = length(obj.current_state_dims) + length(obj.lookahead_state_dims);
        end

        function ref_state_size = get_ref_state_size(obj)
            ref_state_size = obj.ref_state_size;
        end

        function ref_state = build_ref_state(obj, ref_states, ref_state_args, ext_horizon_ref_states, time_step)
            arguments
                obj LookaheadRefStateBuilder
                ref_states (:, :) double
                ref_state_args dictionary
                ext_horizon_ref_states (:, :) double
                time_step uint32
            end

            current_ref_state = ref_states(:, time_step);

            num_states = size(ref_states, 2);
            idx_pp = time_step + uint32(ref_state_args("pure_pursuit_lookahead_num_steps"));

            if idx_pp <= num_states
                lookahead_ref_state = ref_states(:, idx_pp);
            else
                new_idx = min(idx_pp - num_states, size(ext_horizon_ref_states, 2));
                lookahead_ref_state = ext_horizon_ref_states(:, new_idx);
            end

            %% IMPORTANT: this order has to match the one in the controlled model!!
            ref_state = [current_ref_state(obj.current_state_dims)
                         lookahead_ref_state(obj.lookahead_state_dims)];
        end

    end
end
