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

        function ref_state = build_ref_state(obj, ref_traj, ref_state_args, time_step)
            arguments
                obj LookaheadRefStateBuilder
                ref_traj AbstractInpTraj
                ref_state_args dictionary
                time_step uint32
            end

            current_ref_state = ref_traj.states(:, time_step);

            %% FIXME place the code below in the corresponding method (to be implemented in KSTInpTrajTargetPoint)
            % num_states = size(ref_traj.states, 2);
            % idx_pp = time_step + uint32(ref_state_args("pure_pursuit_lookahead_num_steps"));

            % if idx_pp <= num_states
            %     lookahead_ref_state = ref_states(:, idx_pp);
            % else
            %     new_idx = min(idx_pp - num_states, size(ext_horizon_ref_states, 2));
            %     lookahead_ref_state = ext_horizon_ref_states(:, new_idx);
            % end

            lookahead_pos = ref_traj.get_target_point_for_time_step(time_step, ref_state_args);

            %% IMPORTANT: this order has to match the one in the controlled model!!
            ref_state = [current_ref_state(obj.current_state_dims); lookahead_pos];
        end

    end
end
