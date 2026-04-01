classdef KSTInpTrajTargetLine < AbstractInpTraj
    properties (Constant)
        id_properties = ["acc"; "direction_vector"; "is_terminal"]
    end

    properties (SetAccess = private, GetAccess = public)
        k AbstractKSTStateOrder
        params
        constraints AbstractConstraints

        acc double
        start_pos (:, 1) double
        direction_vector (:, 1) double

        lookahead_time_for_ref_traj double
        lookahead_distance_for_ref_traj double

        inp_creator AbstractInpCreator
    end

    methods

        function obj = KSTInpTrajTargetLine(state_order, params, constraints, ...
                acc, start_pos, direction_vector, lookahead_time_for_ref_traj, lookahead_distance_for_ref_traj, inp_creator)
            obj.k = state_order;
            obj.params = params;
            obj.constraints = constraints;
            obj.acc = acc;
            obj.start_pos = start_pos;
            obj.direction_vector = direction_vector / norm(direction_vector);
            obj.lookahead_time_for_ref_traj = lookahead_time_for_ref_traj;
            obj.lookahead_distance_for_ref_traj = lookahead_distance_for_ref_traj;
            obj.inp_creator = inp_creator;

            assert (abs(norm(obj.direction_vector) - 1) <= 1e-5, "Direction vector must be normalized");
        end

        function target_point = get_target_point_for_time_step(obj, time_step, ref_state_args)
            if ref_state_args.isKey("lookahead_distance")
                lookahead_distance = ref_state_args("lookahead_distance");
            else
                lookahead_time = ref_state_args("lookahead_time");
                lookahead_distance = lookahead_time * obj.states(obj.k.V, time_step);
            end

            pos = obj.states([obj.k.X obj.k.Y], time_step);
            target_point = obj.get_target_point(pos, lookahead_distance);
        end

        function target_point = get_target_point(obj, pos, lookahead_distance)
            dir_vector_to_pos = pos - obj.start_pos;
            proj_dir_vector = dot(dir_vector_to_pos, obj.direction_vector) * obj.direction_vector;
            total_lookahead_distance = norm(proj_dir_vector) + lookahead_distance;
            target_point = obj.start_pos + total_lookahead_distance * obj.direction_vector;
        end

        function inp = inp_at(obj, state, ~, dt)
            v = state(obj.k.V);
            inp_a_stop = -v / dt;
            inp_a = max(obj.acc, inp_a_stop);

            lookahead_distance = obj.lookahead_distance_for_ref_traj;
            if isempty(lookahead_distance)
                lookahead_distance = v * obj.lookahead_time_for_ref_traj;
            end
            target_pos = obj.get_target_point(state([obj.k.X obj.k.Y]), lookahead_distance);

            desired_delta = lat_pure_pursuit(state, target_pos, obj.k, obj.params);

            inp = obj.inp_creator.create_inp(state, inp_a, desired_delta);

            %% Hint: we cannot entirely ensure that the pure-pursuit steering angle is within the allowed bounds
            %% (i.e. the overall bounds and the bounds from the friction circle), even though the way the ref trajs
            %% are generated already considers the bounds to some extent. To ensure that the input is within the bounds,
            %% we need to repair the it:
            inp = obj.constraints.repair_inp(state, inp);
        end

    end
end
