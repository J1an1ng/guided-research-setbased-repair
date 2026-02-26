classdef KSTInpTrajTargetPoint < AbstractInpTraj
    properties (Constant)
        id_properties = ["a_profile"; "target_pos"; "is_terminal"]

        min_dist_to_ref_point = 1
    end

    properties (SetAccess = private, GetAccess = public)
        k AbstractKSTStateOrder
        params
        constraints AbstractConstraints
        a_profile (1, :) double
        a_fun function_handle
        target_pos (:, 1) double

        inp_creator AbstractInpCreator
    end

    methods

        function obj = KSTInpTrajTargetPoint(state_order, params, ...
                constraints, a_profile, a_fun, target_pos, inp_creator)
            obj.k = state_order;
            obj.params = params;
            obj.constraints = constraints;
            obj.a_profile = a_profile;
            obj.a_fun = a_fun;
            obj.target_pos = target_pos;
            obj.inp_creator = inp_creator;
        end

        function inp = inp_at(obj, state, t, dt)
            if norm(state([obj.k.X obj.k.Y]) - obj.target_pos) <= obj.min_dist_to_ref_point
                warning("The current position is close to the reference position.");
            end

            v = state(obj.k.V);
            inp_a = obj.a_fun(t);
            inp_a_stop = -v / dt;
            inp_a = max(inp_a, inp_a_stop);

            %% TODO maybe decrease time constant over time t
            desired_delta = lat_pure_pursuit(state, obj.target_pos, obj.k, obj.params);

            inp = obj.inp_creator.create_inp(state, inp_a, desired_delta);

            %% Hint: we cannot entirely ensure that the pure-pursuit steering angle is within the allowed bounds
            %% (i.e. the overall bounds and the bounds from the friction circle), even though the way the ref trajs
            %% are generated already considers the bounds to some extent. To ensure that the input is within the bounds,
            %% we need to repair the it:
            inp = obj.constraints.repair_inp(state, inp);
        end

    end
end
