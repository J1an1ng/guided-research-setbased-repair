classdef KSTInpTrajTargetLineGenerator < AbstractInpTrajGenerator
    %% Samples trajectories that reach points equally distributed on a
    %% halfcircle in front of the ego vehicle.
    properties (Access = private)
        k AbstractKSTStateOrder
        i AbstractInpOrder
        params KSTParams
        constraints AbstractConstraints
        inp_creator AbstractInpCreator

        dyn

        primitive_duration double

        accelerations cell
        target_point_distance double

        lookahead_time_for_ref_traj double
        lookahead_distance_for_ref_traj double
    end

    methods (Access = private)

        function x = simulate(obj, initial_state, acc, direction)
            tspan = [0 obj.primitive_duration];

            function u = inp(x)
                u = obj.inp_creator.create_max_inp(acc, direction);
                x = obj.constraints.state_bounds.to_bounds(x);
                u = obj.constraints.repair_inp(x, u);
            end

            fun = @(~, x) obj.dyn(x, inp(x), obj.k, obj.i, obj.params);
            [~, x] = ode45(fun, tspan, initial_state);
            x = x(end, :)';
        end

        function trajectories = generate_for_a(obj, initial_state, acc)
            pos_idcs = [obj.k.X obj.k.Y];

            pos = initial_state(pos_idcs);

            x_left = obj.simulate(initial_state, acc, 1);
            x_straight = obj.simulate(initial_state, acc, 0);
            x_right = obj.simulate(initial_state, acc, -1);

            pos_left = x_left(pos_idcs);
            pos_straight = x_straight(pos_idcs);
            pos_right = x_right(pos_idcs);

            target_vectors_to_left = obj.generate_for_segment(pos, pos_straight, pos_left, true);
            target_vectors_to_right = obj.generate_for_segment(pos, pos_straight, pos_right, false);
            all_target_vectors = [target_vectors_to_left; {pos_straight - pos}; target_vectors_to_right];

            trajectories = cellfun(@(target_vector) KSTInpTrajTargetLine(obj.k, ...
                obj.params, obj.constraints, acc, pos, target_vector, ...
                obj.lookahead_time_for_ref_traj, obj.lookahead_distance_for_ref_traj, obj.inp_creator), ...
                all_target_vectors, 'UniformOutput', false);
        end

        function target_vectors = generate_for_segment(obj, start_pos, target_pos_straight, target_pos_to_side, rot_left)
            if rot_left
                rot_factor = 1;
            else
                rot_factor = -1;
            end

            vector_straight = target_pos_straight - start_pos;
            vector_to_side = target_pos_to_side - start_pos;

            radius_avg = mean([norm(vector_straight) norm(vector_to_side)]);

            angle = angle_between(vector_straight, vector_to_side);
            circumference = angle * radius_avg;

            num_points = ceil(circumference / obj.target_point_distance);

            if num_points == 0
                target_vectors = cell(0, 1);
                return
            end

            total_angle = angle_between(vector_straight, vector_to_side);

            angle_step = total_angle / double(num_points);
            angles = angle_step:angle_step:total_angle;

            target_vectors = cell(num_points, 1);

            for j = 1:num_points
                angle = rot_factor * angles(j);
                m = rot(angle);
                target_vector = m * vector_straight;
                target_vectors{j} = target_vector;
            end
        end

    end

    methods

        function obj = KSTInpTrajTargetLineGenerator( ...
                state_order, inp_order, params, ...
                constraints, inp_creator, ...
                model, ...
                primitive_duration, accelerations, ...
                target_point_distance, lookahead_time_for_ref_traj, lookahead_distance_for_ref_traj)

            obj.k = state_order;
            obj.i = inp_order;
            obj.params = params;
            obj.constraints = constraints;
            obj.inp_creator = inp_creator;

            obj.dyn = model;
            obj.primitive_duration = primitive_duration;

            obj.accelerations = accelerations;
            obj.target_point_distance = target_point_distance;
            obj.lookahead_time_for_ref_traj = lookahead_time_for_ref_traj;
            obj.lookahead_distance_for_ref_traj = lookahead_distance_for_ref_traj;
        end

        function trajectories = generate(obj, initial_state)
            trajectories = {};

            for i_a = 1:length(obj.accelerations)
                acc = obj.accelerations{i_a};
                new_trajs = obj.generate_for_a(initial_state, acc);
                trajectories = [trajectories; new_trajs]; %#ok<AGROW>
            end
        end

    end
end
