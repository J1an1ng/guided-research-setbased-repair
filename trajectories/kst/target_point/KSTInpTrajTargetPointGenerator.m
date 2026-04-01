classdef KSTInpTrajTargetPointGenerator < AbstractInpTrajGenerator
    %% Samples trajectories that reach points equally distributed on a
    %% halfcircle in front of the ego vehicle.
    properties (Access = private)
        k AbstractKSTStateOrder
        i AbstractInpOrder
        params KSTParams
        constraints AbstractConstraints
        inp_creator AbstractInpCreator

        dyn

        desired_duration double

        accelerations cell
        target_point_distance double
        generate_traj_to_left_only logical
    end

    methods (Access = private)

        function x = simulate(obj, initial_state, a_fun, direction)
            tspan = [0 obj.desired_duration];

            function u = inp(x, t)
                a = a_fun(t);
                u = obj.inp_creator.create_max_inp(a, direction);
                x = obj.constraints.state_bounds.to_bounds(x);
                u = obj.constraints.repair_inp(x, u);
            end

            fun = @(t, x) obj.dyn(x, inp(x, t), obj.k, obj.i, obj.params);
            [~, x] = ode45(fun, tspan, initial_state);
            x = x(end, :)';
        end

        function trajectories = generate_for_a(obj, initial_state, acc_profile)
            a_fun = create_acc_function(acc_profile, obj.desired_duration);

            x_left = obj.simulate(initial_state, a_fun, 1);
            x_straight = obj.simulate(initial_state, a_fun, 0);
            x_right = obj.simulate(initial_state, a_fun, -1);

            pos_idcs = [obj.k.X obj.k.Y];
            pos = initial_state(pos_idcs);

            pos_left = x_left(pos_idcs);
            pos_straight = x_straight(pos_idcs);
            pos_right = x_right(pos_idcs);

            %% for pure pursuit, the target point must be in front of the
            %% front axle, thus we add this offset
            avg_a = mean(acc_profile);
            v_end = initial_state(obj.k.V) + obj.desired_duration * avg_a;
            add_offset = v_end * 0.1;
            offset = [obj.params.truck_wheelbase + add_offset; 0];
            %% FIXME
            pos_left_ext = pos_left; % + rot(x_left(obj.k.PSI)) * offset;
            pos_straight_ext = pos_straight; % + rot(x_straight(obj.k.PSI)) * offset;
            pos_right_ext = pos_right; % + rot(x_right(obj.k.PSI)) * offset;

            trajs_left = obj.generate_for_segment(acc_profile, a_fun, ...
                pos, pos_straight, pos_left, true, pos_straight_ext, pos_left_ext);

            target_vector = pos_straight_ext - pos;
            target_vector = to_length(target_vector, 1.5 * norm(target_vector));
            target_pos = pos + target_vector;
            traj_straight = KSTInpTrajTargetPoint(obj.k, ...
                obj.params, obj.constraints, acc_profile, a_fun, target_pos, obj.inp_creator);
            if ~obj.generate_traj_to_left_only
                trajs_right = obj.generate_for_segment(acc_profile, a_fun, ...
                    pos, pos_straight, pos_right, false, pos_straight_ext, pos_right_ext);
            else
                trajs_right = {};
            end
            trajectories = [trajs_left; {traj_straight}; trajs_right];
        end

        function trajectories = generate_for_segment(obj, a_profile, a_fun, ...
                pos, pos_straight, pos_other, rot_left, pos_ext_straight, pos_ext_other)
            if rot_left
                rot_factor = 1;
            else
                rot_factor = -1;
            end

            vector_straight = pos_straight - pos;
            vector_other = pos_other - pos;

            radius_avg = mean([norm(vector_straight) norm(vector_other)]);

            angle = angle_between(vector_straight, vector_other);
            circumference = angle * radius_avg;

            num_points = ceil(circumference / obj.target_point_distance);

            if num_points == 0
                trajectories = cell(0, 1);
                return
            end

            vector_straight_ext = pos_ext_straight - pos;
            vector_other_ext = pos_ext_other - pos;
            angle_ext = angle_between(vector_straight_ext, vector_other_ext);

            angle_step = angle_ext / double(num_points);
            angles = angle_step:angle_step:angle_ext;

            radius_straight_ext = norm(vector_straight_ext);
            radius_other_ext = norm(vector_other_ext);
            radius_avg_ext = mean([radius_straight_ext radius_other_ext]);
            radius_step_ext = (radius_other_ext - radius_straight_ext) / double(num_points);
            radii = radius_straight_ext + radius_step_ext:radius_step_ext:radius_other_ext;

            if isempty(radii)
                radii = zeros(num_points, 1) + radius_avg_ext;
            end

            trajectories = cell(num_points, 1);

            for j = 1:num_points
                angle = rot_factor * angles(j);
                radius = radii(j);

                m = rot(angle);
                target_vector = m * vector_straight_ext;
                %% FIXME
                target_vector = to_length(target_vector, 1.5 * radius);
                target_pos = pos + target_vector;

                trajectories{j} = KSTInpTrajTargetPoint(obj.k, ...
                    obj.params, obj.constraints, a_profile, a_fun, target_pos, ...
                    obj.inp_creator);
            end
        end

    end

    methods

        function obj = KSTInpTrajTargetPointGenerator( ...
                state_order, inp_order, params, ...
                constraints, inp_creator, ...
                model, ...
                desired_duration, accelerations, ...
                target_point_distance, generate_traj_to_left_only)

            obj.k = state_order;
            obj.i = inp_order;
            obj.params = params;
            obj.constraints = constraints;
            obj.inp_creator = inp_creator;

            obj.dyn = model;
            obj.desired_duration = desired_duration;

            obj.accelerations = accelerations;
            obj.target_point_distance = target_point_distance;
            obj.generate_traj_to_left_only = generate_traj_to_left_only;
        end

        function trajectories = generate(obj, initial_state)
            trajectories = {};

            for i_a = 1:length(obj.accelerations)
                acc_profile = obj.accelerations{i_a};
                new_trajs = obj.generate_for_a(initial_state, acc_profile);
                trajectories = [trajectories; new_trajs]; %#ok<AGROW>
            end
        end

    end
end

function fun = create_acc_function(acc_profile, duration)
    dt_per_acc = duration / length(acc_profile);

    function a = get_acc(t)
        idx = min(floor(t / dt_per_acc) + 1, length(acc_profile));
        a = acc_profile(idx);
    end

    fun = @get_acc;
end
