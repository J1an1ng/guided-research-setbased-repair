classdef KSTPlotter < VehiclePlotter
    properties (Access = private)
        params KSTParams
    end

    methods

        function obj = KSTPlotter(occ_computer, k, params)
            obj = obj@VehiclePlotter(occ_computer, k);
            obj.params = params;
        end

        function res = plot_vehicle(obj, state, color, ax)
            res = plot_vehicle@VehiclePlotter(obj, state, color, ax);

            wheel_left = obj.get_wheel(1, state);
            wheel_right = obj.get_wheel(-1, state);
            p1 = plot(wheel_left, obj.pos_dims, 'Color', color, 'Parent', ax);
            hold on;
            p2 = plot(wheel_right, obj.pos_dims, 'Color', color, 'Parent', ax);

            kingpin_pos = obj.get_kingpin_pos(state);
            p3 = plot(kingpin_pos(1), kingpin_pos(2), '.', 'Color', 'green', 'MarkerSize', 20, 'Parent', ax);

            rear_axle = obj.get_rear_axle_pos(state);
            p4 = plot(rear_axle(1, :), rear_axle(2, :), 'Color', 'blue', 'Parent', ax);

            res = [res; {p1; p2; p3; p4}];
        end

    end

    methods (Access = private)

        function res = get_wheel(obj, factor, state)
            wheel_length = 1;
            wheel_width = 0.3;

            wheel_shape = zonotope(interval( ...
                [-wheel_length / 2; -wheel_width / 2], ...
                [wheel_length / 2; wheel_width / 2]));

            if isprop(obj.k, 'DELTA')
                delta = state(obj.k.DELTA);
            else
                delta = 0;
            end
            psi = state(obj.k.PSI);

            rotation_delta = rot(delta);
            offset_1 = [obj.params.truck_wheelbase; factor * obj.params.truck_width / 2];
            offset_2 = state(obj.pos_dims);
            rotation_psi = rot(psi);

            wheel_shape_transformed = rotation_psi * (rotation_delta * ...
                wheel_shape + offset_1) + offset_2;

            V = vertices(wheel_shape_transformed);
            res = polygon(V(1, :)', V(2, :)');
        end

        function rear_axle = get_rear_axle_pos(obj, state)
            pos_min_relative = [0; -obj.params.truck_width / 2];
            pos_max_relative = [0; obj.params.truck_width / 2];

            rot_psi = rot(state(obj.k.PSI));

            pos_min_abs = state(obj.pos_dims) + rot_psi * pos_min_relative;
            pos_max_abs = state(obj.pos_dims) + rot_psi * pos_max_relative;

            rear_axle = [pos_min_abs pos_max_abs];
        end

        function kingpin_pos = get_kingpin_pos(obj, state)
            kingpin_pos_relative = [obj.params.truck_dist_from_rear_axle_to_hitch; 0];
            kingpin_pos = state(obj.pos_dims) + rot(state(obj.k.PSI)) * kingpin_pos_relative;
        end

    end
end
