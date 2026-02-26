classdef KSTInpDeltaConstraints < AbstractConstraints
    properties
        state_bounds Bounds
        inp_bounds Bounds
        a_min_total_squared double
    end

    methods

        function obj = KSTInpDeltaConstraints(state_order, inp_order, params)
            relevant_dims = [state_order.V];
            obj = obj@AbstractConstraints(state_order, inp_order, relevant_dims, params);

            obj.state_bounds = create_kst_inp_delta_state_bounds(obj.k, obj.params);
            obj.inp_bounds = create_inp_bounds_for_a_delta(obj.i, obj.params);

            obj.a_min_total_squared = power(obj.params.a_min_total, 2);
        end

        function delta_max = delta_max_for_v(obj, v)
            a_min_lat = obj.params.get_a_min_lat(v);
            delta_max = atan2(obj.params.truck_wheelbase * a_min_lat, v^2);
            delta_max = min(delta_max, obj.params.steering_max);
        end

        function itv = repair_interval(~, itv)
            % a state interval for the vehicle model kst with input delta is not repairable
            % as the steering angle delta is not a state variable
        end

        function res = check_state_bounds(obj, x)
            res = obj.state_bounds.contains(x);
        end

        function res = check_inp_bounds(obj, ~, u)
            res = obj.inp_bounds.contains(u);
        end

        function valid = check_constraints(obj, x, u)
            valid = true;

            v = x(obj.k.V);

            a_min_lat = obj.params.get_a_min_lat(v);
            a_min_lat_squared = power(a_min_lat, 2);

            %% Lateral force of truck
            lat_acc = calc_lat_acc_for_truck(u(obj.i.DELTA), v, obj.params.truck_wheelbase);
            lat_acc_squared = power(lat_acc, 2);
            if lat_acc_squared > a_min_lat_squared
                valid = false;
                return
            end

            %% Overall force of truck
            lon_acc_squared = power(u(obj.i.A), 2);
            if lat_acc_squared + lon_acc_squared > obj.a_min_total_squared
                % throw(MException('KSTConstraints:check_constraints', 'Overall force of truck is too high'));
                valid = false;
                return
            end
        end

        function u = repair_inp(obj, x, u)
            u = obj.inp_bounds.to_bounds(u);

            if obj.state_bounds.derivative_exceeds_bound(x, obj.k.V, u(obj.i.A))
                u(obj.i.A) = 0;
            end

            delta_max = obj.delta_max_for_v(x(obj.k.V));
            u(obj.i.DELTA) = to_bounds(u(obj.i.DELTA), [-delta_max, delta_max]);

            %% Hint: we do not consider the constraints regarding the trailer angle for input repairing;
            %% they are considered in constraint checking only.
        end

    end
end
