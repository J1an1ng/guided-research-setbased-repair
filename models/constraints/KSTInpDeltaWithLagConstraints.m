%% 修改过
classdef KSTInpDeltaWithLagConstraints < AbstractConstraints
    properties
        state_bounds Bounds
        inp_bounds Bounds
        a_min_total_squared double
    end

    methods

        function obj = KSTInpDeltaWithLagConstraints(state_order, inp_order, params)
            relevant_dims = [state_order.DELTA state_order.V];
            obj = obj@AbstractConstraints(state_order, inp_order, relevant_dims, params);

            obj.state_bounds = create_kst_state_bounds(obj.k, obj.params);
            obj.inp_bounds = create_inp_bounds_for_a_delta(obj.i, obj.params);

            obj.a_min_total_squared = power(obj.params.a_min_total, 2);
        end

        function delta_max = delta_max_for_v(obj, v)
            a_min_lat = obj.params.get_a_min_lat(v);
            delta_max = atan2(obj.params.truck_wheelbase * a_min_lat, v^2);
            delta_max = min(delta_max, obj.params.steering_max);
        end

        function itv = repair_interval(obj, itv)
            v = max(itv(obj.k.V, :));

            delta_range = itv(obj.k.DELTA, :);
            delta_lb = delta_range(1);
            delta_ub = delta_range(2);

            a_min_lat = obj.params.get_a_min_lat(v);
            a_min_lat_squared = power(a_min_lat, 2);

            lat_acc = calc_lat_acc_for_truck(delta_lb, v, obj.params.truck_wheelbase);
            lat_acc_squared = power(lat_acc, 2);
            if lat_acc_squared > a_min_lat_squared
                delta_lb = sign(delta_lb) * obj.delta_max_for_v(v);
            end

            lat_acc = calc_lat_acc_for_truck(delta_ub, v, obj.params.truck_wheelbase);
            lat_acc_squared = power(lat_acc, 2);
            if lat_acc_squared > a_min_lat_squared
                delta_ub = sign(delta_ub) * obj.delta_max_for_v(v);
            end

            itv(obj.k.DELTA, :) = [delta_lb, delta_ub];
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
            lat_acc = calc_lat_acc_for_truck(x(obj.k.DELTA), v, obj.params.truck_wheelbase);
            lat_acc_squared = power(lat_acc, 2);
            if lat_acc_squared > a_min_lat_squared
                valid = false;
                return
            end

            %% Overall force of truck
            lon_acc_squared = power(u(obj.i.A), 2);
            if lat_acc_squared + lon_acc_squared > obj.a_min_total_squared
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
        end

    end
end
