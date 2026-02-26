classdef KSTConstraints < AbstractConstraints
    properties
        state_bounds Bounds
        inp_bounds Bounds
        a_min_total_squared double
    end

    methods

        function obj = KSTConstraints(state_order, inp_order, params)
            relevant_dims = [state_order.DELTA state_order.V];
            obj = obj@AbstractConstraints(state_order, inp_order, relevant_dims, params);

            obj.state_bounds = create_kst_state_bounds(obj.k, obj.params);
            obj.inp_bounds = create_inp_bounds_for_a_deltav(obj.i, obj.params);

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

        function res = check_inp_bounds(obj, x, u)
            res = obj.inp_bounds.contains(u);
            if ~res
                return
            end

            steering_v_max_for_v = obj.get_steering_v_max(x(obj.k.V));
            res = abs(u(obj.i.DELTA_V)) <= steering_v_max_for_v;
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
                % throw(MException('KSTConstraints:check_constraints', 'Overall force of truck is too high'));
                valid = false;
                return
            end
        end

        function steering_v_max = get_steering_v_max(obj, v)
            if isempty(obj.params.max_steering_wheel_rate_range)
                steering_v_max = obj.params.steering_v_max;
                return
            end

            v_in_km_per_hour = v * 3.6;
            max_steering_wheel_rate = interp1(obj.params.v_range, obj.params.max_steering_wheel_rate_range, ...
              v_in_km_per_hour, 'linear');
            assert(~isnan(max_steering_wheel_rate), 'Steering wheel rate not found for v = %d', v);
            factor = 22.2; % from Christian
            steering_v_max = max_steering_wheel_rate / factor;

            % plot(obj.v_range, obj.max_steering_wheel_rate_range);
            % hold on;
            % plot(v_in_km_per_hour, max_steering_wheel_rate);
            % hold off;
            % pause;
        end

        function u = repair_inp(obj, x, u)
            u = obj.inp_bounds.to_bounds(u);

            steering_v_max = obj.get_steering_v_max(x(obj.k.V));
            u(obj.i.DELTA_V) = to_bounds(u(obj.i.DELTA_V), [-steering_v_max, steering_v_max]);

            if obj.state_bounds.derivative_exceeds_bound(x, obj.k.V, u(obj.i.A))
                u(obj.i.A) = 0;
            end

            if obj.state_bounds.derivative_exceeds_bound(x, obj.k.DELTA, u(obj.i.DELTA_V))
                u(obj.i.DELTA_V) = 0;
            end

            delta_max = obj.delta_max_for_v(x(obj.k.V));
            tigher_bound_check_res = check_for_bounds(x(obj.k.DELTA), -delta_max, delta_max);
            if check_if_derivative_exceeds_bounds(tigher_bound_check_res, u(obj.i.DELTA_V))
                u(obj.i.DELTA_V) = 0;
            end
        end

    end
end
