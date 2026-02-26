classdef KSTTrailerConstraints < AbstractConstraints
    properties
        kst_constraints KSTConstraints
        state_bounds Bounds
        a_min_lat_trailer_squared double
    end

    methods

        function obj = KSTTrailerConstraints(state_order, inp_order, params)
            relevant_dims = [state_order.DELTA state_order.V state_order.HA];
            obj = obj@AbstractConstraints(state_order, inp_order, relevant_dims, params);

            obj.kst_constraints = KSTConstraints(state_order, inp_order, params);

            obj.state_bounds = create_kst_trailer_state_bounds(obj.k, obj.params);

            obj.a_min_lat_trailer_squared = power(obj.params.a_min_lat_trailer, 2);
        end

        function delta_max = delta_max_for_v(obj, v)
            delta_max = obj.kst_constraints.delta_max_for_v(v);
        end

        function ha_max = ha_max_for(obj, v)
            ha_max = atan2(obj.params.trailer_wheelbase * obj.params.a_min_lat_trailer, v^2);
            ha_max = min(ha_max, obj.params.max_hitch_angle);
        end

        function itv = repair_interval(obj, itv)
            itv = obj.kst_constraints.repair_interval(itv);

            v = max(itv(obj.k.V, :));

            ha_range = itv(obj.k.HA, :);
            ha_lb = ha_range(1);
            ha_ub = ha_range(2);

            delta_range = itv(obj.k.DELTA, :);
            delta_lb = delta_range(1);
            delta_ub = delta_range(2);

            trailer_lat_acc_1 = calc_lat_acc_for_trailer(v, delta_lb, ...
                obj.params.truck_wheelbase, obj.params.truck_dist_from_rear_axle_to_hitch, ...
                obj.params.trailer_wheelbase, ha_lb);
            trailer_lat_acc_2 = calc_lat_acc_for_trailer(v, delta_ub, ...
                obj.params.truck_wheelbase, obj.params.truck_dist_from_rear_axle_to_hitch, ...
                obj.params.trailer_wheelbase, ha_lb);
            trailer_lat_acc_squared = max(power(trailer_lat_acc_1, 2), power(trailer_lat_acc_2, 2));
            if trailer_lat_acc_squared > obj.a_min_lat_trailer_squared
                ha_lb = sign(ha_lb) * obj.ha_max_for(v);
            end

            trailer_lat_acc_1 = calc_lat_acc_for_trailer(v, delta_lb, ...
                obj.params.truck_wheelbase, obj.params.truck_dist_from_rear_axle_to_hitch, ...
                obj.params.trailer_wheelbase, ha_ub);
            trailer_lat_acc_2 = calc_lat_acc_for_trailer(v, delta_ub, ...
                obj.params.truck_wheelbase, obj.params.truck_dist_from_rear_axle_to_hitch, ...
                obj.params.trailer_wheelbase, ha_ub);
            trailer_lat_acc_squared = max(power(trailer_lat_acc_1, 2), power(trailer_lat_acc_2, 2));
            if trailer_lat_acc_squared > obj.a_min_lat_trailer_squared
                ha_ub = sign(ha_ub) * obj.ha_max_for(v);
            end

            itv(obj.k.HA, :) = [ha_lb, ha_ub];
        end

        function res = check_state_bounds(obj, x)
            res = obj.state_bounds.contains(x);
        end

        function res = check_inp_bounds(obj, x, u)
            res = obj.kst_constraints.check_inp_bounds(x, u);
        end

        function valid = check_constraints(obj, x, u)
            valid = obj.kst_constraints.check_constraints(x, u);

            if ~valid
                return
            end

            v = x(obj.k.V);

            %% Lateral force of trailer
            trailer_lat_acc = calc_lat_acc_for_trailer(v, x(obj.k.DELTA), ...
                obj.params.truck_wheelbase, obj.params.truck_dist_from_rear_axle_to_hitch, ...
                obj.params.trailer_wheelbase, x(obj.k.HA));
            trailer_lat_acc_squared = power(trailer_lat_acc, 2);
            if trailer_lat_acc_squared > obj.a_min_lat_trailer_squared
                valid = false;
                return
            end

            %% Overall force of trailer (trailers have their own brakes)
            %% --> we do not consider this anymore, as braking of truck is dominant, and
            %% value for maximum overall trailer acceleration is hard to determine.
            % lon_acc_squared = power(u(obj.i.A), 2);
            % if trailer_lat_acc_squared + lon_acc_squared > obj.kst_constraints.a_min_total_squared
            % throw(MException('KSTConstraints:check_constraints', 'Overall force of truck is too high'));
            %    valid = false;
            %    return
            % end
        end

        function u = repair_inp(obj, x, u)
            u = obj.kst_constraints.repair_inp(x, u);
        end

    end
end
