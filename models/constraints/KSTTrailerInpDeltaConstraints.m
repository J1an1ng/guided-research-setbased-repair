classdef KSTTrailerInpDeltaConstraints < AbstractConstraints
    properties
        kst_constraints KSTInpDeltaConstraints
        state_bounds Bounds
        a_min_lat_trailer_squared double
    end

    methods

        function obj = KSTTrailerInpDeltaConstraints(state_order, inp_order, params)
            relevant_dims = [state_order.V state_order.HA];
            obj = obj@AbstractConstraints(state_order, inp_order, relevant_dims, params);

            obj.kst_constraints = KSTInpDeltaConstraints(state_order, inp_order, params);

            obj.state_bounds = create_kst_trailer_inp_delta_state_bounds(obj.k, obj.params);

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

            delta = 0;

            trailer_lat_acc = calc_lat_acc_for_trailer(v, delta, obj.params.truck_wheelbase, ...
                obj.params.truck_dist_from_rear_axle_to_hitch, obj.params.trailer_wheelbase, ha_lb);
            trailer_lat_acc_squared = power(trailer_lat_acc, 2);
            if trailer_lat_acc_squared > obj.a_min_lat_trailer_squared
                ha_lb = sign(ha_lb) * obj.ha_max_for(v);
            end

            trailer_lat_acc = calc_lat_acc_for_trailer(v, delta, obj.params.truck_wheelbase, ...
                obj.params.truck_dist_from_rear_axle_to_hitch, obj.params.trailer_wheelbase, ha_ub);
            trailer_lat_acc_squared = power(trailer_lat_acc, 2);
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

            %% Lateral force of trailer (no overall force needed as trailer
            %% is not separately accelerated)
            trailer_lat_acc = calc_lat_acc_for_trailer(v, u(obj.i.DELTA), obj.params.truck_wheelbase, ...
                obj.params.truck_dist_from_rear_axle_to_hitch, obj.params.trailer_wheelbase, x(obj.k.HA));
            trailer_lat_acc_squared = power(trailer_lat_acc, 2);
            if trailer_lat_acc_squared > obj.a_min_lat_trailer_squared
                valid = false;
                return
            end
        end

        function u = repair_inp(obj, x, u)
            u = obj.kst_constraints.repair_inp(x, u);
        end

    end
end
