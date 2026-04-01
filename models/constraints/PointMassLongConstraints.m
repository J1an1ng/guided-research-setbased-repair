classdef PointMassLongConstraints < AbstractConstraints
    properties
        state_bounds Bounds
        inp_bounds Bounds
        a_min_total_squared double
    end

    methods

        function obj = PointMassLongConstraints(state_order, inp_order, params)
            relevant_dims = [state_order.V];
            obj = obj@AbstractConstraints(state_order, inp_order, relevant_dims, params);

            obj.state_bounds = create_point_mass_long_state_bounds(obj.k, obj.params);
            obj.inp_bounds = create_inp_bounds_for_a(obj.i, obj.params);

            obj.a_min_total_squared = power(obj.params.a_min_total, 2);
        end

        function itv = repair_interval(~, itv)
            % a state interval for the vehicle model point mass long is not repairable as the friction would
            % only be violated by the input acceleration
        end

        function res = check_state_bounds(obj, x)
            res = obj.state_bounds.contains(x);
        end

        function res = check_inp_bounds(obj, ~, u)
            res = obj.inp_bounds.contains(u);
        end

        function valid = check_constraints(obj, x, u)
            valid = true;
            %% Overall force of truck
            lon_acc_squared = power(u(obj.i.A), 2);
            if lon_acc_squared > obj.a_min_total_squared
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
        end

    end
end
