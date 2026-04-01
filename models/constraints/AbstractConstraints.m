classdef (Abstract) AbstractConstraints < handle & matlab.mixin.Heterogeneous
    properties
        relevant_dims (:, 1) uint32

        k AbstractStateOrder
        i AbstractInpOrder
        params
    end

    methods

        function obj = AbstractConstraints(state_order, inp_order, relevant_dims, params)
            obj.k = state_order;
            obj.i = inp_order;
            obj.relevant_dims = relevant_dims;
            obj.params = params;
        end

        function valid = check_bounds(obj, x, u)
            if ~obj.check_state_bounds(x)
                valid = false;
                return
            end

            if ~obj.check_inp_bounds(x, u)
                valid = false;
                return
            end

            valid = true;
        end

        function valid = check(obj, x, u)
            valid = obj.check_bounds(x, u) && obj.check_constraints(x, u);
        end

        function valid = check_interval(obj, x_interval, u)
            %% Important: we assume that x_interval is an interval on column vectors,
            %% i.e., its bounds were defined as columns, not as rows.
            if ~exist('u', 'var')
                u = zeros(obj.i.SIZE, 1);
            end
            n = length(obj.relevant_dims);
            x_proj = x_interval(obj.relevant_dims, :);
            c = mat2cell(x_proj, ones(1, n), 2);
            comb = combvec(c{:});

            x_bounds = NaN(obj.k.get_size(), 2^n);
            x_bounds(obj.relevant_dims, :) = comb;

            are_valid = arrayfun(@(i) obj.check(x_bounds(:, i), u), 1:size(x_bounds, 2));
            valid = all(are_valid);
        end

    end

    methods (Abstract)
        valid = check_state_bounds(obj, x)

        valid = check_inp_bounds(obj, x, u)

        valid = check_constraints(obj, x, u)

        res = repair_interval(obj, itv)

        u = repair_inp(obj, x, u)
    end
end
