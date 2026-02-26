classdef Bounds < handle
    properties (SetAccess = private, GetAccess = public)
        dim_log_idx (1, :) logical
        lb (:, 1) double
        ub (:, 1) double
        % this is actually redundant but we store it for convenience:
        lb_proj (:, 1) double
        ub_proj (:, 1) double
    end

    methods

        function obj = Bounds(lb, ub)
            dim_log_idx = ~isnan(lb);
            assert(all(dim_log_idx == ~isnan(ub)));

            obj.dim_log_idx = dim_log_idx;

            obj.lb = lb;
            obj.ub = ub;

            TOLERANCE = 0.00001;
            obj.lb_proj = obj.lb(dim_log_idx) - TOLERANCE;
            obj.ub_proj = obj.ub(dim_log_idx) + TOLERANCE;
        end

        function res = contains(obj, v)
            v = v(obj.dim_log_idx);
            res = all(obj.lb_proj <= v) && all(v <= obj.ub_proj);
        end

        function res = get_relative_bound_violations(obj, v)
            res = zeros(length(v), 1);
            v = v(obj.dim_log_idx);
            % We compute the relative violation of the bounds for each dimension (positive value if violated)...
            relative_violations = max(0, max(obj.lb_proj - v, v - obj.ub_proj)) ./ (obj.ub_proj - obj.lb_proj);
            % ... but the resulting violation should be negative if the value is below the lower bound
            % and positive if above the upper bound:
            relative_violations = sign(v - obj.lb_proj) .* relative_violations;
            res(obj.dim_log_idx) = relative_violations;
        end

        function res = check_dim(obj, v, dim)
            if ~obj.dim_log_idx(dim)
                res = 0;
                return
            end

            res = check_for_bounds(v(dim), obj.lb(dim), obj.ub(dim));
        end

        function res = derivative_exceeds_bound(obj, v, dim, derivative)
            bound_check_res = obj.check_dim(v, dim);
            res = check_if_derivative_exceeds_bounds(bound_check_res, derivative);
        end

        function res = check_each_dim(obj, v)
            %% returns vector containing -1, 1, or 0 at dim i \in dims:
            % --> -1 if v(i) < lb
            % --> 1 if v(i) > ub
            % --> 0 otw
            res = zeros(length(v), 1);

            v = v(obj.dim_log_idx);
            below = -(v < obj.lb); % -1 if dim is below lb
            above = obj.ub < v; % 1 if dim is above ub
            % we assume that v is not below lb AND above ub at the same time:
            overall = below + above;
            res(obj.dim_log_idx) = overall;

            % dims_in = obj.dims_with_bounds(find(is_within));
            % dims_out = obj.dims_with_bounds(find(~is_within));
        end

        function res = to_bounds(obj, values)
            res = values;
            res(obj.dim_log_idx) = max(min(values(obj.dim_log_idx), obj.ub(obj.dim_log_idx)), obj.lb(obj.dim_log_idx));
        end

    end
end
