classdef RestrictiveKSTTrailerConstraints < KSTTrailerConstraints
    properties
        m_delta_fun double
        delta_max double

        m_ha_fun double
        ha_max double
    end

    methods

        function obj = RestrictiveKSTTrailerConstraints(state_order, inp_order, ...
                params, delta_max_at_max_speed, ha_max_at_max_speed)
            obj = obj@KSTTrailerConstraints(state_order, inp_order, params);

            v_max = obj.state_bounds.ub(obj.k.V);

            obj.delta_max = obj.state_bounds.ub(obj.k.DELTA);
            obj.m_delta_fun = (delta_max_at_max_speed - obj.delta_max) / v_max;

            obj.ha_max = obj.state_bounds.ub(obj.k.HA);
            obj.m_ha_fun = (ha_max_at_max_speed - obj.ha_max) / v_max;
        end

        function valid = check_constraints(obj, x, u)
            valid = abs(x(obj.k.DELTA)) <= obj.get_max_delta(x);
            if ~valid
                return
            end

            valid = abs(x(obj.k.HA)) <= obj.get_max_ha(x);

            if ~valid
                return
            end

            valid = check_constraints@KSTTrailerConstraints(obj, x, u);
        end

    end

    methods (Access = private)

        function res = get_max_delta(obj, x)
            res = lin_fun(obj.m_delta_fun, x(obj.k.V), obj.delta_max);
        end

        function res = get_max_ha(obj, x)
            res = lin_fun(obj.m_ha_fun, x(obj.k.V), obj.ha_max);
        end

    end
end

function res = lin_fun(m, x, t)
    res = m * x + t;
end
