classdef KSTConstantInpTrajJDeltaV < AbstractInpTraj
    properties (Constant)
        id_properties = ["jerk_profile"; "jerk_profile_dt"
                         "start_delta_v"; "gradient_delta_v"]
    end

    properties (SetAccess = private, GetAccess = public)
        state_order AbstractKSTTrailerWithAccStateOrder
        inp_order AbstractInpOrderJDeltaV

        jerk_profile (1, :) double {mustBeNegative}
        jerk_profile_dt double {mustBePositive}
        start_delta_v double
        gradient_delta_v double

        params AbstractKSTTrailerParams
    end

    methods

        function obj = KSTConstantInpTrajJDeltaV(state_order, inp_order, ...
                jerk_profile, jerk_profile_dt, ...
                start_delta_v, gradient_delta_v, params)
            obj.state_order = state_order;
            obj.inp_order = inp_order;
            obj.jerk_profile = jerk_profile;
            obj.jerk_profile_dt = jerk_profile_dt;
            obj.start_delta_v = start_delta_v;
            obj.gradient_delta_v = gradient_delta_v;
            obj.params = params;
        end

        function inp = inp_at(obj, state, t, dt)
            k = obj.state_order;

            v = state(k.V);

            if v <= 0
                %% the trajectory stops if vehicle stops
                inp = [];
                return
            end

            delta_v = obj.start_delta_v + obj.gradient_delta_v * t;
            delta_v = to_bounds(delta_v, [obj.params.steering_v_min obj.params.steering_v_max]);

            j_idx = floor(t / obj.jerk_profile_dt) + 1;
            j_idx = min(j_idx, size(obj.jerk_profile, 2));
            j = obj.jerk_profile(j_idx);

            j_to_min_a = (obj.params.a_min - state(k.A)) / dt;
            j = max(j, j_to_min_a);

            inp = zeros(obj.inp_order.SIZE, 1);
            inp(obj.inp_order.DELTA_V) = delta_v;
            inp(obj.inp_order.J) = j;
        end

        function str = to_string(obj)
            str = sprintf('dv: %0.5g, dv_derivative: %0.5g, jerk: %s', ...
                obj.start_steering_angle_speed, ...
                obj.gradient_steering_angle_speed, mat2str(obj.jerk_profile));
        end

    end
end
