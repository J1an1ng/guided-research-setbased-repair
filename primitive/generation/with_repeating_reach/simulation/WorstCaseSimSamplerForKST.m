classdef WorstCaseSimSamplerForKST < SimulationSampler
    properties (Access = private)
        k AbstractKSTStateOrder
        inp_order AbstractInpOrder

        meas_dist_set
        model_add_dist_set
        inp_dist_set
        lateral_state_dims (1, :) uint32
        long_state_dims (1, :) uint32
        remaining_state_dims (1, :) uint32

        lateral_inp_dims (1, :) uint32
        long_inp_dims (1, :) uint32
        remaining_inp_dims (1, :) uint32
    end

    methods

        function obj = WorstCaseSimSamplerForKST(k, i, meas_dist_set, model_add_dist_set, inp_dist_set, ...
                lateral_state_dims, long_state_dims, lateral_inp_dims, long_inp_dims)
            obj.k = k;
            obj.inp_order = i;
            obj.meas_dist_set = meas_dist_set;
            obj.model_add_dist_set = model_add_dist_set;
            obj.inp_dist_set = inp_dist_set;
            obj.lateral_state_dims = lateral_state_dims;
            obj.long_state_dims = long_state_dims;
            obj.remaining_state_dims = setdiff(1:obj.k.SIZE, [obj.lateral_state_dims, obj.long_state_dims]);

            obj.lateral_inp_dims = lateral_inp_dims;
            obj.long_inp_dims = long_inp_dims;
            obj.remaining_inp_dims = setdiff(1:obj.inp_order.SIZE, [obj.lateral_inp_dims, obj.long_inp_dims]);

        end

        function x = sample_initial_state(obj, initial_set)
            sign_vector = obj.get_sign_vector_for_state(random_sign(1), random_sign(1));
            x = extreme_point(initial_set, sign_vector);
        end

        function res = sample_meas_dist(obj, state, ref_state)
            % we choose the measurement disturbance such that the controller can only poorly
            % counteract
            lat_sign = -obj.get_lateral_scale(state, ref_state);
            lon_sign = -obj.get_long_scale(state, ref_state);
            sign_vector = obj.get_sign_vector_for_state(lat_sign, lon_sign);
            res = extreme_point(obj.meas_dist_set, sign_vector);
        end

        function res = sample_model_add_dist(obj, state, ref_state)
            lat_sign = obj.get_lateral_scale(state, ref_state);
            lon_sign = obj.get_long_scale(state, ref_state);
            sign_vector = obj.get_sign_vector_for_state(lat_sign, lon_sign);
            res = extreme_point(obj.model_add_dist_set, sign_vector);
        end

        function res = sample_inp_dist(obj, state, ref_state)
            lat_sign = obj.get_lateral_scale(state, ref_state);
            lon_sign = obj.get_long_scale(state, ref_state);
            sign_vector = obj.get_sign_vector_for_inp(lat_sign, lon_sign);
            res = extreme_point(obj.inp_dist_set, sign_vector);
        end

    end

    methods (Access = private)

        function sign_vector = get_sign_vector_for_state(obj, lat_sign, lon_sign)
            sign_vector = zeros(obj.k.SIZE, 1);
            sign_vector(obj.lateral_state_dims) = lat_sign;
            sign_vector(obj.long_state_dims) = lon_sign;
            sign_vector(obj.remaining_state_dims) = random_sign(length(obj.remaining_state_dims));
        end

        function sign_vector = get_sign_vector_for_inp(obj, lat_sign, lon_sign)
            sign_vector = zeros(obj.inp_order.SIZE, 1);
            sign_vector(obj.lateral_inp_dims) = lat_sign;
            sign_vector(obj.long_inp_dims) = lon_sign;
            sign_vector(obj.remaining_inp_dims) = random_sign(length(obj.remaining_inp_dims));
        end

        function scale = get_lateral_scale(obj, state, ref_state)
            y_diff = state(obj.k.Y) - ref_state(obj.k.Y);
            if y_diff > 0
                scale = 1;
            else
                scale = -1;
            end
        end

        function scale = get_long_scale(obj, state, ref_state)
            x_diff = state(obj.k.X) - ref_state(obj.k.X);
            if x_diff > 0
                scale = 1;
            else
                scale = -1;
            end
        end

    end
end
