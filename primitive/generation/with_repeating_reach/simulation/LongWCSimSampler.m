classdef LongWCSimSampler < SimulationSampler
    properties (Access = private)
        dim_s uint32
        dim_v uint32
        dim_a uint32

        meas_dist (:, 1) double
        meas_dist_itv
        model_add_dist (:, 1) double
        model_add_dist_itv
        inp_dist (:, 1) double
        inp_dist_itv
    end

    methods

        function obj = LongWCSimSampler(dim_s, dim_v, dim_a, meas_dist, model_add_dist, inp_dist)
            obj.dim_s = dim_s;
            obj.dim_v = dim_v;
            obj.dim_a = dim_a;
            obj.meas_dist = meas_dist;
            obj.meas_dist_itv = interval(-meas_dist, meas_dist);
            obj.model_add_dist = model_add_dist;
            obj.model_add_dist_itv = interval(-model_add_dist, model_add_dist);
            obj.inp_dist = inp_dist;
            obj.inp_dist_itv = interval(-inp_dist, inp_dist);
        end

        function x = sample_initial_state(obj, initial_set)
            x = randPoint(initial_set, 1, 'extreme'); %% TODO make "extreme" customizable
            sign_s = sign(x(obj.dim_s));

            initial_set_itv = interval(initial_set);
            %% Assumes that initial set is symmetric
            x(obj.dim_s) = sign_s * initial_set_itv.sup(obj.dim_s);
            x(obj.dim_v) = sign_s * initial_set_itv.sup(obj.dim_v);
        end

        function res = sample_meas_dist(obj, state, ref_state)
            res = randPoint(obj.meas_dist_itv, 1, 'extreme');

            scale = -obj.get_scale(state, ref_state);

            % we choose the measurement disturbance such that the controller can only poorly
            % counteract
            res(obj.dim_s) = scale * obj.meas_dist(obj.dim_s);
            res(obj.dim_v) = scale * obj.meas_dist(obj.dim_v);
        end

        function res = sample_model_add_dist(obj, state, ref_state)
            %% FIXME this is untested
            res = randPoint(obj.model_add_dist_itv, 1, 'extreme');
            scale = obj.get_scale(state, ref_state);
            res(obj.dim_s) = scale * obj.model_add_dist(obj.dim_s);
            res(obj.dim_v) = scale * obj.model_add_dist(obj.dim_v);
        end

        function res = sample_inp_dist(obj, state, ref_state)
            res = randPoint(obj.inp_dist_itv, 1, 'extreme');
            scale = obj.get_scale(state, ref_state);
            res(obj.dim_a) = scale * obj.inp_dist(obj.dim_a);
        end

    end

    methods (Access = private)

        function scale = get_scale(obj, state, ref_state)
            s_diff = state(obj.dim_s) - ref_state(obj.dim_s);
            if s_diff > 0
                scale = 1;
            else
                scale = -1;
            end
        end

    end
end
