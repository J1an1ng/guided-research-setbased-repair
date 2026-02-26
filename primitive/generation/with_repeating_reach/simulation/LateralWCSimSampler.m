classdef LateralWCSimSampler < SimulationSampler
    properties (Access = private)
        k KSTStateOrder
        i AbstractInpOrder

        meas_dist (:, 1) double
        meas_dist_itv
        model_add_dist (:, 1) double
        model_add_dist_itv
        inp_dist (:, 1) double
        inp_dist_itv
    end

    methods

        function obj = LateralWCSimSampler(k, i, meas_dist, model_add_dist, inp_dist)
            obj.k = k;
            obj.i = i;
            obj.meas_dist = meas_dist;
            obj.meas_dist_itv = interval(-meas_dist, meas_dist);
            obj.model_add_dist = model_add_dist;
            obj.model_add_dist_itv = interval(-model_add_dist, model_add_dist);
            if isempty(inp_dist)
                inp_dist = zeros(i.SIZE, 1);
            end
            obj.inp_dist = inp_dist;
            obj.inp_dist_itv = interval(-inp_dist, inp_dist);
        end

        function x = sample_initial_state(obj, initial_set)
            x = randPoint(initial_set, 1, 'extreme'); %% TODO make "extreme" customizable
            sign_x = sign(x(obj.k.X));
            sign_y = sign(x(obj.k.Y));

            initial_set_itv = interval(initial_set);

            x(obj.k.X) = sign_x * initial_set_itv.sup(obj.k.X);
            if sign_x < 0
                x(obj.k.V) = initial_set_itv.inf(obj.k.V);
            else
                x(obj.k.V) = initial_set_itv.sup(obj.k.V);
            end
            %% Assumes that initial set is symmetric
            x(obj.k.Y) = sign_y * initial_set_itv.sup(obj.k.Y);
            x(obj.k.DELTA) = sign_y * initial_set_itv.sup(obj.k.DELTA);
            x(obj.k.PSI) = sign_y * initial_set_itv.sup(obj.k.PSI);
        end

        function res = sample_meas_dist(obj, state, ref_state)
            res = randPoint(obj.meas_dist_itv, 1, 'extreme');

            scale = -obj.get_scale(state, ref_state);

            % we choose the measurement disturbance such that the controller can only poorly
            % counteract
            res(obj.k.Y) = scale * obj.meas_dist(obj.k.Y);
            res(obj.k.DELTA) = scale * obj.meas_dist(obj.k.DELTA);
            res(obj.k.PSI) = scale * obj.meas_dist(obj.k.PSI);
        end

        function res = sample_model_add_dist(obj, state, ref_state)
            %% FIXME this is untested
            res = randPoint(obj.model_add_dist_itv, 1, 'extreme');
            scale = obj.get_scale(state, ref_state);
            res(obj.k.Y) = scale * obj.model_add_dist(obj.k.Y);
            res(obj.k.DELTA) = scale * obj.model_add_dist(obj.k.DELTA);
            res(obj.k.PSI) = scale * obj.model_add_dist(obj.k.PSI);
        end

        function res = sample_inp_dist(obj, state, ref_state)
            res = randPoint(obj.inp_dist_itv, 1, 'extreme');
            scale = obj.get_scale(state, ref_state);
            if isprop(obj.i, 'DELTA')
                res(obj.i.DELTA) = scale * obj.inp_dist(obj.i.DELTA);
            else
                res(obj.i.DELTA_V) = scale * obj.inp_dist(obj.i.DELTA_V);
            end
        end

    end

    methods (Access = private)

        function scale = get_scale(obj, state, ref_state)
            y_diff = state(obj.k.Y) - ref_state(obj.k.Y);
            if y_diff > 0
                scale = 1;
            else
                scale = -1;
            end
        end

    end
end
