classdef WorstCaseKSTInpDetlaForYSimulator < WorstCaseSimulator
    properties (Access = private)
        k KSTInpDeltaStateOrder
        i InpOrderADelta
    end

    methods

        function obj = WorstCaseKSTInpDetlaForYSimulator(num_sims, dyn, param_conf, dyn_conf_values, meas_dist, model_add_dist, inp_dist, k, i)
            obj@WorstCaseSimulator(num_sims, dyn, param_conf, dyn_conf_values, meas_dist, model_add_dist, inp_dist);
            obj.k = k;
            obj.i = i;
        end

    end

    methods (Access = protected)

        function x = get_initial_state(obj, x, initial_set)
            sign_y = sign(x(obj.k.Y));

            x(obj.k.Y) = sign_y * initial_set.sup(obj.k.Y);
            x(obj.k.PSI) = sign_y * initial_set.sup(obj.k.PSI);
        end

        function res = get_meas_dist(obj, state, ref_state)
            res = randPoint(obj.meas_dist_itv, 1, 'extreme');

            y_diff = state(obj.k.Y) - ref_state(obj.k.Y);

            if y_diff > 0
                scale = -1;
            else
                scale = 1;
            end

            % we choose the measurement disturbance such that the controller can only poorly
            % counteract
            res(obj.k.Y) = scale * obj.meas_dist(obj.k.Y);
            res(obj.k.PSI) = scale * obj.meas_dist(obj.k.PSI);
        end

        function res = get_model_add_dist(obj, state, ref_state)
            %% FIXME this is untested
            res = randPoint(obj.model_add_dist_itv, 1, 'extreme');
            y_diff = state(obj.k.Y) - ref_state(obj.k.Y);
            if y_diff > 0
                scale = 1;
            else
                scale = -1;
            end
            res(obj.k.Y) = scale * obj.model_add_dist(obj.k.Y);
            res(obj.k.PSI) = scale * obj.model_add_dist(obj.k.PSI);
        end

        function res = get_inp_dist(obj, state, ref_state)
            res = randPoint(obj.inp_dist_itv, 1, 'extreme');
            y_diff = state(obj.k.Y) - ref_state(obj.k.Y);
            if y_diff > 0
                scale = 1;
            else
                scale = -1;
            end
            res(obj.i.DELTA) = scale * obj.inp_dist(obj.i.DELTA);
        end

    end
end
