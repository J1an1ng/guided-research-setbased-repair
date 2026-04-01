classdef WorstCaseKSTYSimulator < WorstCaseSimulator
    properties (Access = private)
        k KSTStateOrder
        i InpOrderADeltaV
    end

    methods

        function obj = WorstCaseKSTYSimulator(dyn, param_conf, meas_dist, model_add_dist, inp_dist, k, i)
            obj@WorstCaseSimulator(dyn, param_conf, meas_dist, model_add_dist, inp_dist);
            obj.k = k;
            obj.i = i;
        end

    end

    methods (Access = protected)

        function res = get_meas_dist(obj, state)
            res = randPoint(obj.meas_dist_itv, 1, 'extreme');

            if state(obj.k.Y) > 0
                scale = -1;
            else
                scale = 1;
            end

            % we choose the measurement disturbance such that the controller can only poorly
            % counteract
            res(obj.k.Y) = scale * obj.meas_dist(obj.k.Y);
            res(obj.k.DELTA) = scale * obj.meas_dist(obj.k.DELTA);
            res(obj.k.PSI) = scale * obj.meas_dist(obj.k.PSI);
        end

        function res = get_model_add_dist(obj, state)
            %% FIXME this is untested
            res = randPoint(obj.model_add_dist_itv, 1, 'extreme');
            if state(obj.k.Y) > 0
                scale = 1;
            else
                scale = -1;
            end
            res(obj.k.Y) = scale * obj.model_add_dist(obj.k.Y);
            res(obj.k.DELTA) = scale * obj.model_add_dist(obj.k.DELTA);
            res(obj.k.PSI) = scale * obj.model_add_dist(obj.k.PSI);
        end

        function res = get_inp_dist(obj, state)
            res = randPoint(obj.inp_dist_itv, 1, 'extreme');
            if state(obj.k.Y) > 0
                scale = 1;
            else
                scale = -1;
            end
            res(obj.i.DELTA_V) = scale * obj.inp_dist(obj.i.DELTA_V);
        end

    end
end
