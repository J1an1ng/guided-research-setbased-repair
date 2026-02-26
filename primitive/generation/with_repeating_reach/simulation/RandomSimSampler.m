classdef RandomSimSampler < SimulationSampler
    properties (Access = private)
        meas_dist (:, 1) double
        meas_dist_itv
        model_add_dist (:, 1) double
        model_add_dist_itv
        inp_dist (:, 1) double
        inp_dist_itv
    end

    methods

        function obj = RandomSimSampler(meas_dist, model_add_dist, inp_dist)
            obj.meas_dist = meas_dist;
            obj.meas_dist_itv = interval(zeros(size(meas_dist)), meas_dist);
            obj.model_add_dist = model_add_dist;
            obj.model_add_dist_itv = interval(zeros(size(model_add_dist)), model_add_dist);
            obj.inp_dist = inp_dist;
            obj.inp_dist_itv = interval(zeros(size(inp_dist)), inp_dist);
        end

        function x = sample_initial_state(~, initial_set)
            x = randPoint(initial_set, 1);
        end

        function res = sample_meas_dist(obj, ~, ~)
            res = randPoint(obj.meas_dist_itv, 1);
        end

        function res = sample_model_add_dist(obj, ~, ~)
            %% FIXME this is untested
            res = randPoint(obj.model_add_dist_itv, 1);
        end

        function res = sample_inp_dist(obj, ~, ~)
            res = randPoint(obj.inp_dist_itv, 1);
        end

    end
end
