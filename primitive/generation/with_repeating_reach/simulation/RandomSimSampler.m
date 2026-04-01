classdef RandomSimSampler < SimulationSampler
    properties (Access = private)
        meas_dist_set
        model_add_dist_set
        inp_dist_set
    end

    methods

        function obj = RandomSimSampler(meas_dist_set, model_add_dist_set, inp_dist_set)
            obj.meas_dist = meas_dist;
            obj.meas_dist_set = meas_dist_set;
            obj.model_add_dist_set = model_add_dist_set;
            obj.inp_dist_set = inp_dist_set;
        end

        function x = sample_initial_state(~, initial_set)
            x = randPoint(initial_set, 1);
        end

        function res = sample_meas_dist(obj, ~, ~)
            res = randPoint(obj.meas_dist_set, 1);
        end

        function res = sample_model_add_dist(obj, ~, ~)
            %% FIXME this is untested
            res = randPoint(obj.model_add_dist_set, 1);
        end

        function res = sample_inp_dist(obj, ~, ~)
            res = randPoint(obj.inp_dist_set, 1);
        end

    end
end
