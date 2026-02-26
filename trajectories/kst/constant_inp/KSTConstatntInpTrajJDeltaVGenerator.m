classdef KSTConstatntInpTrajJDeltaVGenerator < AbstractInpTrajGenerator
    % Sampling is done in the following way:
    %   - jerk: user-defined negative jerk-profiles are used
    %   - steering angle speed: different linearly increasing
    %   and decreasing steering angle speed courses are sampled.
    % Throughout this class, we write dv for steering angle speed and j for
    % jerk
    properties (Access = private)
        state_order AbstractKSTTrailerWithAccStateOrder
        inp_order AbstractInpOrderJDeltaV
        params AbstractKSTTrailerParams

        % steering angle speed bounds that should be used
        min_dv double
        max_dv double

        % cell array, where the rows contain the jerk-profiles
        j_profiles (:, :) double {mustBeNegative}
        j_profile_dt double {mustBePositive}

        sampler Sampler
    end

    methods

        function obj = KSTConstatntInpTrajJDeltaVGenerator( ...
                state_order, inp_order, params, ...
                min_start_dv, max_start_dv, ...
                start_dv_step_size, max_abs_dv_gradient, dv_gradient_step_size, ...
                j_profiles, j_profile_dt)
            obj.state_order = state_order;
            obj.inp_order = inp_order;
            obj.params = params;

            obj.j_profiles = j_profiles;
            obj.j_profile_dt = j_profile_dt;
            obj.min_dv = min_start_dv;
            obj.max_dv = max_start_dv;

            dv_starts = min_start_dv:start_dv_step_size:max_start_dv;
            dv_gradients = -max_abs_dv_gradient:dv_gradient_step_size:max_abs_dv_gradient;

            obj.sampler = Sampler({ 1:size(j_profiles, 1); dv_starts; dv_gradients });
        end

        function trajectories = generate(obj, ~)
            all_samples = obj.sampler.sample_all();
            trajectories = cellfun(@(sample) KSTConstantInpTrajJDeltaV( ...
                obj.state_order, obj.inp_order, ...
                obj.j_profiles(sample(1), :), obj.j_profile_dt, ...
                sample(2), samples(3), obj.params), ...
                all_samples);
        end

    end
end
