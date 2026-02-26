classdef (Abstract) StateProcessor < handle & matlab.mixin.Heterogeneous
    % Class for performing state set transformations during primitive
    % generation, more precisely: before computing the reachable set for a
    % motion primitive, we always need to prepare the initial set of the
    % vehicle according to the used control mechanism. This class serves as
    % an interface for such processing tasks.
    methods (Abstract)

        process_initial_set(obj, initial_set)

        process_initial_set_before_reach(obj, r_0)

        process_final_set_after_reach(obj, r_final)

        project_final_set(obj, r)

        get_state_size(obj)

        process_state_before_sim(obj, state, meas_dist)

        process_state_after_sim(obj, state)

        split_and_process(obj, state_collection)

        create_derivative(obj, state, derivative_state, model_add_dist)

    end
end
