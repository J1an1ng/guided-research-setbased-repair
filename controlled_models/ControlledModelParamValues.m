classdef ControlledModelParamValues < handle & matlab.mixin.Heterogeneous
    properties (SetAccess = immutable, GetAccess = private)
        meas_dist (:, 1) double
        model_add_dist (:, 1) double
        model_inp_dist (:, 1) double
        control_weights_arg
    end

    properties (SetAccess = immutable, GetAccess = public)
        meas_dist_itv
        model_add_dist_itv
        model_inp_dist_itv

        combined_dist_set

        ref_state_builder AbstractRefStateBuilder

        ref_state_args
    end

    properties (SetAccess = private, GetAccess = public)
        control_weights (:, 1) double
        control_weights_fun function_handle
    end

    methods

        function obj = ControlledModelParamValues( ...
                meas_dist, model_add_dist, model_inp_dist, ...
                combined_dist_set, ...
                ref_state_builder, ...
                control_weights, ...
                ref_state_args)
            if ~exist('ref_state_args', 'var')
                ref_state_args = dictionary();
            end

            obj.meas_dist = meas_dist;
            obj.model_add_dist = model_add_dist;
            obj.model_inp_dist = model_inp_dist;

            obj.meas_dist_itv = interval(-meas_dist, meas_dist);
            obj.model_add_dist_itv = interval(-model_add_dist, model_add_dist);
            obj.model_inp_dist_itv = interval(-model_inp_dist, model_inp_dist);

            obj.combined_dist_set = combined_dist_set;

            obj.ref_state_args = ref_state_args;

            obj.ref_state_builder = ref_state_builder;
            obj.control_weights_arg = control_weights;
            obj.set_control_weights(control_weights);
        end

        function w = get_control_weights(obj, ref_traj, time_step)
            %% TODO remove at some point - for identifying issues after refactoring only
            if isa(ref_traj, 'Primitive')
                error('ref_traj should not be a Primitive - did you forget to access ref_traj field?');
            end

            if isempty(obj.control_weights_fun)
                w = obj.control_weights;
            else
                w = obj.control_weights_fun(ref_traj, time_step);
            end
        end

        function new_obj = with(obj, NameValueArgs)
            arguments
                obj ControlledModelParamValues
                NameValueArgs.meas_dist = obj.meas_dist
                NameValueArgs.model_add_dist = obj.model_add_dist
                NameValueArgs.model_inp_dist = obj.model_inp_dist
                NameValueArgs.combined_dist_set = obj.combined_dist_set
                NameValueArgs.ref_state_builder = obj.ref_state_builder
                NameValueArgs.control_weights = obj.control_weights_arg
                NameValueArgs.ref_state_args = obj.ref_state_args
            end
            % if ~isfield(NameValueArgs, 'meas_dist')
            % NameValueArgs.meas_dist = obj.meas_dist;
            % end

            new_obj = ControlledModelParamValues( ...
                NameValueArgs.meas_dist, ...
                NameValueArgs.model_add_dist, ...
                NameValueArgs.model_inp_dist, ...
                NameValueArgs.combined_dist_set, ...
                NameValueArgs.ref_state_builder, ...
                NameValueArgs.control_weights, ...
                NameValueArgs.ref_state_args);
        end

    end

    methods (Access = private)

        function set_control_weights(obj, control_weights)
            if isa(control_weights, 'function_handle')
                obj.control_weights = [];
                obj.control_weights_fun = control_weights;
            else
                obj.control_weights = control_weights;
            end
        end

    end
end
