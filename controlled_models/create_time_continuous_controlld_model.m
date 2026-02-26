%% FIXME adapt signature as in create_time_continuous_discretely_controlled_model
function [system_dynamics, system_output] = create_time_continuous_controlld_model()

    function dx = get_system_dynamics( ...
            state, model_params, ...
            model, controller, ...
            state_order, inp_order, params, model_param_conf)

        k = state_order;

        [current_meas_dist, current_model_add_dist, current_inp_dist, ...
            ref_state, ref_inp, weights] = model_param_conf.split(model_params);

        u = controller(state + current_meas_dist, ref_state, ref_inp, state_order, inp_order, params, weights);

        dx(1:k.get_size(), 1) = model(state, u + current_inp_dist, state_order, inp_order, ...
            params) + current_model_add_dist;
    end

    system_dynamics = @get_system_dynamics;

    function output = get_system_output(state, model_params, model_param_conf)
        current_meas_dist = model_param_conf.get_current_meas_dist(model_params);
        output = state + current_meas_dist;
    end

    system_output = @get_system_output;
end
