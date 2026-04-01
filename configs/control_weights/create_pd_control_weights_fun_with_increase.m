function [weights_size, f] = create_pd_control_weights_fun_with_increase(~, control_weights)
    weights_size = 5;
    assert (length(control_weights) == weights_size + 1, 'Control weights do not match the expected size.');
    f = @pd_control_weights_fun_2;

    lateral_time_factor = control_weights(end);
    control_weights = control_weights(1:weights_size);

    function w = pd_control_weights_fun_2(~, time_step)
        w = control_weights;

        w(1) = (1 + lateral_time_factor * log(time_step + 1)) * w(1);
    end

end
