function [weights_size, f] = create_pd_control_weights_fun_with_increase_speed_wise(k)
    weights_size = 5;
    f = @pd_control_weights_fun;

    weights_1 = [9.5381; 371.4935; 22.1964; 55.7358; 79.7620; 1.7452];
    weights_2 = [9.5192; 378.4692; 22.3658; 57.8568; 80.2606; 1.7218];
    weights_3 = [11.8783; 506.3667; 22.3793; 9.9988; 100.3323; 1.5619];
    weights_4 = [0.6836; 154.2969; 21.2402; 484.8633; 122.2656; 0.8545];
    weights_5 = [7.0312; 421.8750; 47.2656; 175.7812; 128.1250; 0.6641];
    weights_6 = [1.5869; 91.3086; 28.2349; 341.1865; 106.7383; 0.9412];
    weights = {weights_1, weights_2, weights_3, weights_4, weights_5, weights_6};
    speed_bounds = [Inf, 15.0, 10.0, 7.5, 5.0, 2.5];
    assert(length(speed_bounds) == length(weights));

    function w = pd_control_weights_fun(ref_traj, time_step)
        v = ref_traj.states(k.V, 1);
        for i = length(speed_bounds):-1:1
            if v < speed_bounds(i)
                w = prepare_weights(weights{i}, weights_size, time_step);
                return
            end
        end
        throw(MException('MATLAB:illegalState', 'Speed out of bounds'));
    end

end

function weights = prepare_weights(weights, weights_size, time_step)
    lateral_time_factor = weights(end);
    weights = weights(1:weights_size);
    weights(1) = (1 + lateral_time_factor * log(time_step + 1)) * weights(1);
end
