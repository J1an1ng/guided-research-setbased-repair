function [accs, acc_max] = create_accelerations(v_range_size, primitive_duration, v_meas_dist)
    %% The accelerations should be chosen in such a way that the velocity of a primitive
    %% ends up roughly in the center of the next initial interval
    acc_weak = -((v_range_size - v_meas_dist) / primitive_duration);
    acc_strong = 2 * acc_weak;

    acc_profile_start = -0.5;
    acc_profile_weak = linspace(acc_profile_start, acc_strong - acc_profile_start, 5);

    accs = {acc_strong, acc_profile_weak};
    acc_max = acc_weak;
end
