function res = get_kst_initial_itv_size(k, state_meas_dist)
    res = dictionary();

    res{k.X} = [-state_meas_dist(k.X), state_meas_dist(k.X)];
    res{k.Y} = 2 * [-state_meas_dist(k.Y), state_meas_dist(k.Y)];
    res{k.PSI} = [-state_meas_dist(k.PSI), state_meas_dist(k.PSI)];
end
