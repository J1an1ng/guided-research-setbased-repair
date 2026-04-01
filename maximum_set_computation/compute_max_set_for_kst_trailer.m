k = KSTTrailerStateOrderDefault;
params = create_kst_trailer_params_2();

max_long_deviation = 0.5;
max_lat_deviation = 0.5;

relevant_dims = [k.X k.Y k.PSI k.HA];

max_psi = calc_max_angle_for_rect(params.l_front, params.truck_width / 2, max_lat_deviation);
max_total_ha = calc_max_angle_for_rect(params.trailer_length - params.trailer_dist_from_front_to_hitch, ...
    params.trailer_width / 2, max_lat_deviation);
max_ha = max_total_ha + max_psi;
max_values_per_dim = [max_long_deviation max_lat_deviation max_psi max_ha];

occ_comp = TruckOccComputer(k, params);

max_truck_w = params.truck_width / 2 + max_lat_deviation;
max_truck_l = params.truck_length / 2 + max_long_deviation;
max_trailer_w = params.trailer_width / 2 + max_lat_deviation;
max_trailer_l = params.trailer_length / 2 + max_long_deviation;
max_size_per_occ = [max_truck_w max_truck_l; max_trailer_w max_trailer_l];

file_path = 'maximum_set_computation/halfspace_lengths_5.mat';

o = MaximumSetComputer(k, relevant_dims, occ_comp, max_values_per_dim, max_size_per_occ, file_path);
c = zeros(length(relevant_dims), 1);
o.compute_max_set(c);
