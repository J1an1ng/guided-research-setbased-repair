k = KSTTrailerStateOrderDefault;
i = InpOrderADeltaVDefault;
params = create_kst_trailer_params_1();
constraints = KSTTrailerConstraints(k, i, params);
% interval_sampler = create_initial_interval_sampler_for_kst_trailer_1(k, params, constraints);
interval_sampler = create_initial_sampler_for_kst_with_delta_v_corr(k, params, constraints);

all_intervals = interval_sampler.sample_all();
cons_int = squeeze(all_intervals(1, :, :));
cons_int = interval(cons_int(:, 1), cons_int(:, 2));

occ_comp = TruckOccComputer(k, params);

reachsets = {};
reachsets{1} = zonotope(cons_int);
polygon_list = occ_comp.compute_accumulated_occupancy(reachsets);

figure('Visible', 'on');
plot(polygon_list{1});
axis equal;
hold on;
plot(polygon_list{2}, 'r');
axis equal;
