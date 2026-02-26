k = KSTTrailerStateOrderDefault;
params = create_kst_trailer_params_1();
occ_comp = TruckOccComputer(k, params);

meas_dist = zeros(k.SIZE, 1);
meas_dist(k.X) = 0.1;
meas_dist(k.Y) = 0.1;
meas_dist(k.DELTA) = 0.017;
meas_dist(k.V) = 0.05;
meas_dist(k.PSI) = 0.017;
meas_dist(k.HA) = 0.017;

% meas_dist = meas_dist_for_kst_trailer_1(k);
% meas_dist(k.HA) = 0.003;

% meas_dist = zeros(k.SIZE, 1);
% meas_dist(k.X) = 0.1;
% meas_dist(k.Y) = 0.12;
% meas_dist(k.DELTA) = 0.03;
% meas_dist(k.V) = 0.02;
% meas_dist(k.PSI) = 0.022;
% meas_dist(k.HA) = 0.008;

meas_dist_itv = interval(-meas_dist, meas_dist);
occs = occ_comp.compute_occupancy(meas_dist_itv);

figure;
hold on;
axis equal;

for i = 1:length(occs)
    plot(occs{i}, [1 2]);
end
