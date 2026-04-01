function dist = meas_dist_for_point_mass_long_1(state_order)
    arguments
        state_order PointMassLongStateOrder
    end

    k = state_order;

    k_kst = KSTTrailerStateOrder();
    dist_kst_1 = meas_dist_for_kst_1(k_kst);

    dist = zeros(k.get_size(), 1);

    dist(k.S) = dist_kst_1(k_kst.X);
    dist(k.V) = dist_kst_1(k_kst.V);
end
