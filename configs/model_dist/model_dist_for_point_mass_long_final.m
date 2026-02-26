function dist = model_dist_for_point_mass_long_final(state_order)
    arguments
        state_order PointMassLongStateOrder
    end

    k = state_order;

    k_kst = KSTTrailerStateOrder();
    dist_kst_final = model_dist_for_kst_trailer_final(k_kst);

    dist = zeros(k.get_size(), 1);
    dist(k.S) = dist_kst_final(k_kst.X);
    dist(k.V) = dist_kst_final(k_kst.V);
end
