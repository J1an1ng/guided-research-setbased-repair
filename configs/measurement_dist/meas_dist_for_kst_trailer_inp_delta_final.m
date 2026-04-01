function dist = meas_dist_for_kst_trailer_inp_delta_final(state_order)
    arguments
        state_order KSTTrailerInpDeltaStateOrder
    end

    k = state_order;

    k_kst = KSTTrailerStateOrder();
    dist_kst_final = meas_dist_for_kst_trailer_final(k_kst);

    dist = zeros(k.get_size(), 1);

    dist(k.X) = dist_kst_final(k_kst.X);
    dist(k.Y) = dist_kst_final(k_kst.Y);
    dist(k.V) = dist_kst_final(k_kst.V);
    dist(k.PSI) = dist_kst_final(k_kst.PSI);
    dist(k.HA) = dist_kst_final(k_kst.HA);
end
