function dist = zero_meas_dist_for_kst_trailer(state_order)
    arguments
        state_order KSTTrailerStateOrder
    end

    k = state_order;

    dist = zeros(k.get_size(), 1);
    dist(k.X) = 0.;
    dist(k.Y) = 0.;
    dist(k.PSI) = 0.;
    dist(k.DELTA) = 0.;
    dist(k.V) = 0.;
    dist(k.HA) = 0.;
end
