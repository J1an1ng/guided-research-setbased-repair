function dist = meas_dist_for_kst_trailer_with_acc_1(state_order)
    arguments
        state_order KSTTrailerWithAccStateOrder
    end

    k = state_order;

    dist = zeros(k.SIZE, 1);
    dist(k.X) = 0.02;
    dist(k.Y) = 0.02;
    dist(k.PSI) = 0.004;
    dist(k.DELTA) = 0.004;
    dist(k.V) = 0.06;
    dist(k.HA) = 0.004;
    dist(k.A) = 0.05;
end
