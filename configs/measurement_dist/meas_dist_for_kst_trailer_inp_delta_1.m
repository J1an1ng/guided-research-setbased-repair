function dist = meas_dist_for_kst_trailer_inp_delta_1(state_order)
    arguments
        state_order KSTTrailerInpDeltaStateOrder
    end

    k = state_order;

    k_kst = KSTTrailerStateOrder();
    dist_kst_1 = meas_dist_for_kst_1(k_kst);

    dist = zeros(k.get_size(), 1);

    dist(k.X) = dist_kst_1(k_kst.X);
    dist(k.Y) = dist_kst_1(k_kst.Y);
    dist(k.V) = dist_kst_1(k_kst.V);
    dist(k.PSI) = dist_kst_1(k_kst.PSI);

    % If measuring the trailer orientation, we can assume to get the same
    % accuracy as for the truck - additionally measuring the HA should
    % actually improve the accuracy of both the truck and trailer orientation.
    % If we set the HA-error to 0, the psi-error would already cover the
    % orientation error of the trailer. Thus, the HA-error is only an additional
    % uncertainty, which we can choose smaller than the psi-error as the latter already
    % covers the orientation error of the trailer.
    % Actually, the measurement uncertainty of the trailer angle does not matter,
    % as it is not used by the controller.
    dist(k.HA) = 0.0005;
end
