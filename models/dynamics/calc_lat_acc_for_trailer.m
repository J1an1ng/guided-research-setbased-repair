function res = calc_lat_acc_for_trailer(v, delta, truck_wheelbase, dist_from_rear_axle_to_hitch, trailer_wheelbase, alpha)
    % gamma is the heading angle of the trailer: gamma = psi + alpha.
    % The lateral acceleration is calculated as v * gamma_dot, i.e., v * gamma_dot = v * (psi_dot + alpha_dot).
    % When reformulating the latter part, one gets the formula below:
    psi_dot = calc_psi_dot(v, delta, truck_wheelbase);
    gamma_dot = (dist_from_rear_axle_to_hitch * psi_dot * cos(alpha) - v * sin(alpha)) / trailer_wheelbase;
    res = v * gamma_dot;
end
