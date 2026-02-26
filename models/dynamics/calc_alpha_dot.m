function alpha_dot = calc_alpha_dot(v, ha, trailer_wheelbase, truck_dist_from_rear_axle_to_hitch, psi_dot)
    alpha_dot = -v * sin(ha) / trailer_wheelbase + psi_dot * ((truck_dist_from_rear_axle_to_hitch / trailer_wheelbase) * cos(ha) - 1);
end
