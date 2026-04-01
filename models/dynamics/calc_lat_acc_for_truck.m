function res = calc_lat_acc_for_truck(delta, v, truck_wheelbase)
    psi_dot = calc_psi_dot(v, delta, truck_wheelbase);
    res = v * psi_dot;
end
