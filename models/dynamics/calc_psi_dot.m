function psi_dot = calc_psi_dot(v, delta, truck_wheelbase)
    psi_dot = v * tan(delta) / truck_wheelbase;
end
