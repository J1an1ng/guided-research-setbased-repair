function r_shifted = shift_kst(r, state_order)
    %% this function can be used for any vehicle model with orientation,
    %% and x and y position.

    % shift the set to the origin of the position space
    c = center(r);
    r_shifted = r - c;

    % rotate the set by the orientation
    psi = -c(state_order.PSI);

    T = eye(state_order.get_size());
    pos_dims = [state_order.X state_order.Y];
    T(pos_dims, pos_dims) = [cos(psi) -sin(psi); sin(psi) cos(psi)];

    r_shifted = T * r_shifted;
end
