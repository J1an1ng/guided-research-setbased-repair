function r_normed = norm_kst(r, state_order)
    %% this function can be used for any vehicle model with orientation,
    %% and x and y position.

    dims_to_norm = [state_order.X state_order.Y state_order.PSI];

    % shift the set to the origin of the position space
    c = center(r);
    shift_vector = zeros(state_order.get_size(), 1);
    shift_vector(dims_to_norm) = c(dims_to_norm);
    r_normed = r - shift_vector;

    % rotate the set by the orientation
    psi = -c(state_order.PSI);

    T = eye(state_order.get_size());
    pos_dims = [state_order.X state_order.Y];
    T(pos_dims, pos_dims) = [cos(psi) -sin(psi); sin(psi) cos(psi)];

    r_normed = T * r_normed;
end
