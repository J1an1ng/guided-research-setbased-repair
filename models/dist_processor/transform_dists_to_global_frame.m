function dists = transform_dists_to_global_frame(state, state_order, dists)
    % The positional disturbances are provided in the
    % vehicle frame, so we need to rotate them to the world frame:
    k = state_order;
    pos_dists = dists([k.X k.Y]);
    pos_dists = rot(state(k.PSI)) * pos_dists;
    dists([k.X k.Y]) = pos_dists;
end
