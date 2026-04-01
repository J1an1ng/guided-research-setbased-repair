function [itv, itv_rotated] = fit_with_interval(z)
    [shortest_vector, ~, ~, ~] = calc_vectors_to_boundary(z);
    angle = atan2(shortest_vector(2), shortest_vector(1));

    if angle >= 0
        % the shortest vector points to the top
        target_angle = pi / 2;
    else
        % the shortest vector points to the bottom
        target_angle = -pi / 2;
    end

    %% Hint: we perform all these steps one after the other
    %% as chaining them resulted in weird errors, namely the
    %% line 'rot(-rot_angle) * polytope(itv)' returned a line
    %% instead of the whole polytope, which did not happen
    %% when invoking the statements separately.
    rot_angle = target_angle - angle;
    r = rot(rot_angle);
    r_inv = rot(-rot_angle);
    % we rotate the occupancy to make it axis-aligned and
    % create an interval around it...
    occ_rot = r * z;
    itv = interval(occ_rot);
    % ...and convert it back to a polytope and rotate it back
    itv_rot_zono = zonotope(itv);
    itv_rotated = r_inv * itv_rot_zono;
end
