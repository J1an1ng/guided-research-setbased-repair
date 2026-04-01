function res = extreme_point(cora_set, direction_sign_vector)
    itv_rads = rad(interval(cora_set));
    direction_vector = direction_sign_vector .* itv_rads;
    % We need the following this if the set is not an interval.
    % Example: if the corner of two dims x and y is "cut away", then the direction vector
    % [rad(x), rad(y)] is ot what we want as it is not orthogonal to the line connecting
    % [rad(x), 0] and [0, rad(y)]; we actually need a normal vector on the hyperplane
    % defined by the points [rad(x), 0 ... 0], [0, rad(y), 0 ... 0], ...
    % direction_vector = 1 ./ direction_vector;
    direction_vector(direction_vector ~= 0) = 1.0 ./ direction_vector(direction_vector ~= 0);

    [~, res] = supportFunc(cora_set, direction_vector);

    % plot(initial_set, [4 5]);
    % hold on;
    % min_rad = min(itv_rads);
    % direction_vector_scaled = (min_rad / min(abs(direction_vector))) * direction_vector;
    % quiver(0, 0, direction_vector_scaled(4), direction_vector_scaled(5));
    % axis equal;
    % plot(x(4), x(5), '.', 'MarkerSize', 10);
end
