function [shortest_vector, min_distance, longest_vector, max_distance] = calc_vectors_to_boundary(poly)

    V = poly.set.Vertices;
    n = size(V, 1);

    centroid = center(poly)';

    min_distance = Inf;
    max_distance = -Inf;
    closest_point = [];
    furthest_point = [];

    % Step 2: Loop through each edge of the polygon to find the shortest distance
    for i = 1:n
        % Get the current edge endpoints
        p1 = V(i, :);
        p2 = V(mod(i, n) + 1, :); % Wrap around to the first vertex

        % Compute the closest point on the current edge to the centroid
        [proj_point, dist] = point_to_line_segment(centroid, p1, p2);

        % Update the minimum distance and closest point if necessary
        if dist < min_distance
            min_distance = dist;
            closest_point = proj_point;
        end

        % Update the maximum distance and furthest point if necessary
        if dist > max_distance
            max_distance = dist;
            furthest_point = proj_point;
        end
    end

    % Compute the shortest distance vector from the centroid to the closest point
    shortest_vector = closest_point - centroid;
    longest_vector = furthest_point - centroid;
end
