function [closest_point, distance] = point_to_line_segment(p, p1, p2)
    % Vector from p1 to p2
    v = p2 - p1;
    % Vector from p1 to p
    w = p - p1;

    % Projection factor of point p onto the line defined by p1 and p2
    c1 = dot(w, v);
    c2 = dot(v, v);

    % Compute the projection point
    if c1 <= 0
        closest_point = p1;
    elseif c2 <= c1
        closest_point = p2;
    else
        b = c1 / c2;
        closest_point = p1 + b * v;
    end

    % Compute the distance from p to the closest point on the line segment
    distance = norm(p - closest_point);
end
