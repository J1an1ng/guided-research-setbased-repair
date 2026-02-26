function max_angle = calc_max_angle_for_rect(dist_from_rot_point_to_front, half_height, max_lat_deviation)
    rect_diag = sqrt(dist_from_rot_point_to_front^2 + half_height^2);
    rect_diag_angle = atan(half_height / dist_from_rot_point_to_front);
    max_dist_of_left_upper_point = half_height + max_lat_deviation;
    max_diag_angle = asin(max_dist_of_left_upper_point / rect_diag);
    max_angle = max_diag_angle - rect_diag_angle;
end
