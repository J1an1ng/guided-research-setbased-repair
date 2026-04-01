function [epsilon_long, epsilon_lat] = get_pos_deviation(psi_ref, x_ref, y_ref, x, y)
    epsilon_long = cos(psi_ref) * (x_ref - x) + sin(psi_ref) * (y_ref - y);
    epsilon_lat = -sin(psi_ref) * (x_ref - x) + cos(psi_ref) * (y_ref - y);
end
