%% This script computes the maximum state set for the KST trailer model with ellipsoidal shape
%% which does not exceed a given occupancy width and length.

k = KSTTrailerStateOrderDefault;
red_k = TrailerOccStateOrder();
params = create_kst_trailer_params_2();

% constraints on the occupancy
max_long_deviation = 0.5;
max_lat_deviation = 0.5;

% dimensions that influence the occupancy
relevant_dims = red_k.create_state_order_mapping(k);
ha_in_red = 4; % index of the hitch angle in relevant_dims

% we expect roughly these radii per dimension. In other words: we hope
% to almost reach these radii.
desired_rads_per_dim = [0.3 0.3 0.03 0.03];

occ_comp = TruckOccComputer(k, params);

max_truck_w = params.truck_width / 2 + max_lat_deviation;
max_truck_l = params.truck_length / 2 + max_long_deviation;
max_trailer_w = params.trailer_width / 2 + max_lat_deviation;
max_trailer_l = params.trailer_length / 2 + max_long_deviation;
% these are bounds on the occupancy half width and length
max_size_per_occ = [max_truck_w max_truck_l; max_trailer_w max_trailer_l];

out_path = 'maximum_set_computation';
iteration = 16; % just to have a unique identifier for the output file

n = length(relevant_dims);
% the Q matrix of the ellipsoid has to be symmetric; this it the number
% of distinct entries
Q_symm_size = (n + 1) * (n / 2);
vec_size = Q_symm_size - 2; % we use the same entry for x and y and 0 for their mutual entry

max_ha = params.max_hitch_angle;
num_centers = 5;
centers = zeros(length(relevant_dims), num_centers);
centers(ha_in_red, :) = linspace(0, max_ha, num_centers);

o = MaximumSetComputerEllipsoid(k, relevant_dims, occ_comp, ...
    desired_rads_per_dim, max_size_per_occ, ...
    vec_size, @custom_q_from_vec, ...
    out_path, iteration);
o.compute_max_set(centers);

function Q = custom_q_from_vec(n, Q_vec)
    % we want a circle shape for x and y, thus we use the same radius for
    % both dimensions and 0 for their mutual entries. The given Q_vec only contains
    % the lower triangle of the matrix, thus we have to add the first and the 0 entry
    Q_vec_ext = [Q_vec(1), Q_vec(1:n - 1), 0, Q_vec(n:end)];
    Q = q_from_vec(n, Q_vec_ext);
end
