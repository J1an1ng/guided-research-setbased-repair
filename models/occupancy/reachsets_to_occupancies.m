function res = reachsets_to_occupancies(reachsets, occ_computer, num_occs_to_merge)
    num_occs_sets = ceil(length(reachsets) / num_occs_to_merge);
    res = cell(num_occs_sets, occ_computer.num_shapes, 2);
    % number of occupancies x number of shapes x inf and sup x position (x and y)
    for i = 1:num_occs_sets
        start_idx = (i - 1) * num_occs_to_merge + 1;
        end_idx = min(i * num_occs_to_merge, length(reachsets));
        merged_occs = occ_computer.compute_accumulated_occupancy_for_multiple_time_intervals( ...
            reachsets, start_idx, end_idx);
        for k = 1:occ_computer.num_shapes
            % we compute the vector to the top or bottom of the shape
            [itv_rot, itv_back_rot] = fit_with_interval(merged_occs{k});

            res{i, k, 1} = itv_rot;
            res{i, k, 2} = itv_back_rot;
        end
    end
end
