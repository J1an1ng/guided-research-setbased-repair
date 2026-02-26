function final_interval = create_final_interval_scaffold(state_length, ...
    sampling_dims, sampling_dims_range_sizes, ...
    other_dims, other_dims_intervals, ...
    overlap_fractions, overlap_abs)

    final_interval = zeros(state_length, 2);
    for i = 1:length(other_dims)
        state_idx = other_dims(i);
        final_interval(state_idx, :) = other_dims_intervals{state_idx};
    end
    for i = 1:length(sampling_dims)
        state_idx = sampling_dims(i);
        range_size = sampling_dims_range_sizes{state_idx};
        % we must consider the overlap, as
        % otherwise a final set might not fit into any initial
        % interval. But: it is not necessary for the dimensions
        % that are not sampled
        if overlap_fractions.isKey(state_idx)
            overlap_fr = overlap_fractions(state_idx);
            final_interval(state_idx, :) = overlap_fr * [-range_size / 2 range_size / 2];
        else
            overlap = overlap_abs(state_idx);
            final_interval(state_idx, :) = [-overlap / 2 overlap / 2];
        end
    end
end
