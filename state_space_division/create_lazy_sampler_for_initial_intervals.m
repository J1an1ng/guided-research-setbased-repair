function [sampler, step_sizes] = create_lazy_sampler_for_initial_intervals( ...
    dims, bounds, range_sizes, ...
    overlap_fractions, overlap_abs, invert_sampling_order, max_samples_per_dim)

    if ~exist("max_samples_per_dim", "var")
        max_samples_per_dim = dictionary(uint32.empty, uint32.empty);
    end

    step_sizes = dictionary();
    sampling_values = cell(length(dims), 1);
    for i = 1:length(dims)
        state_idx = dims(i);
        b = bounds{state_idx};
        range_size = range_sizes{state_idx};
        if overlap_fractions.isKey(state_idx)
            overlap = overlap_fractions(state_idx) * range_size;
        else
            overlap = overlap_abs(state_idx);
        end
        step = range_size - overlap;

        EPS = 0.0000001;
        from = b(1);
        to = b(2) - range_size + EPS;

        if isKey(max_samples_per_dim, state_idx)
            m = max_samples_per_dim(state_idx);
            step = max(step, (to - from) / max(m - 1, 1));
        end
        step_sizes(state_idx) = step;

        values = from:step:to;

        if size(values, 2) == 0
            throw(MException('StateIntervalSampler:EmptySamplingRange', ...
                sprintf('Empty sampling range for %d', state_idx)));
        end

        % we ensure that the upper bound is reached by the last
        % interval:
        if values(end) + range_size + EPS < b(2)
            values(end + 1) = values(end) + step; %#ok<AGROW>
        end

        inv = invert_sampling_order{state_idx};
        if inv
            values = flip(values);
        end
        sampling_values{i} = values;
    end
    sampler = LazySampler(sampling_values);
end
