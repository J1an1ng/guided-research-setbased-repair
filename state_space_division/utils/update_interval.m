function itv = update_interval(itv, dims, total_bounds, lb, range_sizes)
    ub = lb + extract_from(range_sizes, 1, dims);
    ub = min(ub, extract_from(total_bounds, 2, dims));
    itv(dims, :) = [lb ub];
end
