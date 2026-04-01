function res = is_itv_empty(itv)
    dists = itv(:, 2) - itv(:, 1);
    res = any(dists <= 0);
end
