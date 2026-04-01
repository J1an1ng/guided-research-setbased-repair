function res = to_bounds(value, value_bounds)
    res = max(min(value, value_bounds(2)), value_bounds(1));
end
