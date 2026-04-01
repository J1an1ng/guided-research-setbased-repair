function res = is_empty_var(v)
    res = isnumeric(v) && all(size(v) == [0, 0]);
end
