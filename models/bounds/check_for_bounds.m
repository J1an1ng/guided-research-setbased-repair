function res = check_for_bounds(value, lb, ub)
    if value < lb
        res = -1;
    elseif value > ub
        res = 1;
    else
        res = 0;
    end
end
