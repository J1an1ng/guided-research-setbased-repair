function res = merge_zonotopes(set_1, set_2, max_enlargement)
    if ~exist('max_enlargement', 'var')
        max_enlargement = 2.0;
    end

    set_1 = reduce(set_1, 'pca', 5);
    set_2 = reduce(set_2, 'pca', 5);
    c_shift = (center(set_1) - center(set_2)) / 2;
    set_1 = set_1 - c_shift;
    set_1 = set_1 + zonotope(zeros(dim(set_1), 1), diag(c_shift));

    function tmp_res = check_enlargement(enlargement)
        set_1_enlarged = enlarge(set_1, enlargement);
        % we have to use try-catch as mosek sometimes throws the error
        % "Fatal MOSEK error in src/misc/sort.c line 1113."
        try
            tmp_res = contains(set_1_enlarged, set_2, 'approx');
        catch
            tmp_res = false;
        end
    end

    accuracy = 0.015;
    enlargement = binary_search_for_min(@check_enlargement, 1.0, max_enlargement, accuracy);
    if is_empty_var(enlargement)
        res = [];
        return
    else
        res = enlarge(set_1, enlargement);
    end
end

function sol = binary_search_for_min(fun, min_value, max_value, accuracy)
    sol = [];

    if fun(min_value)
        sol = min_value;
        return
    end

    if ~fun(max_value)
        return
    end

    while max_value - min_value >= accuracy
        middle = mean([min_value max_value]);
        if fun(middle)
            max_value = middle;
        else
            min_value = middle;
        end
    end
    sol = max_value;
end
