function vectors = get_all_combinations(values, n)
    %% generates a n x m matrix contianing all vectors of length n with values in values

    % Generate all possible combinations
    [grid{1:n}] = ndgrid(values);

    % Convert the grid to a matrix where each row is a vector of length n
    vectors = reshape(cat(n + 1, grid{:}), [], n);

    vectors = vectors';
end
