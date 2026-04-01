%% TODO remove file
function arr = remove_duplicate_vectors(arr, tolerance)
    if ~exist('tolerance', 'var')
        tolerance = 1e-5;
    end
    n_cols = size(arr, 2);
    keep_cols = true(1, n_cols);
    for i = 1:n_cols - 1
        for j = i + 1:n_cols
            if keep_cols(j)
                col_1 = arr(:, i);
                col_2 = arr(:, j);
                % If the difference between normalized columns is
                % less than tolerance, mark for removal
                if norm(col_1 - col_2) < tolerance
                    keep_cols(j) = false;
                end
            end
        end
    end
    arr = arr(:, keep_cols);
end
