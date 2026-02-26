function res = Expand_Polytope(Polytope, Keep_Dimensions, New_Dimensions, boundaries)
    % EXPAND_POLYTOPE Summary of this function goes here
    %   Detailed explanation goes here
    n_rows_old = size(Polytope.A, 1);
    n_column_old = size(Polytope.A, 2);
    n_rows_new = n_rows_old + 2 * size(New_Dimensions, 2);
    n_column_new = n_column_old + size(New_Dimensions, 2);

    A_new = zeros(n_rows_new, n_column_new);
    b_new = zeros(n_rows_new, 1);

    % Put in old data
    for jj = 1:size(Keep_Dimensions, 2)
        A_new(1:n_rows_old, Keep_Dimensions(jj)) = Polytope.A(1:n_rows_old, jj);
    end
    b_new(1:n_rows_old) = Polytope.b(1:n_rows_old);

    % Put in new interval data
    for kk = 1:size(New_Dimensions, 2)
        A_new(n_rows_old + 1 + 2 * (kk - 1):n_rows_old + 2 + 2 * (kk - 1), New_Dimensions(kk)) = [1; -1];
        b_new(n_rows_old + 1 + 2 * (kk - 1):n_rows_old + 2 + 2 * (kk - 1)) = [boundaries(1, kk), -boundaries(2, kk)];
    end

    res = polytope(A_new, b_new);

end
