function Q = q_from_vec(n, Q_vec)
    Q = diag(Q_vec(1:n)) + squareform(Q_vec(n + 1:end));
end
