function res = random_sign(dims)
    b = randi([0 1], dims, 1);
    res = 2 * b - 1;
end
