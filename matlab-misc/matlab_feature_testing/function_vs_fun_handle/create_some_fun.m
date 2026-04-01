function f = create_some_fun(some_data)
    some_data = some_data * some_data;
    f = @(x) 10 * x + 1 + some_data;
end
