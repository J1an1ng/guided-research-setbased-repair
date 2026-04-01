d_value = some_data_creation_fun("Hallo", "was", "geht");
d_handle = SomeDataClass("Hallo", "was", "geht");

fprintf("With handle argument:");
tic;
for i = 1:1000000
    do_something(d_handle);
end
toc;

fprintf("With value argument:");
tic;
for i = 1:1000000
    do_something(d_value);
end
toc;

arr_value = [1; 2; 3; 4];
arr_handle = SomeArrayClass([1; 2; 3; 4]);
%
fprintf("With array value argument:");
tic;
for i = 1:1000000
    do_something_with_arr(arr_value);
end
toc;
%
fprintf("With array handle argument:");
tic;
for i = 1:1000000
    do_something_with_arr_handle(arr_handle);
end
toc;

function res = do_something(d)
    res = d.a + d.b + d.c;
end

function res = do_something_with_arr(arr)
    res = sum(arr);
end

function res = do_something_with_arr_handle(arr)
    res = sum(arr.a);
end

%% Result: passing an argument as handle is much faster
