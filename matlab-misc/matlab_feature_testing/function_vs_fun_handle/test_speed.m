fun_handle = create_some_fun(3);
num = 10000000;

fprintf('Call of normal function: \n');
r = 0;
tic;
for i = 1:num
    r = r + log(some_fun(3, i));
end
toc;

fprintf('Call of function handle created by function: \n');
r = 0;
tic;
for i = 1:num
    r = r + log(fun_handle(i));
end
toc;

%% Result: call via handle is slower. The difference is smaller if
%% the handle-function does not use variables of the outer context.
