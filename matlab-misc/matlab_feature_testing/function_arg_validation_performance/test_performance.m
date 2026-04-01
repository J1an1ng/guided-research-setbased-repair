num = 100000000;

%% With argument validation
fprintf('With argument validation:');
x = SomeClass(4, 5);

tic;
for i = 1:num
    some_fun_with_val(x, 10);
end
toc;

%% With argument validation
fprintf('Without argument validation:');
x = SomeClass(4, 5);

tic;
for i = 1:num
    some_fun_without_val(x, 10);
end
toc;

%% Conclusion: Argument validation does not come for free.
%% --> better do not use it for function that are called extremely often
%%  and where validation is not really necessary but only for code beauty
