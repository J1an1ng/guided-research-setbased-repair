c = ClassWithDepProp();

fprintf('1.Call: \n');
tic;
fprintf("Result: %d", c.y);
toc;

fprintf('2.Call (fast if dependent prop is cached): \n');
tic;
fprintf("Result: %d", c.y);
toc;

c.x = 10;

fprintf('3.Call (should be slow again as dependent prop needs to be re-computed): \n');
tic;
fprintf("Result: %d", c.y);
toc;

%% Result: dependent properties are apparently not cached.
