o1 = SubTest();
o2 = SubTest();

tic;
for i = 1:1000000
    o1.test(o2);
end
toc;
