%% Testing speed of constant access in Matlab.
%% From: https://stackoverflow.com/questions/53415665/performance-of-matlabs-class-constants
function speed_tests()
    tic;
    for i = 1:200000
        direct_constant_access(1, 2);
    end
    fprintf('Direct constant access: ');
    toc;

    tic;
    c = Consts();
    for i = 1:200000
        passing_extra_obj(1, 2, c);
    end
    fprintf('Passing extra object: ');
    toc;

    tic;
    for i = 1:200000
        persistent_constants(1, 2);
    end
    fprintf('Persistent constants: ');
    toc;

    % Let's assume this code is executed at some point externally:
    % global A B;
    % A = 0.5;
    % B = 3;
    tic;
    for i = 1:200000
        defined_globally(1, 2);
    end
    fprintf('Defined globally: ');
    toc;

    tic;
    for i = 1:200000
        hardcoded(1, 2);
    end
    fprintf('Hardcoded: ');
    toc;

    tic;
    for i = 1:200000
        hardcoded_v2(1, 2);
    end
    fprintf('Hardcoded v2: ');
    toc;
end

function val = direct_constant_access(a, b)
    val = (a + Consts.A)^2 + log(b * Consts.B);
end

function val = passing_extra_obj(a, b, obj)
    val = (a + obj.variable_a)^2 + log(b * obj.variable_b);
end

function val = persistent_constants(a, b)
    persistent A B
    if isempty(A)
        A = Consts.A^2;
        B = Consts.B;
    end
    val = (a + A)^2 + log(b * B);
end

function val = defined_globally(a, b)
    global A B
    val = (a + A)^2 + log(b * B);
end

function val = hardcoded(a, b)
    val = (a + 0.5)^2 + log(b * 3);
end

function val = hardcoded_v2(a, b)
    A = 0.5;
    B = 3;
    val = (a + A)^2 + log(b * B);
end
