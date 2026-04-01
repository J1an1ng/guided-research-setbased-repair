function r = some_fun_with_val(a, b)
    arguments
        a SomeClass
        b (:, :) double
    end

    r = a.a + b;
end
