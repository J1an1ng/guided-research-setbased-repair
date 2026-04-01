classdef SetTransformer < handle
    properties (Access = private)
        fun
        A_fun
        Q_fun
        T_fun
    end

    methods

        function obj = SetTransformer(f, x)
            [obj.fun, obj.A_fun, obj.Q_fun, obj.T_fun] = compute_derivatives(f, x);
        end

        function res = transform(obj, X)
            p = center(X);

            [f, A, Q, T] = eval_derivatives(X, p, obj.fun, obj.A_fun, obj.Q_fun, obj.T_fun);

            % compute Largrange remainder
            rem = lagrange_remainder(X, p, T);

            % compute over-approximating zonotope
            res = f + A * (X + (-p)) + 0.5 * quadMap(X + (-p), Q) + rem;
        end

    end
end

function [fun, A_fun, Q_fun, T_fun] = compute_derivatives(f, x)
    % compute the symbolic derivatives of the function

    % function handle for the nonlinear function
    fun =  matlabFunction(f, 'Vars', {x});

    A = jacobian(f, x);
    A_fun =  matlabFunction(A, 'Vars', {x});

    Q_fun = cell(length(f), 1);
    for i = 1:length(f)
        temp = hessian(f(i), x);
        Q_fun{i} =  matlabFunction(temp, 'Vars', {x});
    end

    T_fun = cell(size(A));
    for i = 1:size(A, 1)
        for j = 1:size(A, 2)
            temp = hessian(A(i, j), x);
            T_fun{i, j} = matlabFunction(temp, 'Vars', {x});
        end
    end
end

function [f, A, Q, T] = eval_derivatives(X, p, fun, A_fun, Q_fun, T_fun)
    % evaluate the derivatives at the linearization point

    % interval encluore of the set
    int = interval(X);

    f = fun(p);
    A = A_fun(p);

    Q = cell(length(f), 1);
    for i = 1:length(f)
        fun_han = Q_fun{i};
        Q{i} = fun_han(p);
    end

    T = cell(size(A));
    for i = 1:size(A, 1)
        for j = 1:size(A, 2)
            fun_han = T_fun{i, j};
            T{i, j} = fun_han(int);
        end
    end
end

function rem = lagrange_remainder(X, p, T)
    % comptute the lagrange remainder of the Taylor series

    % interval enclousre of the shifted initial set
    int = interval(X) - p;

    % Lagrange remainder term
    rem = interval(zeros(size(T, 1), 1));

    for i = 1:size(T, 1)
        for j = 1:size(T, 2)
            rem(i) = rem(i) + int(j) * transpose(int) * T{i, j} * int;
        end
    end

    % convert to zonotope
    rem = zonotope(1 / 6 * rem);
end
