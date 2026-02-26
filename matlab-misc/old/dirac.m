function res = dirac(varargin)
    % dirac - Overloaded dirac operator for interval
    % ------------- BEGIN CODE --------------
    if nargin == 1
        I = varargin{1};
        inf_new = dirac(I.inf);
        sup_new = dirac(I.sup);
        res = interval(inf_new, sup_new);
    else
        n = varargin{1};
        I = varargin{2};
        res = interval(dirac(n, I.inf), dirac(n, I.sup));
    end
end

% ------------- END OF CODE --------------
