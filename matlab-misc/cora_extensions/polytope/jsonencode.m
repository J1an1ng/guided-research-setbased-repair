function res = jsonencode(P, varargin)
    % Check if P is a polytope object
    if ~isa(P, 'polytope')
        % If not a polytope, use the built-in jsonencode
        res = builtin('jsonencode', P, varargin{:});
        return;
    end

    res.A = num2str(P.A, '%.4f');
    res.b = num2str(P.b, '%.4f');
    res = builtin('jsonencode', res, varargin{:});

end
