function res = jsonencode(I, varargin)
    % Check if I is an interval object
    if ~isa(I, 'interval')
        % If not an interval, use the built-in jsonencode
        res = builtin('jsonencode', I, varargin{:});
        return;
    end

    res.inf = num2str(I.inf, '%.4f');
    res.sup = num2str(I.sup, '%.4f');
    res = builtin('jsonencode', res, varargin{:});

end
