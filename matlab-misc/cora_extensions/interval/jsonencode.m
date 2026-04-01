function res = jsonencode(I, varargin)

    res.inf = num2str(I.inf, '%.4f');
    res.sup = num2str(I.sup, '%.4f');
    res = jsonencode(res);

end
