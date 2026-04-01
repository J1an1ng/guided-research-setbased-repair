function res = jsonencode(P, varargin)

    res.A = num2str(P.A, '%.4f');
    res.b = num2str(P.b, '%.4f');
    res = jsonencode(res);

end
