function I = atan2(varargin)
    % atan2 - Overloaded atan2 operator for interval
    % ------------- BEGIN CODE --------------
    I1 = varargin{1};
    I2 = varargin{2};
    r1 = atan2(I1.inf, I2.inf);
    r2 = atan2(I1.inf, I2.sup);
    r3 = atan2(I1.sup, I2.inf);
    r4 = atan2(I1.sup, I2.sup);

    if size(I1.inf, 1) == 1 % row vector
        r = [r1; r2; r3; r4];
        d = 1;
    else
        r = [r1 r2 r3 r4];
        d = 2;
    end

    I = interval(min(r, [], d), max(r, [], d));
end

% ------------- END OF CODE --------------
