function res = sign(I)
    % sign - Overloaded sign operator for interval
    % ------------- BEGIN CODE --------------

    res = interval(sign(I.inf), sign(I.sup));

end

% ------------- END OF CODE --------------
