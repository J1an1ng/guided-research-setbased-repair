function check_range(range, step)
    % checks if 'range(1):step:range(2)' will be non-empty
    if sign(range(2) - range(1)) ~= sign(step)
        throw(MEException('WrongRange', 'range or step invalid'));
    end
end
