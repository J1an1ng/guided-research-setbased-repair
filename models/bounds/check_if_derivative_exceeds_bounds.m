function res = check_if_derivative_exceeds_bounds(bound_check_res, derivative)
    res = bound_check_res * sign(derivative) == 1;
end
