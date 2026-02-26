function v = to_length(v, desired_length)
    v = (v / norm(v)) * desired_length;
end
