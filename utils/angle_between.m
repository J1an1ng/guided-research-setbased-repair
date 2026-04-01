function angle = angle_between(a, b)
    angle = acos(dot(a, b) / (norm(a) * norm(b)));
end
