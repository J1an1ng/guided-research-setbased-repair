function res = mirror_set(r, mirror_dims)
    v = ones(dim(r), 1);
    v(mirror_dims) = -1;
    m = diag(v);
    res = m * r;
end
