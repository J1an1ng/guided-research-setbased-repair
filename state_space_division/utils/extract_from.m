function res = extract_from(dict, idx, dims)
    values = dict(dims);
    res = cellfun(@(v) v(idx), values);
    res = res';
end
