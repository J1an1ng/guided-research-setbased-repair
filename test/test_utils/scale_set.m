function res = scale_set(s, scale, set_id)
    c = center(s.cora_set);
    new_cora_set = scale * (s.cora_set - c) + c;
    res = SetContainer(set_id, new_cora_set, false);
end
