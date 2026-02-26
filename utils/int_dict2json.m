function res = int_dict2json(dict)
    dict_map = containers.Map(string(dict.keys()), dict.values());
    res = jsonencode(dict_map);
end
