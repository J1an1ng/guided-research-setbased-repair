function res = read_primitive_id_set(file_path)
    arr = [];
    if isfile(file_path)
        arr = readlines(file_path);
    end
    if isempty(arr)
        arr = string.empty(0, 2);
    end
    arr = arr(arr ~= "");
    p_ids = arr(1:2:end, 1);
    final_set_ids = arr(2:2:end, 1);
    final_set_ids = arrayfun(@(s) str2double(s), final_set_ids);
    res = dictionary(p_ids, final_set_ids);
end
