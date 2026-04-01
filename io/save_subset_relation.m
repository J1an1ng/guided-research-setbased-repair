function save_subset_relation(subset_relation, out_path)
    out_file = strcat(out_path, '/subset_relation.json');

    % necessary for jsonencode:
    keys_as_str = string(subset_relation.keys());
    values_as_str = string(subset_relation.values());
    % jsonencode does not support dictionaries
    subset_relation_map = containers.Map(keys_as_str, values_as_str);

    json = jsonencode(subset_relation_map, PrettyPrint = true);

    if isfile(out_file)
        warning("Should not happen: Overwrite existing json-file...");
    end
    fid = fopen(out_file, 'wt');
    fprintf(fid, json);
    fclose(fid);
end
