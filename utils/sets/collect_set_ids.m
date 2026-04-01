function res = collect_set_ids(set_containers)
    res = cellfun(@(c) c.id, set_containers);
end
