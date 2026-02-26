function cell_arr = remove_empty_values(cell_arr)
    cell_arr = cell_arr(~cellfun(@isempty, cell_arr));
end
