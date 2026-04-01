function res = box_arrs2set_cell_arr(boxes, id_manager)
    n = size(boxes, 1);
    res = cell(n, 1);
    for i = 1:n
        b = squeeze(boxes(i, :, :));
        itv = interval(b(:, 1), b(:, 2));
        res{i} = SetContainer(id_manager.gen_next_id(), itv, true);
    end
end
