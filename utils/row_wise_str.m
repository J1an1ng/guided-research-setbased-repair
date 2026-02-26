function res = row_wise_str(arr)
    res = arrayfun(@(i) arr_to_str(arr(:, i)), 1:size(arr, 2), 'UniformOutput', false);
    res = res';
end
