function data = round_all_numbers_in_struct(data, num_decimals)
    if isstruct(data)
        fields = fieldnames(data);
        for i = 1:numel(fields)
            field = fields{i};
            data.(field) = round_all_numbers_in_struct(data.(field), num_decimals);
        end
    elseif iscell(data)
        data = cellfun(@(x) formatStruct(x, num_decimals), data, 'UniformOutput', false);
    elseif isnumeric(data) && isa(data, 'double')
        data = round(data, num_decimals);
    end
end
