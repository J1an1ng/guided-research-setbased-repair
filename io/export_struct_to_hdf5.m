function export_struct_to_hdf5(hdf5_file, hdf5_fid, data, parent_path)
    % Recursive function to export a nested struct to an HDF5 file
    % data       : The struct to be exported
    % hdf5_file   : The name of the HDF5 file
    % parent_path: The path in the HDF5 file where the struct should be stored
    if ~exist('parent_path', 'var')
        parent_path = '/';
    end

    fields = fieldnames(data);

    for i = 1:numel(fields)
        field_name = fields{i};
        field_value = data.(field_name);
        current_path = [parent_path field_name];

        if isstruct(field_value)
            gid = H5G.create(hdf5_fid, current_path, "H5P_DEFAULT", "H5P_DEFAULT", "H5P_DEFAULT");
            % Recursively call the function for the nested struct
            export_struct_to_hdf5(hdf5_file, hdf5_fid, field_value, [current_path '/']);
            H5G.close(gid);
        elseif islogical(field_value)
            h5create(hdf5_file, current_path, size(field_value), 'Datatype', 'uint8');
            h5write(hdf5_file, current_path, uint8(field_value));
        elseif isnumeric(field_value)
            % we reverse the order of the array as HDF5 assumes row-major order
            % Assuming A is your n-dimensional array
            dims = ndims(field_value);
            dim_order = 1:dims;
            reversed_order = fliplr(dim_order);
            field_value_rev = permute(field_value, reversed_order);

            h5create(hdf5_file, current_path, size(field_value_rev));
            h5write(hdf5_file, current_path, field_value_rev);
        else
            error('Unsupported data type in struct field: %s', field_name);
        end
    end
end
