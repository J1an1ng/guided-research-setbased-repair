function must_be_of_type_or_empty(arg, class_name)
    if ~isempty(arg) && ~isa(arg, class_name)
        error('Input must be either an object of class %s or empty.', class_name);
    end
end
