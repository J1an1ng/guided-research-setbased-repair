function sets = create_2d_sets(n, r, set_id_manager)
    if ~exist('set_id_manager', 'var')
        set_id_manager = SetContainerIdManager();
    end
    sets = dictionary(uint32.empty, SetContainer.empty);
    for i = 1:n
        for j = 1:n
            c = [j; i];
            s = interval(c - r, c + r);
            set_id = set_id_manager.gen_next_id();
            sets(set_id) = SetContainer(set_id, s, true);
        end
    end
end
