function res = create_kst_occ_comp(state_order, params)
    pos_k = PositionStateOrder();
    truck_occ_comp = create_truck_occ_computer(params, state_order, pos_k);

    res = OccupancyComputer(truck_occ_comp);
end
