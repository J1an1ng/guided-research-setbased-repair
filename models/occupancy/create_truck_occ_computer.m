function truck_occ_comp = create_truck_occ_computer(params, state_order, pos_k)
    truck_occ_k = LocalizationStateOrder();
    shape_set = zonotope(interval([-params.l_rear; -params.truck_width / 2], [params.l_front; params.truck_width / 2]));
    truck_occ_comp = SingleComponentOccupancyComputer(shape_set, ...
        @create_truck_occ, state_order, truck_occ_k, pos_k);
end

function f = create_truck_occ(k, pos_k)

    function res = truck_occ(state)
        [s, p] = split_state(state, k.get_size());
        psi = s(k.PSI);
        x_occ = p(pos_k.X);
        y_occ = p(pos_k.Y);
        res(pos_k.X, 1) = s(k.X) + cos(psi) * x_occ - sin(psi) * y_occ;
        res(pos_k.Y, 1) = s(k.Y) + cos(psi) * y_occ + sin(psi) * x_occ;
    end

    f = @truck_occ;
end
