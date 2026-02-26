function trailer_occ_comp = create_trailer_occ_computer(params, state_order, pos_k)
    trailer_occ_k = TrailerOccStateOrder();

    dist_from_rear_to_hitch = params.trailer_length - params.trailer_dist_from_front_to_hitch;

    % the origin of the trailer is the hitch (even though globally it is the rear axle, but we
    % perform the shift in the function below)
    shape_set = zonotope(interval([-dist_from_rear_to_hitch; -params.trailer_width / 2], ...
                                  [params.trailer_dist_from_front_to_hitch; params.trailer_width / 2]));
    trailer_occ_comp = SingleComponentOccupancyComputer(shape_set, ...
        create_create_trailer_occ(params.truck_dist_from_rear_axle_to_hitch), state_order, trailer_occ_k, pos_k);
end

function g = create_create_trailer_occ(truck_dist_from_rear_axle_to_hitch)

    function f = create_trailer_occ(k, pos_k)

        function res = trailer_occ(state)
            [s, p] = split_state(state, k.get_size());
            psi = s(k.PSI);
            ha = s(k.HA);
            x_occ = p(pos_k.X);
            y_occ = p(pos_k.Y);
            hitch_x = s(k.X) + cos(psi) * truck_dist_from_rear_axle_to_hitch;
            hitch_y = s(k.Y) + sin(psi) * truck_dist_from_rear_axle_to_hitch;
            res(pos_k.X, 1) = hitch_x + cos(psi + ha) * x_occ - sin(psi + ha) * y_occ;
            res(pos_k.Y, 1) = hitch_y + sin(psi + ha) * x_occ + cos(psi + ha) * y_occ;
        end

        f = @trailer_occ;
    end

    g = @create_trailer_occ;
end
