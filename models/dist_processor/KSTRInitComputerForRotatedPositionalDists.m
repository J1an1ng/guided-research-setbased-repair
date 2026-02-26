classdef KSTRInitComputerForRotatedPositionalDists < AbstractRInitComputer
    properties (Access = private)
        state_order AbstractKSTStateOrder
        localization_state_order LocalizationStateOrder
        localization_dims (:, 1) uint32
        other_dims (:, 1) uint32
        set_transformer SetTransformer
    end

    methods (Access = public)

        function obj = KSTRInitComputerForRotatedPositionalDists(state_order)
            obj.state_order = state_order;
            obj.localization_state_order = LocalizationStateOrder();
            obj.localization_dims = obj.localization_state_order.create_state_order_mapping(obj.state_order);
            obj.other_dims = setdiff(1:obj.state_order.get_size(), obj.localization_dims);

            local_size = obj.localization_state_order.get_size();
            syms symb_state [local_size 1];
            obj.set_transformer = SetTransformer(obj.transform(symb_state), symb_state);
        end

        function r_init = compute_initial_set(obj, measured_state, meas_dist_set)
            new_center = zeros(obj.localization_state_order.get_size(), 1);
            new_center(obj.localization_state_order.PSI) = measured_state(obj.state_order.PSI);

            % Hint: the following works as the psi disturbance is zero-centered (i.e., we get the set of possible values for actual psi)
            pos_meas_dist_and_psi_set = new_center + (-project(meas_dist_set, obj.localization_dims));
            pos_meas_dist_rot_and_psi_set = obj.set_transformer.transform(pos_meas_dist_and_psi_set);

            measured_pos_and_zero_psi = zeros(obj.localization_state_order.get_size(), 1);
            measured_pos_and_zero_psi([obj.localization_state_order.X obj.localization_state_order.Y]) = ...
                measured_state([obj.state_order.X obj.state_order.Y]);

            r_init_localization_dims = measured_pos_and_zero_psi + pos_meas_dist_rot_and_psi_set;
            r_init_other_dims = measured_state(obj.other_dims) + (-project(meas_dist_set, obj.other_dims));

            r_init = cartProd(r_init_localization_dims, r_init_other_dims);

            dims_mapping = obj.state_order.revert_state_order_mapping([obj.localization_dims obj.other_dims]);

            r_init = project(r_init, dims_mapping);
        end

    end

    methods (Access = private)

        function res = transform(obj, pos_meas_dist_and_psi)
            pos_meas_dist = pos_meas_dist_and_psi([obj.localization_state_order.X obj.localization_state_order.Y]);
            psi = pos_meas_dist_and_psi(obj.localization_state_order.PSI);
            pos_meas_dist_rot = rot(psi) * pos_meas_dist;
            res = [pos_meas_dist_rot; psi];
        end

    end
end
