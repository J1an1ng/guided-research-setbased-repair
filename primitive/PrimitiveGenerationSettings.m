classdef PrimitiveGenerationSettings < handle
    properties (GetAccess = public, SetAccess = private)
        result_dir {must_be_of_type_or_empty(result_dir, 'ResultDir')}

        create_dynamics
        k {must_be_of_type_or_empty(k, 'AbstractStateOrder')}
        i {must_be_of_type_or_empty(i, 'AbstractInpOrder')}
        params
        max_half_width double
        constraints {must_be_of_type_or_empty(constraints, 'AbstractConstraints')}
        norm_final_set
        mirroring {must_be_of_type_or_empty(mirroring, 'Mirroring')}

        state_meas_dist (:, 1)
        input_dist (:, 1)
        model_dists (:, 1)

        dt_control double
        time_decimal_places uint32
        occ_dt double

        terminal_cond {must_be_of_type_or_empty(terminal_cond, 'AbstractTerminalCondition')}

        primitive_duration double

        do_connectivity_check logical
        check_constraints logical

        extended_horizon double
        process_final_state

        state_space_divider {must_be_of_type_or_empty(state_space_divider, 'AbstractStateSpaceDivider')}

        debug {mustBeNumericOrLogical}

        create_plot logical
        show_plot logical
        save_plot logical
        do_random_simulations logical

        save_to_mat logical
        export_to_cr logical
        num_primitives_per_file

        occ_comp {must_be_of_type_or_empty(occ_comp, 'OccupancyComputer')}

        cora_options
    end

    methods

        function obj = PrimitiveGenerationSettings( ...
                result_dir, ...
                create_dynamics, state_order, inp_order, params, ...
                max_half_width, ...
                constraints, ...
                norm_final_set, ...
                mirroring, ...
                state_meas_dist, input_dist, model_dists, ...
                dt_control, time_decimal_places, occ_dt, ...
                terminal_cond, ...
                primitive_duration, ...
                do_connectivity_check, check_constraints, ...
                state_space_divider, ...
                occ_comp, ...
                debug, create_plot, show_plot, save_plot, do_random_simulations, ...
                extended_horizon, process_final_state, ...
                cora_options)
            if ~exist('cora_options', 'var')
                fprintf("No cora options specified. Taking default.");
                cora_options = create_cora_options_1();
            end

            obj.result_dir = result_dir;

            obj.create_dynamics = create_dynamics;
            obj.k = state_order;
            obj.i = inp_order;
            obj.params = params;
            obj.max_half_width = max_half_width;
            obj.constraints = constraints;
            obj.norm_final_set = norm_final_set;
            obj.mirroring = mirroring;

            obj.state_meas_dist = state_meas_dist;
            obj.input_dist = input_dist;
            obj.model_dists = model_dists;

            obj.dt_control = dt_control;
            obj.time_decimal_places = time_decimal_places;
            obj.occ_dt = occ_dt;

            obj.terminal_cond = terminal_cond;

            obj.primitive_duration = primitive_duration;

            obj.do_connectivity_check = do_connectivity_check;
            obj.check_constraints = check_constraints;

            obj.state_space_divider = state_space_divider;

            obj.occ_comp = occ_comp;
            obj.debug = debug;

            obj.create_plot = create_plot;
            obj.show_plot = show_plot;
            obj.save_plot = save_plot;
            obj.do_random_simulations = do_random_simulations;

            obj.cora_options = cora_options;

            if ~exist("extended_horizon", "var")
                extended_horizon = 0;
                process_final_state = 0;
            end

            obj.extended_horizon = extended_horizon;
            obj.process_final_state = process_final_state;
        end

    end
end
