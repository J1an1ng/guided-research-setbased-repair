classdef Primitive < handle & JsonEncodable
    properties (Constant)
        id_properties = ["start_set_id"; "ref_traj"]
    end

    properties (SetAccess = immutable, GetAccess = public)
        start_set SetContainer
        ref_traj AbstractInpTraj
        reachset_dt double
        occ_dt double
    end

    properties (SetAccess = private, GetAccess = public)
        reachsets reachSet
        constraint_violation logical = false
        is_terminal logical = false
        connectable logical = true
        cora_error string = ""
        final_set_id uint32
        final_set SetContainer
        final_set_mirrored logical
        connectable_start_set_id uint32
    end

    properties (Dependent)
        start_set_id uint32
    end

    methods

        function value = get.start_set_id(obj)
            value = obj.start_set.id;
        end

    end

    methods

        function obj = Primitive(start_set, ref_traj, reachset_dt, occ_dt)
            obj.start_set = start_set;
            obj.ref_traj = ref_traj;
            obj.reachset_dt = reachset_dt;
            obj.occ_dt = occ_dt;
        end

        function fragment = to_fragment_for_processing(obj)
            fragment.is_terminal = obj.is_terminal;
            fragment.final_set = obj.final_set;
        end

        function fragment = to_fragment_for_logging(obj)
            fragment.json = obj.json;
            fragment.final_set_id = obj.final_set_id;
        end

        function set_final_set_id(obj, final_set_id)
            assert(isempty(obj.final_set_id), "Final set id already set");
            obj.final_set_id = final_set_id;
        end

        function set_final_set(obj, final_set)
            assert(final_set.id == obj.final_set_id, "Final set id does not match");
            obj.final_set = final_set;
        end

        function res = is_valid(obj)
            res = ~obj.constraint_violation && obj.connectable && isempty(obj.cora_error);
        end

        function apply_reach_result(obj, reach_res, norm_final_set, mirroring, state_order)
            obj.constraint_violation = reach_res.constraint_violation;
            obj.connectable = reach_res.connectable;
            obj.cora_error = reach_res.cora_error;

            if ~obj.is_valid()
                return
            end

            obj.is_terminal = reach_res.terminal;
            obj.reachsets = reach_res.reachsets;

            %% obj.reachsets might be empty if there occurs a CORA error in the
            %% very first reach-call
            final_set_zono = obj.reachsets(end).timePoint.set{end};
            final_set_zono = norm_final_set(final_set_zono, state_order);
            %% The reduction to order 1.5 makes sense according to some
            %% experiments
            % TODO compare PCA with Girard
            % FIXME is reduction to 1.5 really appropriate??
            final_set_zono = reduce(final_set_zono, 'pca', 50);
            [final_set_zono, was_mirrored] = mirroring.mirror(final_set_zono);
            obj.final_set_mirrored = was_mirrored;
            % FIXME implement own converter using PCA or so.
            % Idea: we determine important directions and calc the halfspaces
            % orthogonal to these directions.
            assert(~isempty(obj.final_set_id), "Final set id not set");
            % Hint: we used constrained Zonotopes, as their convHull is tigher than
            % the one of Zonotopes; for zonotopes, the convHull is not even always enclosed
            % by the bounding box of both zonotopes, which is kind of weird.
            % However, the results of convHull on our conZonotopes usually have many more
            % generators than the input zonotopes, which results in really slow operations.
            % Reducing their order does not work (more precisely, invoking reduce does not
            % produce a conZonotope with the desired order, but one with slightly smaller order
            % than the input conZonotope). FML.
            % final_set_con_zono = conZonotope(final_set_zono);
            obj.set_final_set(SetContainer(obj.final_set_id, final_set_zono, false));
        end

        function res = export_errors(obj)
            res.constraint_violation = obj.constraint_violation;
            res.connectable = obj.connectable;
            res.cora_error = obj.cora_error;
        end

        function res = export_to_struct(obj, occ_computer, max_half_width)
            res.start_set_id = obj.start_set_id;

            res.final_set_id = obj.final_set.id;
            res.final_set_mirrored = obj.final_set_mirrored;

            res.ref_traj = obj.ref_traj.export_to_struct();

            res.reachset_dt = obj.reachset_dt;
            res.occ_dt = obj.occ_dt;

            res.is_terminal = obj.is_terminal;

            num_occs_to_merge = obj.occ_dt / obj.reachset_dt;
            assert(floor(num_occs_to_merge) == ceil(num_occs_to_merge));
            [occs, occs_too_large] = reachsets_to_occupancies_vertices(obj.reachsets, ...
                occ_computer, num_occs_to_merge, max_half_width);
            res.occupancies = occs;
            % TODO maybe store this in Primitive itself as well...
            % would require to compute occupancies outside of the Primitive
            % and set the occupancies_too_large property afterwards.
            res.occupancies_too_large = occs_too_large;
        end

    end
end

% TODO this function should be split into stuff regarding the export (creation of vertices...)
% and creation of the rectangle from the occupancy
function [res, occs_too_large] = reachsets_to_occupancies_vertices( ...
        reachsets, occ_computer, num_occs_to_merge, max_half_width)
    occs = reachsets_to_occupancies(reachsets, occ_computer, num_occs_to_merge);
    [num_occs_sets, num_shapes, ~] = size(occs);

    % Dimensiomns: number of occupancies x number of shapes x inf and sup x position (x and y):
    res = zeros(0, num_shapes, 4, 2);

    for i = 1:num_occs_sets
        merged_occs_arr = zeros(num_shapes, 4, 2);
        for k = 1:num_shapes
            [itv_rot, itv_back_rot] = occs{i, k, :};

            half_width = min(rad(itv_rot));
            if half_width > max_half_width
                occs_too_large = true;
            end

            vtcs = vertices(itv_back_rot);
            vtcs = vtcs';
            merged_occs_arr(k, 1, :) = vtcs(1, :);
            merged_occs_arr(k, 2, :) = vtcs(2, :);
            % the vertex order is weird...
            % FIXME sometimes the order is also normal
            merged_occs_arr(k, 3, :) = vtcs(4, :);
            merged_occs_arr(k, 4, :) = vtcs(3, :);
        end
        res(i, :, :, :) = merged_occs_arr;
    end
end
