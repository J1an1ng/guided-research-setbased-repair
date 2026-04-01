classdef ResultDir < handle
    properties (SetAccess = immutable, GetAccess = public)
        out_path string

        out_path_internal string
        primitive_ids_file_path string
        primitive_scaffolds_out_path string
        reach_log_file_path string
        primitives_final_sets_out_path string
        fig_out_path string
        ref_traj_out_path string

        out_path_export string
        initial_sets_out_path string
        primitives_out_path string
    end

    methods

        function obj = ResultDir(out_path)
            obj.out_path = out_path;

            obj.out_path_internal = strcat(out_path, '/internal');
            [~, ~] = mkdir(obj.out_path_internal);
            obj.fig_out_path = strcat(obj.out_path_internal, "/figs");
            [~, ~] = mkdir(obj.fig_out_path);
            obj.primitive_ids_file_path = strcat(obj.out_path_internal, ...
                "/primitive_ids");
            obj.primitive_scaffolds_out_path = strcat(obj.out_path_internal, ...
                "/primitive_scaffolds");
            [~, ~] = mkdir(obj.primitive_scaffolds_out_path);
            obj.reach_log_file_path = strcat(obj.out_path_internal, "/reach_log");
            [~, ~] = mkdir(obj.reach_log_file_path);
            obj.primitives_final_sets_out_path = strcat(obj.out_path_internal, ...
                "/primitives_final_sets");
            [~, ~] = mkdir(obj.primitives_final_sets_out_path);
            obj.ref_traj_out_path = strcat(obj.out_path_internal, "/ref_traj");
            [~, ~] = mkdir(obj.ref_traj_out_path);

            obj.out_path_export = strcat(out_path, '/export');
            [~, ~] = mkdir(obj.out_path_export);
            obj.initial_sets_out_path = strcat(obj.out_path_export, '/sets');
            [~, ~] = mkdir(obj.initial_sets_out_path);
            obj.primitives_out_path = strcat(obj.out_path_export, '/primitives');
            [~, ~] = mkdir(obj.primitives_out_path);
        end

    end
end
