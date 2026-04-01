classdef ConstantReachPerformer < handle
    properties
        constant_set % any cora set
        final_constant_set % any cora set
    end

    methods

        function obj = ConstantReachPerformer(constant_set, final_constant_set)
            assert(all(center(constant_set) == 0));
            assert(all(center(final_constant_set) == 0));
            obj.constant_set = constant_set;
            obj.final_constant_set = final_constant_set;
        end

        function res = generate(obj, settings, p, ~, ~, ~)
            res = [];
            res.terminal = false;
            % has to be considered for defining final_constant_set:
            res.connectable = true;
            res.constraint_violation = false;
            res.reachsets = [];
            res.cora_error = [];

            traj_length = size(p.ref_traj.inputs, 2);

            reachsets = cell(traj_length, 1);

            r_before = p.start_set.cora_set;

            for i = 1:traj_length
                t_start = p.ref_traj.times(i);
                t_end = t_start + settings.dt_control;
                if i < traj_length
                    assert(abs(t_end - p.ref_traj.times(i + 1)) < 1e-6);
                end
                t_itv = interval(t_start, t_end);

                s_end = p.ref_traj.states(:, i + 1);

                if i == traj_length
                    r_end = s_end + obj.final_constant_set;
                else
                    r_end = s_end + obj.constant_set;
                end

                r_obj = [];
                r_obj.timeInterval.time = {t_itv};
                r_obj.timeInterval.set = {r_before | r_end};
                r_obj.timePoint.time = {t_start; t_end};
                r_obj.timePoint.set = {r_before; r_end};
                r_obj = reachSet(r_obj.timePoint, r_obj.timeInterval);

                reachsets{i} = r_obj;
                r_before = r_end;
            end

            reachsets = [reachsets{:}]';

            res.reachsets = reachsets;

            final_set = reachsets(end).timePoint.set{end};

            if settings.terminal_cond.check_for_set_soft(final_set)
                res.terminal = true;
            end

            if settings.create_plot
                plot_primitive(settings, p, reachsets, []);
                plot_occupancy_and_save(settings, p, reachsets, 30);
            end
        end

    end
end
