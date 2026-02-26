classdef CombinedTraj < AbstractInpTraj
    properties (Constant)
        id_properties = ["trajs"]
    end

    properties (SetAccess = private, GetAccess = public)
        trajs (:, 1) cell % AbstractInpTraj
        durations (:, 1) double
    end

    methods

        function obj = CombinedTraj(trajs, durations)
            assert(length(trajs) == length(durations), ...
                'The number of trajectories and durations must match.');
            obj.trajs = trajs;
            obj.durations = durations;

            for i = 2:length(durations)
                assert(durations(i) > durations(i - 1), ...
                    'Durations must be in increasing order.');
            end
        end

        function inp = inp_at(obj, state, t, dt)
            for i = 1:length(obj.trajs)
                if t <= obj.durations(i)
                    inp = obj.trajs{i}.inp_at(state, t, dt);
                    return
                end
            end
            error('Time t is out of bounds.');
        end

    end

end
