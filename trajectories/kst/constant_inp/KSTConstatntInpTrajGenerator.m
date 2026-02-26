classdef KSTConstatntInpTrajGenerator < AbstractInpTrajGenerator
    properties (Access = private)
        state_order KSTStateOrder
        inp_order InpOrderADeltaV

        sampler Sampler
    end

    methods

        function obj = KSTConstatntInpTrajGenerator(state_order, inp_order, ...
                dv_range, dv_step, a_range, a_step)
            check_range(dv_range, dv_step);
            check_range(a_range, a_step);

            obj.state_order = state_order;
            obj.inp_order = inp_order;

            dv_values = dv_range(1):dv_step:dv_range(2);
            a_values = a_range(1):a_step:a_range(2);
            obj.sampler = Sampler({ a_values; dv_values});
        end

        function trajectories = generate(obj, ~)
            inputs = obj.sampler.sample_all();
            trajectories = cellfun(@(inp) {KSTConstantInpTrajADeltaV( ...
                obj.state_order, obj.inp_order, inp(1), inp(2))}, ...
                inputs);
        end

    end
end
