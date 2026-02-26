classdef FinalReachsetSizeCost < CostDef
    properties (Access = private)
        get_max_itv
        settings PrimitiveGenerationSettings
    end

    methods

        function obj = FinalReachsetSizeCost(get_max_itv, settings)
            obj.get_max_itv = get_max_itv;
            obj.settings = settings;
        end

        function cost = evaluate(obj, ~, reach_res)
            r_final = reach_res.reachsets(end).timePoint.set{end};
            %% Hint: the conversion to an interval is essential, as rad is only defined for
            %% intervals in CORA; also, check that not the yalmip of rad implementation is called
            r_final = interval(obj.settings.norm_final_set(r_final, obj.settings.k));
            max_final_itv = interval(obj.get_max_itv(center(r_final)));
            relative_rads = rad(r_final) ./ rad(max_final_itv);
            cost = mean(arrayfun(@err_fun, relative_rads));
        end

    end
end

function err = err_fun(x)
    err = exp(2 * x);
end
