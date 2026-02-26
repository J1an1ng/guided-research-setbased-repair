%% Model settings
model = @kst_trailer;
k = KSTTrailerStateOrderDefault;
i = InpOrderADeltaVDefault;
transform_initial_set = @shift_kst;
norm_final_set = @norm_kst;
mirroring = KSTTrailerMirroring(k);
model_for_ref_traj = prevent_backward_driving(model);

warning('off', 'CORA:zonotopeZDeprecated');
warning('off', 'CORA:redundantOptions');

%% Configs
params = create_kst_trailer_params_2();
state_meas_dist = meas_dist_for_kst_trailer_1(k);
inp_dist = inp_dist_for_a_deltav_1(i);
constraints = KSTTrailerConstraints(k, i, params);
terminal_cond = StandstillTerminalCondition(k, 1);

result_dir = ResultDir('out/opt_control_params_pd_for_long_primitives_7');

interval_sampler = create_initial_sampler_for_kst_with_delta_v_corr_2( ...
    result_dir, 1, k, params, constraints, terminal_cond, state_meas_dist);

%% Set up dynamics
ref_state_builder = SameRefStateBuilder(k);

weights = [34.4; 1187.5; 23.4; 46.9; 162.5];
[weights_size, f] = create_pd_control_weights_fun_with_increase(k, [weights; 1.0]);
lb = [0; 0; 0; 0; 0; 0];
ub = [100; 2000; 50; 500; 400; 5.0];

% control_params = [37.5; 1200; 16.5; 80; 126; 20];
% lb = [0; 0; 0; 0; 0; 0];
% ub = [100; 2000; 60; 100; 300; 100];
% lb = [37.5; 1200; 16.5; 80; 125];
% ub = [37.5; 1200; 16.5; 80; 126];

ref_state_size = ref_state_builder.get_ref_state_size();

dyn_conf = ControlledModelParamConfig(k, i, ...
    [], false, ... % we integrate the measurement uncertainty in the state-processor
    [], false, ...
    inp_dist, true, ...
    ref_state_builder, true, f, weights_size);

create_dynamics = @() nonlinearSys(build_controlled_model( ...
    @discretely_controlled_model_with_observer, model, @pd_controller, ...
    k, i, params, dyn_conf), 3 * k.SIZE, dyn_conf.get_params_size());

dt_controller = 0.01;

primitive_duration = 1.0;
% accs = [-1.0 -3.0];
accs = {-3.0};
% we cannot use the bounds of :constraints as we want to only drive
% forward here
bounds = create_kst_trailer_state_bounds(k, params, true);

occ_comp = TruckOccComputer(k, params);
truck_plotter = KSTTrailerPlotter(occ_comp, k, params);

state_processor_class = @StateProcessorForDiscretelyControlledWithObserver;

occ_comp = TruckOccComputer(k, params);

check_point_file_path = strcat(result_dir.out_path, "/checkpoint.mat");
debug = false;
create_plot = false;
do_random_simulations = false;

settings = PrimitiveGenerationSettings( ...
    result_dir, ...
    create_dynamics, k, i, params, constraints, ...
    transform_initial_set, norm_final_set, ...
    mirroring, ...
    state_meas_dist, inp_dist, [], ...
    dt_controller, 0.1, ...
    terminal_cond, ...
    primitive_duration, ...
    false, false, ... % FIXME set to true at some point
    interval_sampler, ...
    occ_comp, ...
    debug, create_plot, false, false, do_random_simulations);

c = zeros(k.SIZE, 1);

%% TODO use different velocities and target point distance accordingly
c(k.V) = 2.5;
target_point_distance = 2.3;

get_max_zono = create_get_max_zono(constraints, state_meas_dist, k);
itv = get_max_zono(c);
% get_max_itv = create_get_max_itv_kst_2(k, constraints, state_meas_dist(k.DELTA));
% itv = get_max_itv(c);

itv_as_arr = zeros(k.SIZE, 2);
itv_i = interval(itv);
itv_as_arr(:, 1) = itv_i.inf;
itv_as_arr(:, 2) = itv_i.sup;
if ~settings.constraints.check_interval(itv_as_arr) ...
    || settings.terminal_cond.check_for_itv_soft(itv_as_arr)
    error('Invalid interval');
end

inp_traj_gen = KSTInpTrajTargetPointGenerator(settings.k, settings.i, ...
        settings.params, model, bounds, ...
        primitive_duration, accs, target_point_distance, 0.01);
primitives = create_primitive_scaffolds(inp_traj_gen, itv, ...
    settings, model_for_ref_traj); % 6, truck_plotter);

sim = StandardSimulator(create_dynamics(), dyn_conf);
gen = RepeatingReachPerformer(state_processor_class, dyn_conf, sim);

% cost_def = FinalReachsetSizeCost(get_max_itv, settings);
% cost_defs = [RefPathDeviationCost(k), OccupancyCost(settings, occ_comp, 10, [0.1 0.5])];
% RefStateDeviationCost(settings, [1.0; 1.0; 1.0; 1.0; 1.0; 1.0])
% cost_defs = [RefPathDeviationCost(k), FinalReachsetSizeCost(get_max_itv, settings)];
% cost_weights = [100.0 1.0];
% cost_def = CombinedCost(cost_defs, cost_weights);
% cost_def = VerticesRefPathDeviationCost(k, 1.0, 5.0);

% FIXME check
dim_weights = [3.0; 3.0; 0.1; 1.0; 1.0; 1.0];
cost_def = ReachsetVerticesCost(get_max_zono, 5, settings, dim_weights);

opt = SetBasedControlWeightOptimizer(settings, dyn_conf, ...
    @create_pd_control_weights_fun_with_increase, ...
    gen, primitives, lb, ub, check_point_file_path, cost_def);
opt_ctrl_params = opt.optimize();
display(opt_ctrl_params);

% Result 1 - pd controller: 21.0938  398.4375   10.5469  194.5312   84.3750
% Result 2 - pd controller 2: 73.4375  890.6250   24.2188  192.1875  118.7500   23.4375
% Result 3 - pd controller 2: 43.0859  237.8906   14.1211   80.0391  166.0938   28.2422
% Result 4 - pd controller 2: 14.0668  530.8713   16.3888  259.4789   74.5251         0
% Result 4.2 - pd controller 2 = [40.2670; 688.9399; 17.1937; 217.4451; 126.6965; 0];
% Result 5 - pd controller 2 = [17.4589; 507.6565; 19.0683; 299.9930; 97.4165; 0];
% Result 6 - pd controller 2 on multiple primitives = 29.3493  615.8613
% Result 7: 0         0  664.6628   72.1120  167.4441
% 19.7979  147.9185   90.8707    1.4171
% Result 6.Run:  0.1038  194.7735   21.3443  290.8045   81.8774    0.6783
% --> I think the maximum sets were wrong
%  0         0   22.3945  269.5276   93.3618    1.2796
% 5: 0.0001   59.1158   12.2289  299.9034   78.2653    3.2065 --> not too
% bad
% Result - pd controller: 7.2754  720.2148   22.0947  346.9238  113.8672
% Result - pd controller only 1 primitive:  18.7500  312.5000   21.8750  281.2500  175.0000
% Result for 3.0 s: 53.9062  859.3750   14.4531    3.9062  190.6250

% Optimization for 1.0s-primitives; controller with increasing lateral weights;
% for different initial velocities; large initial sets (like max merge sets). Results:
% - 20.0 m/s: weights_1 = [9.5381; 371.4935; 22.1964; 55.7358; 79.7620; 1.7452];
% - 15.0 m/s: weights_2 = [9.5192; 378.4692; 22.3658; 57.8568; 80.2606; 1.7218];
% - 10.0 m/s: weights_3 = [11.8783; 506.3667; 22.3793; 9.9988; 100.3323; 1.5619];
%             [6.7383; 587.8906; 48.0957; 1.4648; 107.4219; 3.6572]
% - 7.5 m/s: weights_4 =

function fun = create_get_max_zono(constraints, state_meas_dist, k)
    load('maximum_set_computation/max_z_16.mat', 'z');
    get_delta_rad_for_v = create_get_delta_rad_for_v(constraints, state_meas_dist(k.DELTA));

    function itv = get_max_zono(c)
        delta_rad = get_delta_rad_for_v(c(k.V));
        res = zeros(2, 2);
        res(1, :) = [-delta_rad, delta_rad];
        res(2, :) = [-0.7, 0.7];
        res = interval(res(:, 1), res(:, 2));
        z_lifted = cartProd(z, res);
        z_lifted_re_orderd = project(z_lifted, [1 2 5 6 3 4]);
        z_lifted_re_orderd = z_lifted_re_orderd + c;
        itv = z_lifted_re_orderd;
    end

    fun = @get_max_zono;
end
