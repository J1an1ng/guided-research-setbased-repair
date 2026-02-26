%% Model settings
% model = @kst_trailer;
model = @kst_flexible_trailer;
k = KSTTrailerStateOrder();
i = InpOrderADeltaV();
transform_initial_set = @shift_kst;
norm_final_set = @norm_kst;
mirroring = KSTTrailerMirroring(k);
model_for_ref_traj = prevent_backward_driving(model);

%% Configs
params = create_kst_trailer_params_MAN();

state_meas_dist = meas_dist_for_kst_trailer_1(k);
inp_dist = inp_dist_for_a_deltav_1(i);
constraints = KSTTrailerConstraints(k, i, params);
terminal_cond = StandstillTerminalCondition(k, 1);

result_dir = ResultDir('out/single_primitive_X');

ref_state_builder = SameRefStateBuilder(k);
%% weights = [34.4; 1187.5; 23.4; 46.9; 162.5];
%%% weights = [0.5104; 197.6734; 8.4568; 283.7780; 47.6594; 3.7754];
%% weights = [1.31; 393.5464; 17.1167; 399.3327; 110.8457; 1.1494];

weights_1 = [9.5381; 371.4935; 22.1964; 55.7358; 79.7620; 1.7452];
% weights_2 = [9.5192; 378.4692; 22.3658; 57.8568; 80.2606; 1.7218];
% weights_3 = [11.8783; 506.3667; 22.3793; 9.9988; 100.3323; 1.5619];
% weights_3 = [7.0312; 421.8750; 47.2656; 175.7812; 128.1250; 0.6641];
% weights_4 = [0.6836; 154.2969; 21.2402; 484.8633; 122.2656; 0.8545];
% weights_5 = [7.0312; 421.8750; 47.2656; 175.7812; 128.1250; 0.6641];
% weights_6 = [1.5869; 91.3086; 28.2349; 341.1865; 106.7383; 0.9412];
[weights_size, f] = create_pd_control_weights_fun_with_increase(k, weights_1);
%% weights_size = 5;
%% weights = [53.9062; 859.3750; 14.4531; 3.9062; 190.6250];
%% weights = [34.4; 1187.5; 23.4; 46.9; 162.5];
dyn_conf = ControlledModelParamConfig(k, i, ...
    [], false, ... % we integrate the measurement uncertainty in the state-processor
    [], false, ...
    inp_dist, true, ...
    ref_state_builder, true, f, weights_size);

ref_state_size = ref_state_builder.get_ref_state_size();
create_dynamics = @() nonlinearSys(build_controlled_model( ...
    @discretely_controlled_model_with_observer, model, @pd_controller, ...
    k, i, params, dyn_conf), 3 * k.get_size(), dyn_conf.get_params_size());

dt_controller = 0.01;

primitive_duration = 1.0;
%% FIXME
target_point_distance = 2.0; % inf;

%% FIXME
acc_max = -3.0; % FIXME -1.0
accs = {acc_max, [-1.0 -1.5 -2.0 -2.5 -3.0]};
% we cannot use the bounds of :constraints as we want to only drive
% forward here
bounds = create_kst_trailer_state_bounds(k, params, 1);

inp_creator = InpCreatorForADeltaV(k, i, params, 0.01);

inp_traj_gen = KSTInpTrajTargetPointGenerator(k, i, params, ...
    constraints, inp_creator, ...
    model, primitive_duration, accs, target_point_distance, false);

num_workers = 1;

c = zeros(k.get_size(), 1);
c(k.V) = 20.0;
% c(k.V) = 2.5;
% FIXME
% c(k.HA) = 0.1;

load('maximum_set_computation/max_z_16.mat', 'z');

itv = interval(z);
itv = 0.6 * (itv - center(itv)) + center(itv);

get_delta_rad_for_v = create_get_delta_rad_for_v(constraints, state_meas_dist(k.DELTA));

delta_rad = get_delta_rad_for_v(c(k.V));
%% delta_rad = get_delta_range_for_v(15.0);

res = zeros(2, 2);
res(1, :) = [-delta_rad, delta_rad];
res(2, :) = [-0.7, 0.7];
res = interval(res(:, 1), res(:, 2));
itv_lifted = cartProd(itv, res);
itv_lifted_re_orderd = project(itv_lifted, [1 2 5 6 3 4]);
itv_lifted_re_orderd = itv_lifted_re_orderd + c;
itv = itv_lifted_re_orderd;

a = itv.inf;
b = itv.sup;

a(k.X) = -state_meas_dist(k.X);
b(k.X) = state_meas_dist(k.X);

a(k.Y) = -2 * state_meas_dist(k.Y);
b(k.Y) = 2 * state_meas_dist(k.Y);

a(k.PSI) = -state_meas_dist(k.PSI);
b(k.PSI) = state_meas_dist(k.PSI);

% itv = interval(a, b);

%% TODO: we test if the maximum merge set is a sensible choice to start further primitive generations from
%% f = create_max_merge_set_getter_for_kst(k, constraints);
%% itv = f([0; 0; 0; 3; 0; 0]);

%% FIXME
% itv = 0.5*itv;

initial_set = SetContainer(1, itv, false); %% true actually

state_processor_class = @StateProcessorForDiscretelyControlledWithObserver;

occ_comp = create_kst_trailer_occ_comp(k, params);
max_half_width = params.max_road_width / 2;

debug = true;
create_plot = true;
show_plot = true;
do_random_simulations = true;

dummy_state_space_divider = create_base_initial_sampler_for_hierarchical_single_path( ...
    result_dir, num_workers, k, params, constraints, terminal_cond, state_meas_dist);

settings = PrimitiveGenerationSettings( ...
    result_dir, ...
    create_dynamics, k, i, params, ...
    max_half_width, ...
    constraints, ...
    transform_initial_set, norm_final_set, ...
    mirroring, ...
    state_meas_dist, inp_dist, [], ...
    dt_controller, 0.1, ...
    terminal_cond, ...
    primitive_duration, ...
    false, false, ...
    dummy_state_space_divider, ...
    occ_comp, ...
    debug, create_plot, show_plot, false, do_random_simulations);

traj_gen = ValidInpTrajGenerator(inp_traj_gen, settings.terminal_cond, ...
        settings.primitive_duration, ...
        settings.dt_control, ...
        settings.time_decimal_places, ...
        model_for_ref_traj, settings.k, settings.i, params, ...
        settings.constraints, ...
        settings.extended_horizon, settings.process_final_state);

sim = WorstCaseKSTYSimulator(create_dynamics(), dyn_conf, state_meas_dist, [], inp_dist, k, i);
% sim = StandardSimulator(create_dynamics(), dyn_conf);

reach_performer = RepeatingReachPerformer(state_processor_class, dyn_conf, sim);

sub_out_path = strcat(result_dir.out_path_internal, "/batch_1");
mkdir(sub_out_path);

p = build_primitive_scaffolds_for_start_set(initial_set, sub_out_path, settings, traj_gen);

figure;
hold on;
for i = 1:length(p)
    pr = p{i};
    plot(pr.ref_traj.states(1, :), pr.ref_traj.states(2, :));
end

fprintf('Number of primitives: %d\n', length(p));

%% FIXME
p = p(1);

p{1}.set_final_set_id(10);

q = parallel.pool.DataQueue;
p{1} = perform_reach_for_primitive(p{1}, settings, ...
  reach_performer, dictionary(uint32.empty, uint32.empty), q);

% cr = p{1}.export_to_struct(occ_comp);

% if ~p{1}.is_valid()
%    error('Primitive is not valid');
% end

% figure;
% plot_occupancy(settings, p{1}, p{1}.reachsets, 10);
