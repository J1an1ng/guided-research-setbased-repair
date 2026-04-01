final_set_path = ['/home/sebastian/Documents/Projects/highway-planner', ...
                  '/commonroad-primitive-planning/primitive_generation', ...
                  '/out/obs_hierarchical_single_path_5/reach_results/primitives/final_sets/140.mat'];
final_set = load(final_set_path);
final_set = final_set.p_final_set.cora_set;

k = KSTTrailerStateOrderDefault;
trailer_occ_k = TrailerOccStateOrder();
params = create_kst_trailer_params_1();
occ_comp = TruckOccComputer(k, params);

plot_occupancy_for_set(occ_comp, final_set, true, trailer_occ_k);
