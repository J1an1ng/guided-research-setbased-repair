# 修改过的文件列表

此文件用于记录当前工作区中修改过/新建的文件。
## 模型改动
models/dynamics/kst.m
models/dynamics/kst_inp_delta.m
models/constraints/KSTInpDeltaWithLagConstraints.m
models/state/kst_trailer/KSTInpDeltaStateOrder.m
primitive/generation/with_repeating_reach/RepeatingReachPerformer.m
generation/perform_reach_for_primitive.m
primitive/Primitive.m

## 控制器和runner改动
controllers/pd/pd_controller_inp_delta.m
controllers/pd/pd_controller_inp_delta_4d.m
runner/primitives_for_edgar/generate_primitives_for_edgar.m
runner/primitives_for_edgar/optimize_control_params_for_edgar.m
runner/primitives_for_edgar/diagnose_subset_relation.m

## 配置和reachability参数
configs/measurement_dist/meas_dist_for_kst_final.m
configs/vehicle_params/create_kst_params_edgar.m
configs/initial_interval_sampling/create_base_initial_sampler_for_hierarchical_for_truck_only.m
models/vehicle_params/KSTParams.m

## CORA本地patch
matlab-misc/cora_extensions/zonotope/
matlab-misc/cora_extensions/interval/jsonencode.m
matlab-misc/cora_extensions/polytope/jsonencode.m