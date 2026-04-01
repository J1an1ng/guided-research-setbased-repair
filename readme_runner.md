# readme_runner
项目运行说明
直接clone整个文件夹到本地在MATLAB中打开generate_primitives_for_edgar.m即可


1. 添加项目路径
使用add_this_project.m, 路径为change_path\add_this_project.m

2. CORA 初始化（每次 MATLAB 会话必须执行）

在每次启动新的 MATLAB 会话后，必须重新运行：installCORA

3. 运行generate_primitives_for_edgar.m

路径为runner\primitives_for_edgar

## 发现以及进度：

现在主要在排 reachable set 生成这条链路，目前大概情况是这样：

之前有几个结构性问题已经解决了：

CORA 里 split 的越界问题（zero-radius 维度导致 generator 数和状态维度不一致）。

i=2时 nonlinear 传播时 generator 爆炸的问题，现在通过 reduction 控制住了。

params.u 和 timeStep 不匹配的问题，现在每一步只传当前输入列。

现在 reachable set 可以正常跑到 i=100，不会中途崩溃了。

不过在 reach 完成之后，进入 occupancy 计算阶段时会报错：
Index in position 1 exceeds array bounds.
OccupancyComputer/compute_accumulated_occupancy

但是我觉得我的方向不是很对....因为我今天生成一个reachable set跑到i=100跑了非常久。

最初报错的位置
cora/contSet/@zonotope/split.m
→ aux_splitOneDim
触发行 G(:,dim)
Error：Index in position 2 exceeds array bounds. Index must not exceed 8.
原因：dim = 9 size(G,2) = 8
split调用链：cora/contDynamics/@contDynamics/private/priv_select.m
这个函数负责：
当 linearization error 超过阈值时，选择一个维度，调用 split() 来把集合分裂
## 线性误差的阈值可调 
## Jacobin矩阵的设置上维度不匹配
## CORA reduce function 
## MATLAB profile
priv_select调用链：cora/contDynamics/@contDynamics/linReach.m
它做的事情：
线性化系统，传播 zonotope，计算 remainder，如果误差太大 → 调 priv_select 分裂集合
linReach调用链：cora/contDynamics/@nonlinearSys/initReach.m
这个函数：初始化 nonlinear reach，设置初始 reachable set，调用 linReach 进行传播
initreach调用链：cora/contDynamics/@contDynamics/reach.m
reach调用：RepeatingReachPerformer.generate
文件：primitive/generation/with_repeating_reach/RepeatingReachPerformer.m

完整调用链：
generate_primitives_for_edgar
    ↓
PrimitiveSetGenerator.start_generation
    ↓
PrimitiveSetGenerator.perform_reach
    ↓
perform_reach_for_primitive
    ↓
RepeatingReachPerformer.generate
    ↓
cora/contDynamics/@contDynamics/reach
    ↓
cora/contDynamics/@nonlinearSys/initReach
    ↓
cora/contDynamics/@contDynamics/linReach
    ↓
cora/contDynamics/@contDynamics/private/priv_select
    ↓
cora/contSet/@zonotope/split
    ↓
aux_splitOneDim   ← 报错位置


## 

最初报错的index error问题可能的根源：box conversion，当 split(Z) 被调用时，它会先执行：Z = zonotope(interval(Z))，把原来的 zonotope变成 axis-aligned box zonotope
如果状态是 n 维，box zonotope 应该有n 个 generator。但是我们已经知道dim 9,10,11,12 radius = 0也就是说：这些维度是 zero radius，即interval lower bound = upper bound，所以这些维度在几何上其实是一个点。当 box zonotope 被构造时，这些维度可能 不会产生 generator，导致state dimension = 12，generator number = 8。但 split 代码还在做：for dim = 1:length(Z.c) dim = 1..12,当 dim = 9 时：G(:,9)就越界了。

priv_select.m 第 35–66 行
1. 生成所有候选Split
Rtmp = split(Rinit.set)
注意这里 没有给 split 指定维度。所以 split.m 会进入 单参数模式：split(Z)
在这种情况下，split 会：
先把 zonotope 转成一个 interval hull（box）
Z = zonotope(interval(Z))
然后对 每个维度都试一次 split，例如如果状态空间是 12 维：
split dimension 1
split dimension 2
...
split dimension 12
所以这一步会产生：12 个候选 split 方案,每个方案对应一个 split 维度。
2. 对每个 split 方案做一次传播测试
接下来 priv_select.m 会做：每个 split 会产生两个子集合,只保留第一个 half-set然后做：linReach propagation，即用这个 split 后的集合继续做 reachable set propagation
3. 计算每个 split 的误差指标
perInd = max(Rtp.error ./ maxError):线性化误差 / 允许误差
这个指标越小，说明这个 split 后的 reachable set approximation 更好。
4. 选出最佳 split 维度
[~, dimForSplit] = min(perfInd): 选择误差指标最小的那个维度。
5. 当Rtp.error > maxError 进行split
if perfInd > 1
    dimForSplit = priv_select(sys,Rstart,params,options);
end

注：maxError 是上层误差容忍度，现在的设置是maxError = 1e6，允许非常大的误差，所以理论上split 很少会发生。
尝试了设置maxError = Inf 时，报错Abort analysis due to reachable set explosion!
（cora_options.maxError = Inf * ones(state_proc.get_state_size(), 1);）
尝试在CORA的split函数中进行修改成Girard reduction：
% Before (line 46):
Z=zonotope(interval(Z));
% After:
Z=reduce(Z,'girard',1);

% Before (line 55):
Z=zonotope(interval(Z));
% After:
Z=reduce(Z,'girard',1);
i=2时报错：CORA error message: Index in position 2 exceeds array bounds. Index must not exceed 11.

## 
Girard order = 1 ≈ Box
Position 2的 output是哪个

Rinit

## 
入口	generate_primitives_for_edgar.m	正确使用 KSTInpDeltaStateOrder(4D)、InpOrderADelta、kst_inp_delta、pd_controller_inp_delta_4d

## 
State/Input 定义	KSTInpDeltaStateOrder.m	SIZE=4, [X=1,Y=2,V=3,PSI=4]，无 DELTA 属性,InpOrderADelta.m	SIZE=2, [DELTA=1,A=2]

## 
dynamic	kst_inp_delta.m	delta = u(i.DELTA) 从输入读，产出 4D 导数

## 
controller	pd_controller_inp_delta_4d.m	只访问 k.X/Y/V/PSI，输出到 inp_order.DELTA 和 inp_order.A

## 
constraints	KSTInpDeltaConstraints.m	i.DELTA 做 input 约束，注释明确写"delta 不是 state"

## 
ref-trajectory	KSTInpTrajTargetPoint.m、KSTInpTrajTargetPointGenerator.m、InpCreatorForADelta.m	只用 k.X/Y/V/PSI 和 inp_order.DELTA/A

## 
trace	lat_pure_pursuit.m	只用 k.X/Y/PSI

## 
close-loop system	create_cont_time_discr_contr_model.m、ControlledModelParamConfig.m、ControlledModelParamValues.m	全部通过 state_order.get_size() / inp_order.SIZE 动态获取维度

## 
state process	StateProcessorForDiscrContrWithDistProc.m.KSTDistProcForRotatedPositionalDists.m用 k.SIZE 计算增广状态，无硬编码维度

## 
reachability	RepeatingReachPerformer.m	维度无关

## 
Primitive 生成	PrimitiveSetGenerator.m、PrimitiveGenerationSettings.m、Primitive.m	维度无关

## 
归一化/采样/边界	norm_kst.m、BoxLikeStateSpaceDivider.m、create_kst_inp_delta_state_bounds.m、create_inp_bounds_for_a_delta.m	均用 get_size() 动态创建

## R_0
R_0 的 12 行（维度）分别对应什么
状态维度 
n=4,n=4（KSTInpDeltaStateOrder.SIZE = 4），经过 process_initial_set 三步处理后得到 3n=12维：
行号	维度	来源
1	X（位置x）	真实状态
2	Y（位置y）	真实状态
3	V（速度）	真实状态
4	PSI（航向角）	真实状态
5	X'（位置x副本）	project(r_0, [1:4, 1:4]) 复制的上一拍测量状态
6	Y'（位置y副本）	同上
7	V'（速度副本）	同上
8	PSI'（航向角副本）	同上
9	meas_dist_X	cartProd(r_0, obj.meas_dist) 拼接的测量噪声
10	meas_dist_Y	同上
11	meas_dist_V	同上
12	meas_dist_PSI	同上

state_meas_dist = zeros(k.get_size(), 1);
obj.meas_dist = zonotope(interval(-zeros(4,1), zeros(4,1))) = 一个零体积的点（退化 zonotope，没有 generators）。
所以 cartProd(r_0_8dim, obj.meas_dist) 只是把维度从 8 扩到 12，但不增加任何新 generators。

## 尝试state_meas_dist 赋非零值