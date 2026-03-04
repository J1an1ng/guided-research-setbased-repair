# readme_runner
项目运行说明
直接clone整个文件夹到本地在MATLAB中打开generate_primitives_for_edgar.m即可

在当前项目中，我使用 add_project 没用。
实际运行方式如下：

1. 添加项目路径

我直接将整个 commonroad-highway-fail-safe-planning 及其subfolders加入 MATLAB 路径（但是这个repo里我只上传了commonroad-highway-fail-safe-planning/matlab_primitive_generation，可能需要把这个repo里所有的文件夹都添加到路径才能正常跑）

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