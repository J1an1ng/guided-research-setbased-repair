# readme_runner
项目运行说明

在当前项目中，我使用 add_project 没用。
实际运行方式如下：

1. 添加项目路径

我直接将整个 commonroad-highway-fail-safe-planning 及其subfolders加入 MATLAB 路径

2. CORA 初始化（每次 MATLAB 会话必须执行）

在每次启动新的 MATLAB 会话后，必须重新运行：installCORA

3. 运行generate_primitives_for_edgar.m

路径为runner\primitives_for_edgar