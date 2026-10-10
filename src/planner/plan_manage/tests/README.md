# Mode 2 途中点通过速度（本地试验版）

`fsm/waypoint_continuous=true`（默认）时，途中点采用非零终端速度，
最后一个点采用零速度。`fsm/waypoint_pass_speed` 是直线通过速度上限，
默认 `0` 表示跟随 `manager/max_vel`（网页改 max_vel 时同步生效），
大于 0 时取它与 `max_vel` 的较小值。它约束的是规划边界，不直接下发给底盘。

- 根据入段、出段夹角计算角平分线方向，速度乘以 `cos²(转角/2)`。
  max_vel=0.9 时，30° 弯约 0.84 m/s，90° 弯约 0.45 m/s。
- 根据加速度、转角、到达半径和前后段长进一步限速。
  120° 及以上、掉头、过短段、纯竖直段和最终点采用零速度。
- 楼梯/陡坡保持原来的停点行为：出段或入段（取上一航点到本航点）
  `|dz|/水平长度 > fsm/waypoint_max_slope`（默认 0.15，约 8.5°）时零速。
  第一个点的入段不做坡度判断，避免 odom 与航点 z 的偏差被误判为楼梯。
- 重复点向后查找有效出段，末尾重复点仍按终点处理。
- 重规划保留该航点的终端速度，局部目标也保留非零速度；
  被障碍替换的目标点改为零速目标并重新生成参考轨迹。
- 非共线起止速度时添加段内参考约束点，减少长段参考轨迹偏移。
  实际避障仍由原有局部规划和碰撞检查处理。
- `manager/boundary_aware_time`（mode 2 且开关开启时由 launch 置 true）：
  局部初始多项式的时长按实际起止速度的梯形速度曲线计算。原分配按
  "静止出发、静止到达"估算，两端都要求非零速度时段中会凹下去
  （1.5 m 点距、0.9 m/s 时仿真最低约 0.3 m/s）。终点速度为零的段仍用原分配，
  终点、急转和楼梯的减速过程与原来一致。

不改变到达半径（0.3 m，按 3D 距离判断）、航向误差冻结、急停、避障与终点控制。
连续模式的控制器在非零终端速度轨迹结束后等待下一段最多 0.15 秒，
避免接段时立即发出零速度。等待期间仍执行航向冻结和急停；超时即停车，
最终点的零速轨迹不等待。`continuous_handoff` 仅在 mode 2 且开关开启时启用。
通过速度仅约束规划边界，不保证实际指令严格匀速。
尤其大转角仍可能触发控制器的原地航向对齐，产生平移停顿。

## 本地编译与测试

在工作空间根目录运行（本机使用 focal schroot）：

```bash
schroot -c focal -- bash -c 'source /opt/ros/noetic/setup.bash && catkin_make --pkg scan_planner -DCATKIN_ENABLE_TESTING=ON -j4'
```

在工作空间 `build` 目录运行：

```bash
schroot -c focal -- bash -c 'source /opt/ros/noetic/setup.bash && ctest -R waypoint_velocity_test --output-on-failure'
```

`waypoint_velocity_test.cpp` 使用实际 `PolynomialTraj` 生成器，验证直线与
转弯的非零速度衔接、最终点零速、禁用开关、重复点、短段、坡度/楼梯停点、
无效参数、速度限制、局部目标制动约束，以及按起止速度分配的时长
（含"两端 0.9 m/s 的直线初始多项式无速度凹陷"）。采样 0.31–40 m 长度的参考段，
本地测试中转弯参考最大横向偏移约 0.065 m。
该采样结果不是任意地图或实际 B-spline 的误差上限。

在工作空间根目录运行直线路线闭环仿真（无硬件节点）：

```bash
schroot -c focal -- bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 src/SCAN-Planner/src/planner/plan_manage/tests/waypoint_continuity_sim.py'
```

脚本创建独立的 localhost ROS master（11329），运行规划器、闭环控制器和
项目自带运动学模型，按实机配置（1.5 m 点距、max_vel 0.9）对比开关两档，
检查途中点不减速（连续模式 > 0.8 m/s）及终点停车。本地结果：
旧行为途中点最低指令速度约 0.77 m/s，新版约 0.88–0.90 m/s，终点误差 < 0.01 m。
同样配置下另测了带 30°/90° 弯的路线：途中点按转角减速，90° 处原地转向
两版相同，最大路径偏差 0.124 m（旧版 0.093 m）。
此模型没有 M20S 底盘响应、楼梯动力学和实际定位误差，不证明实机停顿已消除。

在工作空间根目录可单独运行控制器交接测试：

```bash
schroot -c focal -- bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 src/SCAN-Planner/src/planner/plan_manage/tests/controller_handoff_test.py'
```

测试使用独立 localhost master（11339）和真实控制器节点，验证没有下一段时
0.2 秒非零速轨迹在约 0.35 秒后停车（含 0.15 秒等待），零速终点不等待，
急停指令立即打断等待。直线仿真与交接测试运行结束后清理自己的 ROS 进程。

## 切回原行为

通过 `run.launch` 启动时设置 `waypoint_continuous:=false`。
也可在包含 `advanced_param.xml` 时传同名参数，或配置节点私有参数
`fsm/waypoint_continuous=false`（节点初始化时读取，修改后需要重启）。
直接配置节点参数时，还需将控制器的 `continuous_handoff=false`；
使用上述 launch 参数会同步设置两端。
`waypoint_pass_speed` 同样支持 launch 参数传入。
`manager/boundary_aware_time` 由同一组 launch 参数推导；直接配置节点参数时需要单独关闭。

本版本尚未部署到板子，尚未进行 M20S 实机验证。

## 卡住脱困（stuck escape）

有目标且处于 GEN_NEW_TRAJ / REPLAN_TRAJ / EXEC_TRAJ 时，若 `fsm/stuck_timeout`（默认 10 s）内
平移不超过 `fsm/stuck_min_dist`（0.15 m）、转向不超过 `fsm/stuck_min_yaw`（0.5 rad），
并且机身（双圆柱中心连线）到原始障碍的距离小于 `double_cylinder_radius + fsm/escape_margin`，
FSM 进入 `ESCAPE`：在 `fsm/escape_search_radius`（1 m）内找最近的点，要求
目标点间距 ≥ 上述阈值 + `escape_tolerance`、膨胀地图空闲、非未知（`fsm/escape_allow_unknown`），
且直线路径上任一点都不比起点更靠近障碍。找到后发 `/planning/escape_goal`，
`closed_loop_controller` 保持航向，以 `escape_speed`（0.3 m/s）在机体系平移（可后退、可横移）。
到达（`escape_tolerance`）或超时（`escape_timeout`）后回到 GEN_NEW_TRAJ，重新开始卡住计时。
不靠近障碍或找不到可达空闲点时只打印 WARN，原流程继续，计时重新开始。
新 bspline 或 `/planning/stop` 会立即结束脱困平移。只有 `controller_mode:=closed_loop` 时启用。

平移过程中 FSM 以 20 Hz 用实时地图复查剩余路径：沿路径逐点比较当前地图与搜索时快照的机身间距
（封顶在所需间距），任一点下降超过 `fsm/escape_abort_eps`（0.08 m，需大于一个体素）即急停
（`ESCAPE_SAFETY` → EMERGENCY_STOP，fail_safe 后回到 GEN_NEW_TRAJ，仍卡住则 10 s 后按新地图重新搜索）。
逐点而不是取路径最小值，是为了不让起点旁边那面墙把路径后段的闯入"遮住"。
上面的比较两边都从当前位置画线，只能发现地图变化；狗偏离原路线（侧滑、航向漂移、定位跳变）贴向一面
没变的墙时，两边算出同样的值。所以另加一条：当前位置在实时地图、实际航向下的机身间距，与原路线上
同一进度（投影）处规划时的间距比较，低于后者 `escape_abort_eps` 以上同样急停。
`escape_timeout` 应大于 `escape_search_radius / escape_speed`（默认 1 m / 0.3 m/s ≈ 3.3 s < 5 s）。

闭环仿真：`tests/escape_sim.py 5.0 3.0 0.0`（正常脱困）、加 `--intruder`（脱困开始后在目标点后方放一个人，
脱困降速到 0.1 m/s，应在约 0.15 s 内 ESCAPE_SAFETY）、加 `--drift`（左侧 0.45 m 加侧墙，脱困开始后注入朝墙的
横向指令模拟侧滑，直到收到 `/planning/stop`；应报 `Off the escape line` 并在碰墙前停下）。

纯逻辑单测：`ctest -R escape_search_test --output-on-failure`。
