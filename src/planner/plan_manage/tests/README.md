# Mode 2 途中点通过速度（本地试验版）

`fsm/waypoint_continuous=true`（默认）时，途中点采用非零终端速度，
最后一个点采用零速度。`fsm/waypoint_pass_speed=0.5` 是直线通过速度上限，
还受 `manager/max_vel` 限制，并非对机器狗直接下发固定速度。

- 根据入段、出段夹角计算角平分线方向，速度乘以 `cos²(转角/2)`。
  默认 90° 转弯的终端速度约为 0.25 m/s。
- 根据加速度、转角、到达半径和前后段长进一步限速。
  120° 及以上、掉头、过短段、纯竖直段和最终点采用零速度。
- 重复点向后查找有效出段，末尾重复点仍按终点处理。
- 重规划保留该航点的终端速度，局部目标也保留非零速度；
  被障碍替换的目标点改为零速目标并重新生成参考轨迹。
- 非共线起止速度时添加段内参考约束点，减少长段参考轨迹偏移。
  实际避障仍由原有局部规划和碰撞检查处理。

不改变到达半径（0.3 m）、航向误差冻结、急停、避障与终点控制。
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
转弯的非零速度衔接、最终点零速、禁用开关、重复点、短段、坡度、无效参数、
速度限制以及局部目标制动约束。采样 0.31–40 m 长度的参考段，
本地测试中转弯参考最大横向偏移约 0.065 m。
该采样结果不是任意地图或实际 B-spline 的误差上限。

在工作空间根目录运行直线路线闭环仿真（无硬件节点）：

```bash
schroot -c focal -- bash -c 'source /opt/ros/noetic/setup.bash && source devel/setup.bash && python3 src/SCAN-Planner/src/planner/plan_manage/tests/waypoint_continuity_sim.py'
```

脚本创建独立的 localhost ROS master（11329），运行规划器、闭环控制器和
项目自带运动学模型，对比开关两档，检查途中点非零通过及终点停车。
本地新版在 1.5 m、3 m 途中点附近最低指令速度约为 0.43、0.71 m/s，
终点误差约 0.004 m，最终速度为零。旧版在该理想模型下没有复现现场停顿，
因此这些结果仅验证实现，不证明 M20S 上的停顿已消除。
此模型没有 M20S 底盘响应、楼梯动力学和实际定位误差。

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

本版本尚未部署到板子，尚未进行 M20S 实机验证。
