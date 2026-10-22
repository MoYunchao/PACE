# PACE

PACE 主方法的 ROS 2 推理与 Gazebo 探索工作空间，包含预测返航和单容量共享充电协调。

本版本由 `main_charging_station/SenseSimulation_fix_v2/SenseSimulation_fix` 整理，
保留三个场景（30×30、40×40、室内地图）和当前默认 MAPPO 检查点。
不包含 ACE、Bandwidth-MRE、Burgard 等对比方法的策略、模型，以及旧 MAPPO 检查点、
训练代码、实验结果、构建产物或嵌套 Git 历史。
独立新增的四维信息消融见 [FEATURE_ABLATION.md](FEATURE_ABLATION.md)。

原目录中的分步启动脚本、`start_all.sh`、`start_all_greedy.sh`、
`start_all_mappo_linear.sh`，以及 `tools/`、`scripts/` 工具已恢复，
对应的 launch 文件一并保留。恢复的脚本保持原样；原版说明文件中的历史路径和
参数说明也未改写。新增的实验记录功能仅由对应的新入口启用。

## 环境与构建

使用已配置的 Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 环境。
运行 Python 必须能同时导入 ROS 2 Python 包与 PyTorch、NumPy、PyYAML。
导航依赖 Nav2（含 MPPI 与 Smac planner）、SLAM Toolbox、Gazebo ROS、xacro 等。
本仓库内保留了跟随间隙恢复、激光仿真、点云转换等源码依赖。

Livox 仿真插件依赖 `livox_ros_driver2` 消息，因此保留该驱动源码及原有许可证。
驱动构建还要求系统安装 Livox-SDK2 **头文件与库**；目录中的 `.so` 单独存在不代表
SDK 已安装。安装说明见 [驱动文档](src/livox_ros_driver2_humble/README.md)。
CMakeLists 保持原版，包括原有的库版本设置。

```bash
source /opt/ros/humble/setup.bash
# 若使用已有的 torch 环境，请同时确保该环境能导入 rclpy。
bash toBuild.sh
```

`toBuild.sh` 为原有构建脚本，会删除当前目录的 build、log、install 后重新构建。
必须在本机重新构建；运行脚本会自动加载当前目录的 `install/setup.bash`。
原始代码里的导航参数包含历史路径，主启动链中的 robot_*_slam.launch.py
会通过 RewrittenYaml 注入当前工作空间的行为树路径。

## 启动

```bash
bash start_all_mappo.sh --map default
bash start_all_mappo.sh --map maze40
bash start_all_mappo.sh --map real
```

默认模型及配置：

```text
models/inference/mappo/mappo_v4_mask_energy_safe_30k/
├── config.json
└── models/best_model.pt
```

配置必须与模型一起保留。配置中的历史训练数据绝对路径是训练元数据，
本工作空间不包含这些离线训练数据，也不执行重新训练。

## 补充充电实验

先在 30×30 场景试跑以下较低容量，检查是否形成多轮补能与并发请求：

```bash
bash start_all_mappo.sh --map default \
  robot0_battery_capacity_ah:=0.30 \
  robot1_battery_capacity_ah:=0.32 \
  robot2_battery_capacity_ah:=0.31 \
  discharge_during_return:=false
```

以上容量是试跑起点，不保证产生指定充电次数；应根据运行结果继续调整。
参数设置的是电池容量（Ah），不是初始 SOC。原 30×30 PACE 运行记录的容量为
`0.40 / 0.43 / 0.41 Ah`，这些运行未发生充电。

默认 `discharge_during_return:=false`：返航开始后暂停放电；
如需计入实际返航耗电，明确传入 `discharge_during_return:=true`。
正式对比必须统一这一设置。原有补能模型为授权后立即重置电量，
再占用一个共享逻辑资源 10 秒；三个接入点分别位于机器人自己的初始位置。

## 结果与配置记录

默认输出目录：

```text
output/energy_mission_runs/map_<场景>/mappo_fuzzy/<时间戳>/
```

启动脚本新增记录 `run_config.json`（命令、容量等参数、Git 提交号与模型哈希）、
`model_config.json` 和 `launch.log`，保留现有的 `summary.json`、
`energy_mission_metrics.csv`、电量/充电状态记录以及 `rl_infer/` 输出。
输出目录默认不进入 Git，应单独备份实验数据。

可用环境变量改变结果根目录：

```bash
RESULT_ROOT=/absolute/path/to/results bash start_all_mappo.sh --map default \
  robot0_battery_capacity_ah:=0.30 \
  robot1_battery_capacity_ah:=0.32 \
  robot2_battery_capacity_ah:=0.31
```

检查 `charging_metrics` 中的充电次数、队列和竞争记录，并结合状态时间线判断
是否覆盖返航、等待、充电和重新探索。容量选定后，使用相同配置重复正式实验。

## 验证范围

上传整理阶段进行了 Python 语法、Shell 语法、启动文件引用和模型完整性核对。
整理环境没有 ROS 2 / Gazebo，因此尚未进行本版本的 colcon 构建和仿真运行。
原运行源码与检查点的来源校验见 `SOURCE_MANIFEST.json`；
其中标明了为精简入口及记录配置而修改的源文件。
第三方组件保留各自的许可证和说明。
