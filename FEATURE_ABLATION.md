# 完整 PACE 上的四维评分信息消融

这是独立新增的 `pace_ablation` 包，不替换或修改原 PACE 节点、CMakeLists、
`start_all_mappo.sh`、原有 launch 文件或训练模型。

## 实验问题

在同一个预训练 PACE 模型下，前沿评分失去某维实际信息后，任务表现如何变化？
比较基准 `full` 就是完整 PACE。五组加载同一检查点，保留动态权重，不重新训练。
原有充电 PACE 运行只有在模型哈希、地图、容量与其余运行设置一致时，才可作为完整组复用。

| 模式 | 处理 |
|---|---|
| full | 四维真实输入、MAPPO 动态权重，即完整 PACE |
| semantic | 不使用该候选实际语义值 |
| distance | 不使用该机器人到候选的实际前沿评价距离 |
| gain | 不使用该候选实际信息增益 |
| risk | 不使用该候选实际风险值 |

对被移除维度，在归一化空间统一取 `0, 0.25, 0.5, 0.75, 1`，
分别运行原有隶属函数、规则、局部地图调整及解模糊，再取五个**最终评分的算术平均**。
其余三维使用实际值，四个权重始终使用该机器人当前的 MAPPO 输出；不置零、不重新分配权重。

这是离散参考分布下的评分平均，不是将所有候选判成零风险/零距离，
也不是把某个动态权重固定为 0.25。所有候选共用参考分布，避免选择一个固定特征值
持续激活某类规则。分布为预设均匀离散分布，未根据实验结果调整。
它仍是一种评分层的信息消融，不代表重新设计/训练后的三维系统最优性能。
actor 的观测和网络不做屏蔽，因此被消融维度的相关信息仍可能通过策略观测影响动态权重。
实验应称为「固定预训练模型下的逐维评分信息消融」，不能声称整个 PACE 完全移除了该类信息。

导航、碰撞规避、可达性筛选、资源状态及返航安全层继续使用实际数据。
尤其 `risk` 只消融模糊评分中的风险信息，并不移除安全机制；
其他保留维度可能与被移除信息相关，结果不应解释为整个系统完全失去该类信息。

## 构建与运行

已经构建过原工作空间时，只需构建新增包：

```bash
cd ~/PACE
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select pace_ablation
```

新克隆工作空间可沿用原 `bash toBuild.sh` 构建全部包。

所有组都必须通过 `--model` 显式指定同一模型，并提供三个电池容量（Ah），
避免不小心使用不同默认值。下面以 v5 模型、maze40、三台均为 0.4 Ah 为例：

```bash
bash start_feature_ablation.sh --map maze40 --mode full \
  --model models/inference/mappo/mappo_v5_battery040_090_seed42/models/best_model.pt \
  robot0_battery_capacity_ah:=0.40 \
  robot1_battery_capacity_ah:=0.40 \
  robot2_battery_capacity_ah:=0.40 \
  discharge_during_return:=false
```

依次将 `--mode full` 改为 `semantic`、`distance`、`gain`、`risk`。
场景参数仍为 `default`、`maze40`、`real`。一次运行一组，完全结束上一组后再启动下一组。
初始位置、导航、能源模型、充电协调、启动延迟复用原场景设置。
唯一额外计算是被消融组每个候选执行五次模糊评分，原完整对照只执行一次。
本方案不人为匹配计算开销；应在部署机器上确认评分不会造成明显决策延迟。

## 结果

```text
output/mappo_feature_ablation_runs/map_<场景>/mappo_full/<时间戳>/
output/mappo_feature_ablation_runs/map_<场景>/mappo_without_semantic/<时间戳>/
output/mappo_feature_ablation_runs/map_<场景>/mappo_without_distance/<时间戳>/
output/mappo_feature_ablation_runs/map_<场景>/mappo_without_gain/<时间戳>/
output/mappo_feature_ablation_runs/map_<场景>/mappo_without_risk/<时间戳>/
```

每次保存 `run_config.json`、`launch.log` 和原 supervisor 的汇总、CSV、充电与电量记录。
配置包含模式、动态权重标记、模型及配置哈希、参考值、电池容量、Git 提交号和源码哈希。
同时保存 `model_config.json` 和 `rl_infer/` 的观测、动态权重等推理记录。
旧版本固定权重消融的结果仍在 `output/feature_ablation_runs/`，不要与当前模型消融混合统计。

评分消息中的 `total_score` 是用于分配的平均最终评分；四个等级隶属度字段是
参考样本的平均隶属度，仅供诊断。**不要再次对平均隶属度解模糊来替换 total_score**，
两者通常不等价。

建议报告成功次数/总次数、T95、ATL、EPC、充电次数；未完成的运行单独报告。
沿用相同重复次数和配对实验条件。该 Gazebo 链路没有新增随机种子控制开关，
不要将运行编号称为已设置的仿真种子。

## 本地验证

```bash
python3 src/pace_ablation/tests/test_scoring.py
python3 src/pace_ablation/tests/test_launch.py
```

检查新节点中的原有评分函数保持一致；独立编译实际 C++ 评分方法，验证完整对照与
原动态权重评分等价、给定相同策略输出时被消融实际值不影响评分、其余特征仍能改变评分，以及最终评分确为
五个参考评分的平均。该检查不替代 ROS 构建和 Gazebo 联调。
