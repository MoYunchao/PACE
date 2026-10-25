# 剩余消融批量入口

两个新程序复用原四次批量程序的 summary 检测、进程清理和失败处理。
不修改 PACE、模型、评分逻辑或现有启动脚本。

| 程序 | 地图/模式 | 三台电池容量 | 次数 |
|---|---|---|---|
| `run_risk_maze40_4runs.py` | maze40 / risk | 各 0.40 Ah | 4 |
| `run_default_ablation_4runs.py` | default / semantic、distance、gain、risk | 各 0.30 Ah | 每种 4 次，共 16 次 |

都使用 v5 的 `best_model.pt`，返航暂停放电。各次实验结束并清理进程后等待 30 秒，
模式切换时也等待 30 秒；最后一次结束后直接退出。
开始批次前结束现有仿真，在可使用 ROS 和 PyTorch 的环境中执行。
两个批次单独运行，不要同时启动。

```bash
cd ~/PACE
python3 -u run_risk_maze40_4runs.py
```

maze40 完成后启动默认地图的全部四种消融：

```bash
python3 -u run_default_ablation_4runs.py
```

也可以单独运行默认地图的一种模式，例如：

```bash
python3 -u run_default_ablation_4runs.py --mode gain
```

两个程序都支持 `--dry-run` 查看命令而不启动 ROS，以及 `--timeout-sec 2400`
调整单轮等待汇总的超时。默认超时 1800 秒；启动错误、超时或清理失败都会停止后续运行。
Ctrl+C 清理当前实验并停止批次。已有任务失败汇总会保留，不自动重跑或删除。

结果分别保存在：

```text
output/risk_maze40_batches/<时间戳>/risk/run_01/ ... run_04/
output/default_ablation_batches/<时间戳>/semantic/run_01/ ... run_04/
output/default_ablation_batches/<时间戳>/distance/run_01/ ... run_04/
output/default_ablation_batches/<时间戳>/gain/run_01/ ... run_04/
output/default_ablation_batches/<时间戳>/risk/run_01/ ... run_04/
```

每个 run 下都有 `console.log` 和 `results/` 内的原始实验输出。
批次根目录 `batch_summary.json` 记录模式、命令、各次状态和原始汇总路径。
这些批次程序不会自动跳过以前的结果；再次启动会创建新批次并执行完整计划。
