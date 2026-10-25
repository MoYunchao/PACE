# maze40 语义消融自动运行 4 次

运行程序前结束现有的手动仿真实验，使用能够同时导入 ROS 与 PyTorch 的环境。
该程序只调用现有命令，不更改模型、容量、启动脚本或评分方式：

```bash
bash start_feature_ablation.sh --map maze40 --mode semantic \
  --model models/inference/mappo/mappo_v5_battery040_090_seed42/models/best_model.pt \
  robot0_battery_capacity_ah:=0.40 \
  robot1_battery_capacity_ah:=0.40 \
  robot2_battery_capacity_ah:=0.40 \
  discharge_during_return:=false
```

启动：

```bash
cd ~/PACE
python3 -u run_semantic_maze40_4runs.py
```

程序按以下顺序执行，不并行运行：

1. 启动第 1 次实验，为它创建独立的结果目录。
2. 等待有效的任务 `summary.json`，不以终端是否停止输出为完成依据。
3. 留出 3 秒写入末尾记录，再向本轮进程发送 SIGINT；未退出时逐级使用 SIGTERM/SIGKILL。
4. 确认本轮进程退出后等待 30 秒，启动下一次，共 4 次。
5. 第 4 次结束后直接退出，不再等待 30 秒。

程序只跟踪和关闭自己启动的进程及其子进程，不使用全局 `pkill`。
Ctrl+C 会清理当前实验并结束批次，不会启动下一轮。
每次运行最多等待 1800 秒得到汇总；超时、启动错误或清理失败时停止整批，保留已生成文件。
有效的未达标任务汇总也会保留，并注明 `target_reached: false`；不会自动重跑或丢弃。

批次目录：

```text
~/PACE/output/semantic_maze40_batches/<批次时间戳>/
├── batch_summary.json
├── run_01/
│   ├── console.log
│   └── results/map_maze40/mappo_without_semantic/<单轮时间戳>/...
├── run_02/...
├── run_03/...
└── run_04/...
```

单轮目录里仍保存原有 summary、CSV、run_config、launch.log 和 rl_infer 记录。
控制台显示每轮开始/结束及日志路径；详细实时输出在 `console.log` 中。
整批传输时，打包这个批次目录即可。

仅检查计划、不启动实验：

```bash
python3 run_semantic_maze40_4runs.py --dry-run
```

若需要改变单轮超时上限，例如 2400 秒：

```bash
python3 -u run_semantic_maze40_4runs.py --timeout-sec 2400
```

模拟任务完成、超时清理和 4 次/30 秒调度的验证：

```bash
python3 tools/test_semantic_batch.py
```

这些检查不启动 ROS/Gazebo，不替代远程仿真验证。
