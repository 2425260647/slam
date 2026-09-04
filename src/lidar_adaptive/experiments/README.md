# 实验目录模板

复制本目录为 `experiments/<date>_<dataset>_<case>/`，再写入本次运行材料：

```text
README.md                 # 本次假设、配置、结果和失败原因
config_snapshot/          # YAML、Cartographer Lua、launch 快照
input_manifest/           # bag 文件名、时长、话题、SHA256
logs/                     # roslaunch、节点和系统日志
topics/                   # 质量、信息量、选择标志、TF 导出
maps/                     # PGM/YAML 或 pbstream
metrics/                  # JSON/CSV，含均值、标准差、失败数
figures/                  # 论文图及生成命令
```

结果必须保留成功和失败运行。没有连续外部真值的真实数据不得在 `metrics/` 中填写 ATE/RPE。
