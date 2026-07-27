# 当前论文主线清理清单

## 保留

### Bag

- `bags/corridor_01.bag`
- `bags/corridor_new.bag`
- `bags/corridor_new1bag.bag`
- `bags/corridor_new1bag.orig.bag`
- `bags/corridor_repeat_01.bag`
- `bags/corridor_repeat_02.bag`
- `bags/corridor_repeat_03.bag`

用户明确要求所有 bag 均保留，不纳入清理。

### 实验目录

- `paper_exp/formal_corridor_20260724/`
- `paper_exp/innovation1_directional_adaptive_fusion/`
- `paper_exp/innovation2_slip_adaptive_backend/`

### my_navigation 正式 Lua

- `cartographer_scout_2d.lua`
- `clean_baseline_no_innovation.lua`
- `innovation1_baseline_a_static_low.lua`
- `innovation1_baseline_b_static_high.lua`
- `innovation1_proposed_dynamic_anisotropic.lua`
- `innovation2_baseline_static_high.lua`
- `innovation2_baseline_static_low.lua`
- `innovation2_proposed_slip_adaptive.lua`

## 删除候选

### 旧 corridor_new 地图和诊断结果

- 顶层 `maps/` 整个目录。当前内容均为 `corridor_new*`、旧 `corridor_01_odom_map` 和前端诊断结果，正式结果已转入 `paper_exp/formal_corridor_20260724/`。

### my_navigation 临时 Lua

- `diagnostic_frontend_no_loop_low_odom.lua`
- `diagnostic_frontend_no_loop_no_odom.lua`
- `diagnostic_low_odometry.lua`
- `diagnostic_no_free_space.lua`
- `diagnostic_no_odometry.lua`
- `robust_corridor_baseline.lua`
- `robust_corridor_medium_loop.lua`
- `robust_corridor_wide_relocalization.lua`
- `robust_corridor_wide_strong.lua`
- `scan_r045_h0025.lua`
- `scan_r045_h0030.lua`
- `scan_r050_h0025.lua`
- `scan_r050_h0030.lua`
- `scan_r055_h0025.lua`
- `scan_r055_h0030.lua`

其中 `scan_r*` 是参数扫描脚本自动生成的配置；`run_parameter_scan.sh` 需要重跑时会重新生成，不需要常驻 `my_navigation/config`。

### 临时文件

- 根目录 `cartographer_print_configuration*` 日志和链接。
- `paper_exp/**/__pycache__/` Python 字节码缓存。

## 明确不删除

- `bags/` 下的全部 bag。
- 源码、创新点文档、论文初稿、每日总结。
- 当前正式实验的 PBStream、地图、指标、日志和复现实验脚本。
- 工作区内已有但与本清单无关的用户修改。
