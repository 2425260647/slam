# 论文 Baseline：Cartographer 2D 当前源码快照

## 标识

- Git tag：`paper-baseline-cartographer-20260907`
- Baseline 类型：论文实验的 Cartographer 2D 源码基线
- 记录日期：2026-09-07
- 基线提交：以该 annotated tag 指向的提交为准

## 源码范围

本基线包含当前仓库中的：

- `src/cartographer`
- `src/cartographer_ros`
- 当前论文实验使用的 Cartographer Lua 配置和 Scout Mini/Gazebo 启动入口

在建立该基线时，`src/cartographer` 和 `src/cartographer_ros` 工作区均无未提交差异。
本次没有把已有的 `lidar_adaptive` 实验改动、未跟踪实验数据、导航包改动或其他工作区文件
混入基线提交。

## Cartographer 源码指纹

以下指纹由 baseline 建立前的 Git tree 计算得到，用于确认后续核心算法修改的起点：

```text
src/cartographer + src/cartographer_ros tree sha256:
c82205cb68d978f5737bb7d4acdb356cee3416556d619232e48cfc3ebb2e97a9
```

后续每一个论文模块都必须记录：

1. 修改前 baseline tag；
2. 修改文件清单；
3. 修改后提交或源码指纹；
4. 回滚方法；
5. 与 baseline 的独立消融结果。

## 当前基线行为边界

- 当前 Cartographer 2D 使用已有的局部扫描匹配、MotionFilter、active submap、Pose Graph
  和回环流程。
- 当前前端已存在扫描匹配点云与地图插入点云分离，以及按角度保留最近回波的地图插入过滤。
- 当前论文基线不包含新的“每帧匹配、选中帧地图插入”桥接，不包含自适应地图增量准入。
- `lidar_adaptive` 中已有 C1 置信投影和实验记录作为既有研究工作保留，但不改变本基线中
  Cartographer 源码的定义。

## 基线复核命令

```bash
git show --stat --oneline paper-baseline-cartographer-20260907
git diff paper-baseline-cartographer-20260907 -- src/cartographer src/cartographer_ros
git ls-tree -r --full-tree paper-baseline-cartographer-20260907 -- \
  src/cartographer src/cartographer_ros | sha256sum
```

后续修改 Cartographer 核心前，必须先保留 baseline 结果，再建立独立实验分支或提交；
不能通过修改当前工作区的配置结果反推 baseline 性能。

## 论文实验基线口径

- Gazebo 使用连续 `/gazebo/model_states` 真值计算 ATE/RPE。
- 真实 bag 没有连续外部真值时，只报告闭合误差、墙体结构、Chamfer、运行时间和存储代理。
- 关键帧、submap、后端节点、pbstream 字节数和 CPU 时间必须与同一输入、同一回放速度和
  同一配置比较。
- PGM 宽高不单独等同于地图存储大小；它同时受覆盖范围和分辨率影响。
