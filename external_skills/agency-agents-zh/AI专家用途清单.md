# agency-agents-zh AI 专家用途清单

这份文档是 `jnMetaCode/agency-agents-zh` 的中文速查版，用来快速判断“该找哪一类 AI 专家做什么事”。

仓库里一共有 266 个专家，完整逐项索引以仓库自带的 `AGENT-LIST.md` 为准；这份文档先给你一个能直接用的概览。

## 总体理解

- 每个 AI 专家都对应一个明确岗位
- 同一部门里的专家共享一个业务领域，但各自任务不同
- 你先按部门找，再按专家名细分
- 对复杂项目，通常是“架构师 + 执行专家 + 审查专家”组合使用

## 部门用途总览

| 部门 | 主要用途 |
|---|---|
| 工程部 | 写代码、架构设计、调试、审查、部署、自动化、嵌入式、前后端开发 |
| 设计部 | UI、UX、品牌、视觉、提示词、信息呈现 |
| 营销部 | 内容增长、SEO、社媒、电商、投放、AEO、传播 |
| 付费媒体部 | 广告投放、归因、关键词、创意测试、预算优化 |
| 销售部 | 线索获取、销售跟进、赢单、提案、售前 |
| 金融部 | 财务核算、预测、税务、风控、投资分析 |
| 人力资源部 | 招聘、绩效、组织管理 |
| 法务部 | 合同审查、制度文件、合规风险 |
| 供应链部 | 采购、库存、物流、供应商、工厂规划 |
| 产品部 | 需求、优先级、反馈、趋势、行为设计 |
| 项目管理部 | 会议纪要、实验追踪、项目推进、工作流管理 |
| 测试部 | 测试设计、证据收集、性能、可用性、API 测试 |
| 支持部 | 客服、财务支持、法务支持、基础设施支持 |
| 专项部 | 会议、数据、培训、身份、政策、治理、文档等专门任务 |
| 空间计算部 | XR、Vision Pro、沉浸式交互、终端集成 |
| 游戏开发部 | Unity、Unreal、Godot、Roblox、Blender、关卡、叙事、技术美术 |
| 学术部 | 研究、叙事、心理、历史、人类学、地理、学习规划 |
| GIS 部 | 地图、空间分析、Web GIS、制图、三维场景、空间数据工程 |
| 安全部 | 威胁建模、代码审计、云安全、合规、渗透、事件响应 |

## 你这个机器人项目最相关的角色

这些角色最接近你现在的 ROS、自主搜索、地图、定位和驱动任务：

- `engineering-software-architect`：做整体系统架构拆分
- `engineering-embedded-linux-driver-engineer`：处理驱动、设备树、底层通信
- `engineering-pc-host-engineer`：上位机、实时显示、串口/CAN/工业协议
- `engineering-embedded-firmware-engineer`：下位机、传感器、控制固件
- `engineering-iot-solution-architect`：做设备接入、边缘通信、系统联调
- `engineering-codebase-onboarding-engineer`：快速读懂现有代码库
- `engineering-minimal-change-engineer`：尽量少改动地修问题
- `engineering-code-reviewer`：检查风险和回归
- `engineering-sre`：部署、可观测性、稳定性
- `gis-spatial-data-engineer`：地图数据清洗、坐标、空间数据处理

## 高价值角色速查

### 工程

- `engineering-frontend-developer`：做前端界面
- `engineering-backend-architect`：做服务端和接口
- `engineering-ai-engineer`：做 AI/模型集成
- `engineering-data-engineer`：做数据管线
- `engineering-security-engineer`：做安全审查
- `engineering-rapid-prototyper`：快速做原型
- `engineering-technical-writer`：写技术文档

### 设计

- `design-ui-designer`：界面视觉
- `design-ux-architect`：交互和信息架构
- `design-brand-guardian`：品牌一致性
- `design-image-prompt-engineer`：图像提示词
- `design-visual-storyteller`：可视化叙事

### 营销

- `marketing-seo-specialist`：搜索优化
- `marketing-social-media-strategist`：社媒策略
- `marketing-content-creator`：内容创作
- `marketing-growth-hacker`：增长实验
- `marketing-xiaohongshu-operator`：小红书运营
- `marketing-douyin-strategist`：抖音增长

### 产品

- `product-manager`：产品决策
- `product-sprint-prioritizer`：排需求优先级
- `product-feedback-synthesizer`：整理反馈
- `product-trend-researcher`：找趋势
- `product-behavioral-nudge-engine`：行为引导设计

### 测试

- `testing-api-tester`：接口测试
- `testing-performance-benchmarker`：性能基准
- `testing-accessibility-auditor`：无障碍检查
- `testing-evidence-collector`：收集测试证据
- `testing-test-results-analyzer`：分析测试结果

### 安全

- `security-architect`：安全架构
- `security-appsec-engineer`：应用安全
- `security-cloud-security-architect`：云安全
- `security-incident-responder`：事件响应
- `security-threat-intelligence-analyst`：威胁情报

## 选择方法

1. 先确定任务属于哪个部门
2. 再选一个“负责人”角色
3. 再补一个“执行”角色
4. 最后让审查/测试角色做校验

## 完整清单

- 仓库完整索引：`AGENT-LIST.md`
- 项目说明：`README.md`
- 部门目录：按文件夹名直接对应，例如 `engineering/`、`design/`、`marketing/`

