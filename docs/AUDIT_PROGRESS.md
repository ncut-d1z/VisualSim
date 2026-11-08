# VisualSim 文档审计进度（AUDIT_PROGRESS）

> 当前轮次：Round 1；状态：**四份文档已审计，13 项整改目标均已更正并静态复核**。代码尚未实施或验收。
>
> 审计依据：[AUDIT.md](AUDIT.md)；文档对象：SRS.md、HLD.md、LLD.md、IMPLEMENTATION_PLAN.md；仅允许修改 ncut-d1z/VisualSim 的 docs 文档。来源 ncut-d1z/ros2-vio-node 禁止写入。
>
> 初始登记基线：目标 main **f5745154fd36cc77685eafaea7b512f864050fc4**；审计文件先行提交 **4eb7110b36c26679a26c48499f4656d97723f9f0**。本轮已对整改后的文档对象执行逐项静态复核，整改结论不等于运行程序验证。

## 1. 逐文件审计状态

审计状态采用：**空白 / 正在审计 / 已审计**。整改状态采用：**未整改 / 已整改 / 无需整改**。

| 文件相对路径 | 审计状态 | 审计目标 | 发现问题 |
| --- | --- | --- | --- |
| docs/SRS.md | 已审计 | v1 状态、§6 加载状态、§10 追踪概览 | A01、A02、A03 |
| docs/HLD.md | 已审计 | §3 配置/Generate 数据流，§9.2 依赖视图 | A04、A05、A06 |
| docs/LLD.md | 已审计 | §2.4、§5.1、§6.2、§8、§9.1 | A03、A04、A07、A08、A09、A10 |
| docs/IMPLEMENTATION_PLAN.md | 已审计 | §6、§8、§9 引用、映射、Markdown、shell 失败分支 | A02、A11、A12、A13 |

## 2. 整改目标逐条登记（审计前，不预判）

| ID | 严重性 | 文件相对路径 | 具体整改目标 | 整改状态 | 核查/整改证据 |
| --- | --- | --- | --- | --- | --- |
| A01 | 建议修 | docs/SRS.md | 头部状态/版本标签“v1 需求基线（冻结，待实施验证）” | 已整改 | SRS 头部现已标识 v1 需求基线（冻结，待实施验证）；旧稿缺标签 |
| A02 | 必须修 | docs/SRS.md；docs/IMPLEMENTATION_PLAN.md | 比对 SRS §10 与 PLAN §8 完整追踪矩阵，处理 FR-M01/M02、FR-M04、FR-S01、NFR-07 及其他差异 | 已整改 | SRS §10 映射逐项按 PLAN §8 同步；程序化复核 18 个 FR/NFR AC 集合全部一致 |
| A03 | 必须修 | docs/SRS.md；docs/LLD.md | SRS §6 补 LoadMotionConfig 新实例状态限制；LLD §9.1 明确 EventHistoryFrozen 等一致失败码与原子失败 | 已整改 | SRS §6 加载实例限制；LLD §9.1 明确先检查实例并返回 Status::EventHistoryFrozen，原子不提交 |
| A04 | 必须修 | docs/HLD.md；docs/LLD.md | HLD Generate 序列先 Eval 当时传感器集合，后 Seq 检查时间和编号；与 LLD §8 对齐 | 已整改 | HLD §3.1 已先 Eval 得到 at_ns 相机/IMU 集合，再 Seq 校验；与 LLD §8 顺序一致 |
| A05 | 建议修 | docs/HLD.md | HLD 配置加载图/文字明确只能装载到新实例，增量使用即时式 API | 已整改 | HLD §3 数据流图及文字都限定新实例 LoadMotionConfig，增量通过事件 API |
| A06 | 建议修 | docs/HLD.md | HLD 部署视图分清 Sophus 与 yaml-cpp，明确配置解析/CLI 与核心链接边界 | 已整改 | HLD §9.2 将 Sophus 几何依赖与 yaml-cpp 配置解析依赖分开，并注明实际链接边界 |
| A07 | 必须修 | docs/LLD.md | last_successful_ns 未生成状态、首次合法时间、Reset 后状态明确 | 已整改 | LLD §8 明确 optional<TimestampNs>=nullopt，首次含 0ns 合法；Reset 回未生成状态；伪代码跳过空比较 |
| A08 | 建议修 | docs/LLD.md | next_auto_camera_index 初始值、默认 cam0 实化、首次与后续自动分配和更新规则明确 | 已整改 | LLD §5.1 明确 next_auto_camera_index=0、虚拟 cam0、替代→1、保留→扫描 cam1+、跳号和失败不提交 |
| A09 | 建议修 | docs/LLD.md | Brownian-bridge L_s 明确“当前父区间长度” | 已整改 | LLD §6.2 已明示 L_s 为当前父区间长度，总增量为父区间 Δ |
| A10 | 建议修 | docs/LLD.md | 圆周相容性表 r0 就地解释或引用 LLD §4.2 | 已整改 | LLD §2.4 圆周相容性表解释 r0 为段首有向半径并指向 LLD §4.2 |
| A11 | 必须修 | docs/IMPLEMENTATION_PLAN.md | 清理 §9 Windows 负例后多余 Markdown ~~~ | 已整改 | PLAN §9 删除孤立 ~~~，四份文档分隔符均通过成对检查 |
| A12 | 建议修 | docs/IMPLEMENTATION_PLAN.md | V02 “§5.1”标注为“LLD §5.1” | 已整改 | PLAN §6 V02 引用已改为 LLD §5.1 |
| A13 | 建议修 | docs/IMPLEMENTATION_PLAN.md | LIB 未找到时 shell 模板明确错误和 exit 1，不依赖 set -e | 已整改 | PLAN §9 检测库文件未找到时打印错误并 exit 1，不依赖外部 set -e |

## 3. 本轮审核意见基线摘要

用户判断：四份文档已经具备 SRS/HLD/LLD/PLAN 各自结构并达到 v1 设计基线冻结候选水平；圆周参数、YAML baseline→events 同刻顺序、容差、Raycast 失败码与统计验收参数已有冻结设计；本轮重点是 5 个必须修问题和其余建议修事项，**不应重开架构设计**。

按文件归类，SRS 重点是状态/追踪/加载实例限制；HLD 是 Generate 顺序、LoadMotionConfig 图示和 yaml-cpp/Sophus 部署；LLD 是时间初始态、相机自动编号、LoadMotionConfig 状态码、Brownian-bridge 术语、r0 说明；PLAN 是 Markdown 代码块、追踪矩阵、V02 引用和命令模板失败路径。

## 4. 过程记录

| 轮次 | 阶段 | 结果 |
| --- | --- | --- |
| Round 1 | 先行登记 | 初始提交 4eb7110b36c26679a26c48499f4656d97723f9f0；全数录入 13 个问题，四份文件状态“空白” |
| Round 1 | 逐文档审计 | 对照 f5745154fd36cc77685eafaea7b512f864050fc4 的 SRS/HLD/LLD/PLAN 实际章节核实全部 13 项；未发现需要重新设计的阻断项 |
| Round 1 | 文档整改 | SRS 修状态/加载/映射；HLD 修序列/配置/依赖；LLD 修初始时间/自动索引/失败码/公式术语；PLAN 修引用/代码块/命令 |
| Round 1 | 静态复核 | 18 个 FR/NFR 追踪映射全部一致；HLD 求值先于 Seq；四份文档 Markdown ~~~ 标记均成对；LLD 关键状态文字复核通过 |

## 5. 汇总

- 审计对象：**4**；已审计：**4**；正在审计：**0**；空白：**0**。
- 具体整改目标：**13**；已整改：**13**；无需整改：**0**；未整改：**0**。
- 严重性：**必须修 5/5 已闭环**；其他建议修 **8/8 已闭环**。
- 新增问题：本轮审计范围内**未新增阻断项**；若后续发现，遵守先登记后整改。
- 本轮仅文档整改；**生产代码、P0～P6 开发、编译、CTest 与功能验收均未执行**。
