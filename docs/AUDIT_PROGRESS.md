# VisualSim 文档审计进度（AUDIT_PROGRESS）

> 当前轮次：Round 1；状态：审计待启动，意见已登记，**尚未核实或整改**。
>
> 审计依据：[AUDIT.md](AUDIT.md)；文档对象：SRS.md、HLD.md、LLD.md、IMPLEMENTATION_PLAN.md；仅允许修改 ncut-d1z/VisualSim 的 docs 文档。来源 ncut-d1z/ros2-vio-node 禁止写入。
>
> 登记基线：目标 main **f5745154fd36cc77685eafaea7b512f864050fc4**。本轮报告以实际复核记录为准；不得将用户审核中的判断直接填成“已整改”。

## 1. 逐文件审计状态

审计状态采用：**空白 / 正在审计 / 已审计**。整改状态采用：**未整改 / 已整改 / 无需整改**。

| 文件相对路径 | 审计状态 | 审计目标 | 发现问题 |
| --- | --- | --- | --- |
| docs/SRS.md | 空白 | v1 状态、§6 加载状态、§10 追踪概览 | A01、A02、A03 |
| docs/HLD.md | 空白 | §3 配置/Generate 数据流，§9.2 依赖视图 | A04、A05、A06 |
| docs/LLD.md | 空白 | §2.4、§5.1、§6.2、§8、§9.1 | A03、A04、A07、A08、A09、A10 |
| docs/IMPLEMENTATION_PLAN.md | 空白 | §6、§8、§9 引用、映射、Markdown、shell 失败分支 | A02、A11、A12、A13 |

## 2. 整改目标逐条登记（审计前，不预判）

| ID | 严重性 | 文件相对路径 | 具体整改目标 | 整改状态 | 核查/整改证据 |
| --- | --- | --- | --- | --- | --- |
| A01 | 建议修 | docs/SRS.md | 头部状态/版本标签“v1 需求基线（冻结，待实施验证）” | 未整改 | 待审计 |
| A02 | 必须修 | docs/SRS.md；docs/IMPLEMENTATION_PLAN.md | 比对 SRS §10 与 PLAN §8 完整追踪矩阵，处理 FR-M01/M02、FR-M04、FR-S01、NFR-07 及其他差异 | 未整改 | 待审计 |
| A03 | 必须修 | docs/SRS.md；docs/LLD.md | SRS §6 补 LoadMotionConfig 新实例状态限制；LLD §9.1 明确 EventHistoryFrozen 等一致失败码与原子失败 | 未整改 | 待审计 |
| A04 | 必须修 | docs/HLD.md；docs/LLD.md | HLD Generate 序列先 Eval 当时传感器集合，后 Seq 检查时间和编号；与 LLD §8 对齐 | 未整改 | 待审计 |
| A05 | 建议修 | docs/HLD.md | HLD 配置加载图/文字明确只能装载到新实例，增量使用即时式 API | 未整改 | 待审计 |
| A06 | 建议修 | docs/HLD.md | HLD 部署视图分清 Sophus 与 yaml-cpp，明确配置解析/CLI 与核心链接边界 | 未整改 | 待审计 |
| A07 | 必须修 | docs/LLD.md | last_successful_ns 未生成状态、首次合法时间、Reset 后状态明确 | 未整改 | 待审计 |
| A08 | 建议修 | docs/LLD.md | next_auto_camera_index 初始值、默认 cam0 实化、首次与后续自动分配和更新规则明确 | 未整改 | 待审计 |
| A09 | 建议修 | docs/LLD.md | Brownian-bridge L_s 明确“当前父区间长度” | 未整改 | 待审计 |
| A10 | 建议修 | docs/LLD.md | 圆周相容性表 r0 就地解释或引用 LLD §4.2 | 未整改 | 待审计 |
| A11 | 必须修 | docs/IMPLEMENTATION_PLAN.md | 清理 §9 Windows 负例后多余 Markdown ~~~ | 未整改 | 待审计 |
| A12 | 建议修 | docs/IMPLEMENTATION_PLAN.md | V02 “§5.1”标注为“LLD §5.1” | 未整改 | 待审计 |
| A13 | 建议修 | docs/IMPLEMENTATION_PLAN.md | LIB 未找到时 shell 模板明确错误和 exit 1，不依赖 set -e | 未整改 | 待审计 |

## 3. 本轮审核意见基线摘要

用户判断：四份文档已经具备 SRS/HLD/LLD/PLAN 各自结构并达到 v1 设计基线冻结候选水平；圆周参数、YAML baseline→events 同刻顺序、容差、Raycast 失败码与统计验收参数已有冻结设计；本轮重点是 5 个必须修问题和其余建议修事项，**不应重开架构设计**。

按文件归类，SRS 重点是状态/追踪/加载实例限制；HLD 是 Generate 顺序、LoadMotionConfig 图示和 yaml-cpp/Sophus 部署；LLD 是时间初始态、相机自动编号、LoadMotionConfig 状态码、Brownian-bridge 术语、r0 说明；PLAN 是 Markdown 代码块、追踪矩阵、V02 引用和命令模板失败路径。

## 4. 过程记录

| 轮次 | 阶段 | 结果 |
| --- | --- | --- |
| Round 1 | 登记 | 已收录用户所有逐文件修改意见，未开始逐项核查，所有条目初始“未整改” |

## 5. 汇总

- 审计对象：4；已审计：0；正在审计：0；空白：4。
- 具体整改目标：13；已整改：0；无需整改：0；未整改/待核查：13。
- 本轮不实施生产代码、P0～P6 编码任务或构建/CTest。
