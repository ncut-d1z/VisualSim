# VisualSim 文档审计进度（AUDIT_PROGRESS）

> 当前轮次：**Round 5 已完成：A31–A34 共 4/4 项已整改**，累计 A01–A34 **34/34 闭环**。实施计划现含 46 组真实旧片段与拟议新代码对照，另有 8 组新内核骨架；**没有修改生产代码、没有执行 CMake/CTest 或功能验收**。
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


## 6. Round 2：审计前问题登记（2026-11-08）

> **登记顺序**：本节与 AUDIT.md §8 先行写入并提交，随后才执行实际审计。登记状态不是问题已证实；“未整改”表示待核验或待文字修订。本轮以用户给定“v1 高符合度，可启动 P0”为待验证基线，不直接假定完成。

### 6.1 Round 2 逐文件最终审计状态

| 文件相对路径 | Round 2 审计状态（空白 / 正在审计 / 已审计） | 重点 | 相关事项 |
| --- | --- | --- | --- |
| docs/SRS.md | 已审计 | §6 Reset、FR/NFR/AC、追踪概览 | A14；§8.3 跨文档检查 |
| docs/HLD.md | 已审计 | §9.2 部署 Loader 链接语义、时序数据流 | A15；§8.3 跨文档检查 |
| docs/LLD.md | 已审计 | §2.1 提交点、§8 Reset/虚拟 cam0、v1 冻结合同 | A16、A17；§8.3 跨文档检查 |
| docs/IMPLEMENTATION_PLAN.md | 已审计 | §2 轮次措辞、§9 V01 覆盖、追踪矩阵 | A18、A19；§8.3 跨文档检查 |

### 6.2 本轮新增整改目标（先登记，后逐项核验与整改）

| ID | 相对路径 | 整改目标 | 整改状态 | 证据/备注 |
| --- | --- | --- | --- | --- |
| A14 | docs/SRS.md | §6 ResetSamplingSession 明确包含已使用的虚拟默认 cam0 最小协议控制状态 | 已整改 | SRS §6 已补已使用虚拟 cam0 (SensorId=0) 的 Reset 协议状态范围；整改提交 71cb320b |
| A15 | docs/HLD.md | §9.2 部署图明确 Loader 不是必须独立部署的库，划清 yaml-cpp/Shared 链接关系 | 已整改 | HLD §9.2 部署图不再使用 Loader→Shared 独立节点，并说明 LoadMotionConfig 不是独立运行库；71cb320b |
| A16 | docs/LLD.md | §8 ResetSamplingSession 明确包括已使用的虚拟默认 cam0 | 已整改 | LLD §8 已显式包含曾使用/实化的虚拟默认 cam0，未使用时仍未生成；71cb320b |
| A17 | docs/LLD.md | §2.1 异常翻译与返回对象构造均先于事务提交点 | 已整改 | LLD §2.1 已注明异常翻译及正常/错误 Result 构造均须先于提交点；71cb320b |
| A18 | docs/IMPLEMENTATION_PLAN.md | §2 审计轮次表述避免过时的“第三轮审计” | 已整改 | PLAN §2 已将“本轮第三轮审计”改为“此前的文档审计”；71cb320b |
| A19 | docs/IMPLEMENTATION_PLAN.md | §9 检查单解释 V01 由构建、安装、外部消费者和 ROS 依赖扫描覆盖 | 已整改 | PLAN §9 已说明 V01 由构建、安装、消费者、ELF/ROS 扫描及 Windows 拒绝负例覆盖；71cb320b |

### 6.3 用户本轮评价已完成核验的规范性范围

- **SRS**：术语/假设、FR、§6 外部接口及 LoadMotionConfig 仅限新实例、NFR-01～07、AC-01～12、§10 追踪；圆周/空视线/延迟验收范围。
- **HLD**：概念模块、配置事件加载与 Eval→Seq Generate、只读 Raycast、安装部署中 Sophus/yaml-cpp 边界、NFR/风险。
- **LLD**：v1 接口/Result/异常/线程/ABI、固定时间与容差、圆周参数和 r0、相机自动编号、IMU Brownian-bridge 和统计、Room/Raycast/YAML、optional last_successful_ns。
- **PLAN**：P0 只复核冻结 API、P0～P6、V01～V13、§8 追踪、§9 可执行命令（libvisualsim.so 未找到应 exit 1）、性能延期与回滚。
- **跨文档**：AUDIT.md §8.3 的 9 项检查须逐项出具核查结论。

### 6.4 Round 2 过程记录

| 阶段 | 状态 | 证据/结果 |
| --- | --- | --- |
| 先行登记 | 已完成 | 审计前提交 d949f9f159aec7f94d92c5504f59f409bd8596c3，先登记 A14–A19 |
| 逐文件审计 | 已完成 | 4/4；SRS/HLD/LLD/PLAN 的冻结合同和用户给出的符合度清单逐项核对 |
| 编辑性整改 | 已完成 | A14–A19 六项小幅文字/部署图调整，提交 71cb320b75722b15338cbc54f6810b49ed822675 |
| 静态复核与状态汇总 | 已完成 | SRS §10/PLAN §8 的 FR/NFR→AC 共 18/18 同口径，0 差异；4 文档代码分隔符成对 |

### 6.5 Round 2 登记时的初始统计（历史快照）

- 本轮审计对象：4；已审计 0、正在审计 0、空白 4。
- 本轮新增整改建议：6；已整改 0、无需整改 0、未整改/待核验 6。
- 历史 Round 1：13/13 已整改，保留原有表格和过程记录。
- 文档评价仍待本轮复核；生产代码、P0～P6、编译、CTest 不在此轮执行范围。


### 6.6 Round 2 最终审计结论与证据

| 复核项 | 结论 | 文件依据 |
| --- | --- | --- |
| 圆周显式参数与重入 | 通过 | SRS FR-M04 的圆心/半径/法向/相位/恢复行为；HLD §4.2 委托 LLD 细则；LLD §4.2 CircularReactivationPolicy；PLAN P2/V04 |
| YAML baseline→events | 通过 | LLD §9.1 固定同刻顺序；SRS AC-02、HLD §3 配置→时间线、PLAN V02 均指向等价事件语义 |
| Raycast 空视线 | 通过 | SRS FR-G02/AC-08、LLD §7 的先整体验证后成功空结果、PLAN V08；HLD §7 不另定义矛盾语义 |
| GetChangeHistory | 通过 | SRS §6 全量/纳秒闭区间能力，HLD §3 历史查询概览，LLD §2/§3.3 签名/闭区间排序；PLAN §8 FR-M03/FR-C01 追踪 |
| 99% 卡方 | 通过 | SRS AC-06、LLD §6.2 8 个种子/ν=98303/双侧 99% 冻结表、PLAN V06-S（未执行） |
| 默认 cam0/自动编号 | 通过 | SRS FR-S01/AC-05、HLD §6.1 概览、LLD §5.1 next_auto_camera_index=0/单调分配、PLAN V05；新增 Reset 澄清一致 |
| v1 冻结状态、P0 | 通过 | 四份文档头部 v1 标签，PLAN §4 复核而不重复冻结 |
| SRS §10 与 PLAN §8 | 通过 | 程序化比对 18 个 FR/NFR→AC 关联集合全部一致，0 差异；PLAN 仍是唯一完整追踪矩阵 |
| 新的 Reset 包括虚拟 cam0 | 通过 | SRS §6、LLD §8 同时明确已使用的默认 cam0，均不清空模式/标定历史 |
| LoadMotionConfig 新实例限制 | 通过 | SRS §6、HLD §3、LLD §9.1 的 EventHistoryFrozen、原子失败规则一致 |
| Markdown 与命令可靠性 | 通过 | 四份设计文档 ~~~ 围栏均成对；PLAN §9 的 LIB 缺失显式 exit 1、V01 覆盖说明 |

- **本轮审计对象 4/4 已审计**；正在审计 0、空白 0。
- **本轮整改建议 6/6 已整改**；无需整改 0、未整改 0。
- **累计 A01–A19：19/19 已整改并有静态核查结论**；未整改 0。Round 1 的 13 条原始记录及初始状态快照保留，用于追溯“先登记后整改”的顺序。
- **设计状态**：v1 设计基线冻结，按文档检查已具备启动 **P0 源码差异审查**条件。
- **工程状态**：P0–P6 尚未实施；不声称 C++ 编译、CTest、数值统计验证或性能验收已经通过。AC-11/V12 性能仍延期。


## 7. Round 3：源码与实施计划一致性审计（2026-11-08）

> 审计基线：目标 `VisualSim/main@d4b38f2ea73fa5aef700b303517a45ec21281561`；来源**只读** `ros2-vio-node/ftr-eskf@fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03`。本节先登记后修改，不改动旧 19 项历史。

### 7.1 先行登记的审计状态

| 相对路径 | 审计状态 | 目标 |
| --- | --- | --- |
| docs/IMPLEMENTATION_PLAN.md | 已审计 | 分支基线、全文件清单、差异、迁移取舍、工具链 |
| docs/HLD.md | 已审计 | 工具链及源码依赖边界不与架构冲突 |
| docs/LLD.md | 已审计 | 工具链版本、算法边界不与源码迁移方案冲突 |
| docs/AUDIT.md | 已审计 | 本轮审计规则先行登记 |
| docs/AUDIT_PROGRESS.md | 已审计 | 问题先登记、过程及整改状态更新 |

### 7.2 先登记的整改目标

| ID | 文件相对路径 | 具体整改目标 | 整改状态 | 当前依据 |
| --- | --- | --- | --- | --- |
| A20 | docs/IMPLEMENTATION_PLAN.md | 采用 `ftr-eskf` 固定来源 SHA，不沿用来源 main | 已整改 | PLAN §1 与 §3 已切换为来源 ftr-eskf@fd1e0817；整改提交 26e42049102b |
| A21 | docs/IMPLEMENTATION_PLAN.md | 逐文件列出双方 VisualSim 本体、直接仓内依赖和 CameraIO 的头/源文件，说明未来是否写入目标仓库 | 已整改 | PLAN §3.1–3.3 38 行清单（目标4、来源本体15、关联19），固定 Git tree 全部路径存在；26e42049102b |
| A22 | docs/IMPLEMENTATION_PLAN.md | 对同名 Room.hpp 和功能对应的轨迹/入口/CMake 逐项比较、决定合并方式 | 已整改 | PLAN §3.4 直接比较 Room.hpp blob b6dfb2fc vs e7f63817；另对照 Path/Trajectory、CLI 与 CMake；26e42049102b |
| A23 | docs/IMPLEMENTATION_PLAN.md | 新增 ImuNoiseModel、StressModel/Tests、CameraIO/FFV1 的迁移决定及约束 | 已整改 | PLAN §3.2/3.3/3.5/3.7 说明 ImuNoiseModel、Stress、CameraIO/FFV1 的取舍及 v1 限制；26e42049102b |
| A24 | docs/IMPLEMENTATION_PLAN.md；docs/HLD.md；docs/LLD.md | 记录用户提供的 CMake 4.3.4、GCC 16.2.0、Ubuntu 24.04 运行环境，不误记为本轮实测 | 已整改 | PLAN §7.2、HLD §9.1、LLD §9.2 记录用户提供 CMake 4.3.4、GCC 16.2.0；未实机运行；26e42049102b |
| A25 | docs/IMPLEMENTATION_PLAN.md；docs/AUDIT_PROGRESS.md | 完整复核文件清单及与 v1 合同一致性；更新进度并明确 P0 代码未开始 | 已整改 | PLAN §3.7 差距矩阵完成，38/38 路径核验；SRS v1 不变、P0 代码未开始，本节状态闭环 |

### 7.3 先行快照

- 本轮拟核查范围：目标 `src/`/测试文件；来源 `src/VisualSim/`、与其编译/头引用相关的 `CameraIO` 和 `include/euroc_vio/` 依赖。
- 旧 19 项审计记录保持已整改；本轮 A20–A25 先全部标记未整改，待文件静态核查。
- 禁止修改只读来源及生产代码；**不执行编译/CTest/仿真**，也不称 P0 代码差异审查已经完整执行。


### 7.4 Round 3 实际审计与整改结论

| 阶段/范围 | 结果 | 证据 |
| --- | --- | --- |
| 审计规范与初始整改项登记 | 已完成，先于文档修改 | `898dd10ceb90abb6fe0ca55a9a770885c3c8a299` |
| 目标仓库源码清点 | 4/4 | src/Room.hpp、src/Trajectory.hpp、src/main.cpp、tests/TrajectoryTests.cpp |
| 来源 src/VisualSim 本体 | 15/15 | ftr-eskf 固定 tree 下全部头/源文件 |
| 来源 CameraIO/引用依赖 | 19/19 | 包括 src/CameraIO/、include/euroc_vio/ 和一个 VisualSlam 测试间接依赖 |
| 逐文件去留与同名冲突决策 | 已完成 | PLAN §3.1–§3.7；同名 Room.hpp 和职责对应文件具体差异 |
| 工具链事实录入 | 已完成 | PLAN/HLD/LLD 三处均注明用户提供 CMake 4.3.4、GCC 16.2.0，非本轮实测 |
| 三份设计/计划文档修订 | 已完成 | `26e42049102beb62da22bafccfad4c0ed1a2618a` |
| 来源仓库保护 | 保持只读 | 来源 ftr-eskf SHA 仍为 `fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03` |
| P0–P6 生产代码、编译/CTest | 未执行 | 文档审计不等于实际库验收 |

**Round 3 最终统计**：审计对象 PLAN、HLD、LLD 与两份审计文件共 5 份，**已审计 5/5**；A20–A25 **已整改 6/6**，无需整改 0、未整改 0。**累计 A01–A25 已整改 25/25**，旧轮次 19 项及登记时的未整改快照仍保留以供追溯。

**当前结论**：实施计划已与 **指定 ftr-eskf 固定源码版本**在迁移候选范围、同名差异及纳入/排除决策上静态对齐；但冻结的 v1 大部分运行时能力并不存在于当前目标代码，必须由后续 P0–P6 另行实现与测试。启动 P0 时需要再次获取最新 HEAD 以处理这一固定快照之后的增量。

## 8. Round 4：逐文件实施细化先行登记（2026-11-08）

> 本节先登记再变更，保持 A01–A25 的原始关闭记录；固定对照：目标 VisualSim/main（初始 HEAD `24d887447b391b3b2773197b6568228bfb80a152`）、来源只读 ros2-vio-node/ftr-eskf（`fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03`）。

| 编号 | 相对文件 | 整改目标 | 状态 |
| --- | --- | --- | --- |
| A26 | docs/IMPLEMENTATION_PLAN.md | 新增覆盖原 38 行头/源文件的详细逐文件迁移/排除说明，不改变原表格 | 已整改 |
| A27 | docs/IMPLEMENTATION_PLAN.md | §3.6 仓库列写明完整仓库名且增加分支列，原 8 行处置不变 | 已整改 |
| A28 | docs/IMPLEMENTATION_PLAN.md | 逐项说明 3.6 构建/脚本的操作和验收 | 已整改 |
| A29 | docs/IMPLEMENTATION_PLAN.md | 增补 Ubuntu 24.04/CMake/GCC/依赖的只读取证命令，不编译 | 已整改 |
| A30 | docs/AUDIT_PROGRESS.md | 逐项核实、保留旧记录、最终结论和文件/提交证据 | 已整改 |

| 本轮对象 | 审计状态 |
| --- | --- |
| docs/IMPLEMENTATION_PLAN.md | 已审计 |
| docs/AUDIT.md | 已审计（已登记本轮规则） |
| docs/AUDIT_PROGRESS.md | 已审计 |

**初始统计**：本轮 5 项，已整改 0、未整改 5；上一轮累计 A01–A25 已整改 25、未整改 0。本轮只对 docs 做静态检查，P0–P6 代码实施未执行。

### 8.1 Round 4 结果与证据

| 审计/整改项 | 最终状态 | 可核查依据 |
| --- | --- | --- |
| A26 38 文件细化 | 已整改 | §3.8 F01–F38，四个目标、十五个来源核心、十九个关联文件，每项具体处理、目标/排除和验证 |
| A27 §3.6 仓库/分支 | 已整改 | 8 行：VisualSim/main 5 行、ros2-vio-node/ftr-eskf 3 行；原处置文案完全保留 |
| A28 构建配置细则 | 已整改 | §3.9 B01–B08 对应全部 §3.6 行，补 Linux SHARED、安装、README、Windows/FFmpeg 排除与验证 |
| A29 环境命令 | 已整改 | §3.11 不带 sudo 的 OS/编译器/ABI、pkg-config、Sophus Config 只读取证，不误记为实测 |
| A30 复核和台账 | 已整改 | §3.1–3.5/§3.7 逐字保留、§8–§11 原文未改变；F=38、B=8；更新审计台账 |

**顺序**：先行登记提交 `774ec85a1c70a542f62b69fec634d8ddee59aea3` → 实施计划修订提交 `8c83b724fa97b88f70db830c46432ae8325452d0` → 本轮审计结论归档。

**本轮统计**：审计对象 3/3（PLAN、AUDIT、AUDIT_PROGRESS），A26–A30 已整改 **5/5**，无需整改 **0**，未整改 **0**；累计 A01–A30 已闭环 **30/30**。原表、既定 v1 API、数学、统计阈值保持不变。来源 ftr-eskf 保持只读，P0–P6 代码实施、编译、CTest 均未执行。

## 9. Round 5：改动前后真实代码片段审计（先行登记）

> 用户意见：此前 §3.8/§3.9 只有文字步骤，必须提供真实“改动前”与明确“计划改动后”代码。只读源码基线：VisualSim/main 和 ros2-vio-node/ftr-eskf 当前提交；目标仅 docs。

| ID | 文件相对路径 | 整改目标 | 初始状态 |
| --- | --- | --- | --- |
| A31 | docs/IMPLEMENTATION_PLAN.md | 38 个头/源文件逐项提供现有原码定位与拟修改代码，含排除项 | 未整改 |
| A32 | docs/IMPLEMENTATION_PLAN.md | 8 个构建/脚本记录逐项前后代码或明确删除/排除示例 | 未整改 |
| A33 | docs/IMPLEMENTATION_PLAN.md | 与 LLD v1 冻结 API、时间戳、无结果缓存、Room-only、ROS-free 一致 | 未整改 |
| A34 | docs/AUDIT_PROGRESS.md | 保存静态审计证据与最终覆盖率，先登记后改 | 未整改 |

初始统计：本轮已整改 0/4；未整改/待核查 4/4；Round 1–4 的 A01–A30 仍保留原有完成记录。不能把代码片段文档视为已经编译或迁移的源码。

### 9.1 Round 5 关闭状态及核查证据

| 项目 | 终态 | 实际证据 |
| --- | --- | --- |
| A31 文件前/后片段 | 已整改 | PLAN §3.12.1–3.12.4 的 F01–F38 全数存在（38/38）；每项真实旧片段+拟议新片段 |
| A32 构建/脚本对照 | 已整改 | PLAN §3.13 的 B01–B08 全数存在（8/8），B03 附 run.ps1 / clean.ps1 额外旧原文 |
| A33 冻结合同/真实性 | 已整改（静态） | GitHub 旧源码 SHA 和行号链接共 48 处；Result<T> 判错用 status；OpenCV/Eigen PUBLIC/依赖查找；仅 Room/无结果缓存 |
| A34 审计台账与进度 | 已整改 | 先行审计登记 `06b5c88cd2aff63a75d1ea94d4adaee5bd25e9ff`；PLAN 文档提交 `75bdf3576c3cbb3454e402c1d897c627e08858a0`；本节归档 |

**代码片段静态数量核对**：F=38、B=8、N=8；已有文件的代码前/后成对 46/46；旧源码行号链接 48；原 §3.1–3.11、追踪矩阵与阶段说明保留。所有片段均可能需要其上下文与头文件方可编译；**只是计划，不是已完成的代码补丁**。

**Round 5 统计**：已审计 PLAN、AUDIT、AUDIT_PROGRESS **3/3**；整改目标 **4/4 已整改**、无需整改 0、未整改 0。此前 Round 1–4 累计 30 项全部保留，总计 A01–A34 **34/34 已整改**。P0–P6 工程阶段仍未开展，尚无原生构建、CTest 或数值统计验收记录。

## 10. Round 6：实机编译环境原始证据归档（先行登记）

> 依据：用户已连续两轮提供 Ubuntu 24.04.5、内核、CMake/GCC/依赖、标准库和 CMake 配置路径的终端输出。目标是新建 `docs/BUILD_ENVIRONMENT.md` 集中归档，今后所有要求用户执行的命令均追加记录。本节先登记，再创建文件并修订实施计划。

| ID | 文件相对路径 | 整改目标 | 初始状态 |
| --- | --- | --- | --- |
| A35 | docs/BUILD_ENVIRONMENT.md | 完整记录两轮用户命令和原始输出（含 Bash find 错误） | 未整改 |
| A36 | docs/BUILD_ENVIRONMENT.md | 制定后续命令连续登记及用户反馈追加规则 | 未整改 |
| A37 | docs/IMPLEMENTATION_PLAN.md | 环境取证章节引用专用文件，不误报已构建 | 未整改 |
| A38 | docs/AUDIT_PROGRESS.md | 保存登记、提交、复核和最终状态 | 未整改 |

初始统计：Round 6 已整改 0、未整改/待核查 4。历史 A01–A34 34/34 已闭环，且本轮不修改生产 C++/CMake/源码。
