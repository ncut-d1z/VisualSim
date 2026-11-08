# VisualSim 文档审计进度（AUDIT_PROGRESS）

> 规则与全部目标见 [AUDIT.md](AUDIT.md)。第一阶段：**登记人工反馈，尚未开始审计**。审计中发现的问题必须按相同 ID 更新本表；跨文档条目 X 是联动核验项，不因某个单文件改动自动关闭。

## 1. 审计对象

| 文件（仓库相对路径） | 文件审计状态 | 最近审计结论 |
| --- | --- | --- |
| docs/SRS.md | 未审计 | 等待逐项审计 |
| docs/HLD.md | 未审计 | 等待逐项审计 |
| docs/LLD.md | 未审计 | 等待逐项审计 |
| docs/IMPLEMENTATION_PLAN.md | 未审计 | 等待逐项审计 |

## 2. 全量整改目标（初始化，共 28 项）

| 编号 | 文档路径 | 整改目标 | 审计结论 | 整改状态 | 证据 / 备注 |
| --- | --- | --- | --- | --- | --- |
| SRS-01 | docs/SRS.md | 补充可核验的 NFR：性能/内存边界、并发与多实例、错误恢复/可观测性、ABI 与可重定位安装；未批准的数值不虚构 | 待审计 | 未整改 | — |
| SRS-02 | docs/SRS.md | 集中列出外部接口需求：运动操作、Generate、Raycast、相机创建/标定、配置加载、历史查询、会话重置及输入/输出/错误类别 | 待审计 | 未整改 | — |
| SRS-03 | docs/SRS.md | 补术语表、假设/依赖、范围外；统一 W/B/C/S、T_BC/T_BS、camX、作用域、事件历史、门控和有效时间 | 待审计 | 未整改 | — |
| SRS-04 | docs/SRS.md | 去除旧设计“最近 64 组缓存”的直接引用及实现措辞；只保留不得保存历史计算结果的产品约束 | 待审计 | 未整改 | — |
| SRS-05 | docs/SRS.md | 补强 AC-06 噪声、AC-09 无缓存等验收的可操作判据；无证据的性能阈值不擅定 | 待审计 | 未整改 | — |
| SRS-06 | docs/SRS.md | 建立 FR/AC → LLD 接口或算法 → V 用例 → P 阶段追踪关系，并保持唯一主表 | 待审计 | 未整改 | — |
| HLD-01 | docs/HLD.md | 上收架构原则、下沉圆周重锚、事件排序、append-only、同刻处理、门控校验和导数单侧等 LLD 细节 | 待审计 | 未整改 | — |
| HLD-02 | docs/HLD.md | 补公共接口类别与模块间接口概览和数据合同 | 待审计 | 未整改 | — |
| HLD-03 | docs/HLD.md | 补非功能设计：复杂度/无缓存重算、并发/多实例、错误、日志/可观测性、内存与安装 | 待审计 | 未整改 | — |
| HLD-04 | docs/HLD.md | 补技术风险及权衡：无缓存性能、浮点复现、Eigen/OpenCV ABI、历史冻结、配置 schema 演进 | 待审计 | 未整改 | — |
| HLD-05 | docs/HLD.md | 集中列出第三方依赖与平台/时间/场景假设 | 待审计 | 未整改 | — |
| HLD-06 | docs/HLD.md | 移除旧 PoseCache 等迁移变更叙述，变更背景移至实施计划/ADR | 待审计 | 未整改 | — |
| LLD-01 | docs/LLD.md | 冻结外部接口签名、必选重载、参数单位、版本策略，不再用“可微调/待实现时定”替代明确合同 | 待审计 | 未整改 | — |
| LLD-02 | docs/LLD.md | 统一错误返回/异常策略：Status/Result 值语义、无值含义、消息规范和错误映射 | 待审计 | 未整改 | — |
| LLD-03 | docs/LLD.md | 明确单实例并发与多实例隔离、Set*/Generate/Raycast/历史读取同步合同 | 待审计 | 未整改 | — |
| LLD-04 | docs/LLD.md | 明确共享库 ABI 策略、Pimpl/可见性、cv::Mat 所有权、生命周期和异常安全 | 待审计 | 未整改 | — |
| LLD-05 | docs/LLD.md | 细化 Evaluate 初始基线/时间域、SetPose 与圆周、出栈重锚、切换单侧导数、Raycast 畸变视域 | 待审计 | 未整改 | — |
| LLD-06 | docs/LLD.md | 将详细测试 V01–V11 移到实施计划中的测试/追踪部分，LLD 保留测试接口或交叉引用 | 待审计 | 未整改 | — |
| LLD-07 | docs/LLD.md | 冻结 YAML schema、字段单位/合法性、同刻顺序、配置原子性、IMU 噪声单位/随机过程或显式记录待决点 | 待审计 | 未整改 | — |
| PLAN-01 | docs/IMPLEMENTATION_PLAN.md | 明确实施计划定位，不替代 SRS/HLD/LLD，并只记任务/验证/风险 | 待审计 | 未整改 | — |
| PLAN-02 | docs/IMPLEMENTATION_PLAN.md | 减少设计决定重复叙述，改为引用唯一的 SRS/HLD/LLD 章节与需求编号 | 待审计 | 未整改 | — |
| PLAN-03 | docs/IMPLEMENTATION_PLAN.md | 加入排期/负责人（未知则待定）、风险登记、依赖版本冻结策略、CI 平台矩阵、回滚方案及验收追踪 | 待审计 | 未整改 | — |
| PLAN-04 | docs/IMPLEMENTATION_PLAN.md | 为各阶段交付列明可执行的验收命令/操作检查清单（不声称已运行） | 待审计 | 未整改 | — |
| X-01 | docs/SRS.md; docs/HLD.md; docs/LLD.md | 跨文档层级消歧：SRS 不含迁移与旧缓存细节、HLD 不含 LLD 过程、LLD 测试移计划 | 待审计 | 未整改 | — |
| X-02 | docs/SRS.md; docs/HLD.md; docs/LLD.md | 集中管理分散的接口/schema/噪声/边界规则未决项；冲突应标待决而非假称已冻结 | 待审计 | 未整改 | — |
| X-03 | docs/SRS.md; docs/HLD.md; docs/LLD.md | 统一术语与坐标系命名，引用唯一术语定义，检查与事件历史、动态相机一致 | 待审计 | 未整改 | — |
| X-04 | docs/SRS.md; docs/HLD.md; docs/LLD.md; docs/IMPLEMENTATION_PLAN.md | 建立并核验跨文档 FR/AC/LLD/V/P 可追溯矩阵，禁止孤儿需求与验收 | 待审计 | 未整改 | — |
| X-05 | docs/SRS.md; docs/HLD.md; docs/LLD.md; docs/IMPLEMENTATION_PLAN.md | NFR 自需求到架构、接口/实现约束、风险/验收贯通，未决性能阈值单列 | 待审计 | 未整改 | — |

## 3. 统计（登记阶段）

- 文档审计：0 / 4 已审计；0 正在审计。
- 人工目标：28 / 28 已登记，0 已整改，28 未整改，0 待决；**本阶段尚未核验任何目标**。
- 本轮修改范围：只能写入 ncut-d1z/VisualSim 的 docs/，第一个仓库只读。
- 下一步：依据 AUDIT.md 第 3 节逐项审计与实际整改；每次提交后更新本表状态、证据、SHA。
