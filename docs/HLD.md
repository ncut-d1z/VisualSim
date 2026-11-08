# VisualSim 运行时库概要设计（HLD）

> 状态：**v1 设计基线（本轮文档审计后冻结，待代码实施与功能验证）**。功能验收以 [SRS.md](SRS.md) 为准，逐接口及公式见 [LLD.md](LLD.md)；**源文件迁移及实施清单移至 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md)**。本文件不包含迁移方向表。
>
> 目标：Ubuntu 24.04 / Linux 独立可安装共享库；无 ROS；Windows 配置失败；不为迁就发行版默认工具链而强制降级。

## 1. 核心设计：带时间戳的命令历史驱动解析仿真

采用**即时式命令 API + 带时间戳的不可随意改写的配置事件历史**：

1. 调用者提交 BeginMotionMode、设置线/角运动模式及参数、EndMotionEnd、相机创建/重命名/标定变更、Room 参数变更等事件，每条携带生效时间。
2. 系统保存当前有效设置与**全部用户修改事件**；不会保存按帧计算出的物体位姿、图像、IMU 数据或射线交点。
3. 查询时以输入时间 t 为截点，从初始设置开始按事件时间顺序解析，并对被更改时间分隔出的每段运动应用对应解析规律，累积到 t。
4. 只读查询运动状态/射线交点不依赖某一图像已生成，也不改变事件历史、运动作用域或采样计数。

系统不保留历史计算结果；历史记录仅由调用者提交的业务配置变化组成。任意有效时刻的几何查询无需预生成图像。

## 2. 概念模块

~~~text
C++ Caller          Config File Parser           Optional CLI/Exporter
     \                    |                          /
      \                   |                         /
        Public Simulator API + Config Timeline
          |                 |                    |
          |                 |                    +-- Camera Registry (0..N)
          |                 |                    +-- IMU Calibration
          |                 +-- Scope Stack (BeginMotionMode / EndMotionEnd)
          |                 +-- Linear/Angular Mode Events
          |                 +-- Timestamped Camera/Room Events
          |
       Historical Evaluator (on demand, no result cache)
          |             |                |
      Analytic       Room Scene        Sensor Models
      Motion         only             Camera 0..N / IMU
          |             |                |
          +-------------+----------------+
                  |             |
               Ray Query     Generate Frames
                             (only minimal per-sensor generation counters)
~~~

| 模块 | 职责 | 保留的内容 |
| --- | --- | --- |
| Public Simulator | 提供事件式运动 API、配置文件装载、按时间查询、门控生成、重置会话 | 当前业务配置、必要 API 控制状态 |
| EventTimeline | 按纳秒时间和稳定次序维护全部修改事件，可读取/回放 | **用户修改记录**（允许的唯一历史数据） |
| MotionScope | 在作用域开始时暂存两类当前模式与参数；嵌套栈按 LIFO 恢复 | 活跃作用域结构，属于运动模式状态 |
| HistoricalEvaluator | 按需重放时间区间并拼接解析运动，得到 t 的 p/R/v/a/ω/α | 不长期保存计算得到的位姿或分段中间结果 |
| CameraRegistry | 管理 0..N 台相机、稳定内部身份、camX 自动名及可变显示名 | 相机创建/命名/标定的当前值和修改事件 |
| Room | 唯一有限几何场景，四墙、地板、天花板、可渲染纹理 | Room 当前参数及其修改历史 |
| CameraRenderer | 结合历史相机位姿、内参、外参、畸变生成每台相机图像 | 不缓存图像 |
| ImuModel | 结合历史运动真值、IMU 外参和噪声生成测量值 | 不缓存 IMU 历史样本/偏置历史 |
| RayQuery | 以历史时刻相机位姿和 Room 表面返回最近有效交点 | 不缓存位姿或求交结果 |
| FrameSequencer | 保证严格递增、编号及事务提交 | 仅最近成功时间、下一编号等最小控制状态 |

CLI/Exporter 可在显式调用时将输出写为 PNG/CSV/YAML；不得把 CLI 中某两条硬编码轨迹当作核心的唯一运动。可用示例配置文件表达“前向匀速”“定轴匀速旋转”等已有场景，**统一使用 Room**。

## 3. 对外接口及内部模块合同概览

| 接口组 | 调用方可见的输入/输出 | 主要责任模块 |
| --- | --- | --- |
| 即时式运动 | 生效纳秒、Begin/End、线/角模式与参数 → 事件提交结果 | Public Simulator、MotionScope、EventTimeline |
| 配置文件与历史 | 配置路径/变更请求 → 完整有效事件序列；时间 → 历史配置与运动真值 | 配置解析、EventTimeline、HistoricalEvaluator |
| 动态相机管理 | 新外参/生效时间、名称、标定 → 稳定传感器身份与对应时刻参数 | CameraRegistry、EventTimeline |
| 按需生成 | 时间与 camera/imu 门控 → N 台相机及可选 IMU 帧、错误码 | FrameSequencer、Renderer、ImuModel |
| 真实求交 | 时间、相机标识、单位视线列表 → Room 最近表面交点或逐项失败状态 | HistoricalEvaluator、Room、RayQuery |
| 安装与导出 | 安装前缀或显式文件输出配置 → 可消费库/按需数据集 | CMake package、Exporter |

模块之间共享定义明确的**时刻快照（单次调用临时值）**：解析运动状态、该时刻相机/IMU 参数、Room 参数和请求时间。时间线只保留用户输入的配置修改事件；瞬时快照不被跨调用持久化。错误返回策略、快照字段和全部时序边界下沉到 LLD。

### 3.1 运行时交互视图：事件提交与按需生成

~~~mermaid
sequenceDiagram
    participant Caller as C++ / Config Caller
    participant API as Simulator API
    participant Timeline as EventTimeline
    participant Eval as HistoricalEvaluator
    participant Seq as FrameSequencer
    participant Cam as CameraRenderer
    participant IMU as ImuModel
    Caller->>API: Begin/Set/End(effective_ns, params)
    API->>Timeline: 校验事件/追加有效变更
    Timeline-->>API: 事件提交状态
    API-->>Caller: Result
    Caller->>API: Generate(at_ns, camera/imu gate)
    API->>Seq: 校验被启用传感器的时间与编号
    API->>Eval: 读取时刻 at_ns 的真值/标定
    Eval->>Timeline: 按时间回放用户配置历史
    Timeline-->>Eval: 有效事件
    Eval-->>API: 临时运动/传感器状态
    opt camera gate
        API->>Cam: 渲染当时存在的全部相机
        Cam-->>API: 临时图像帧
    end
    opt imu gate
        API->>IMU: 生成该时刻的测量
        IMU-->>API: 临时 IMU 帧
    end
    API->>Seq: 全部成功后原子提交最小编号/时间状态
    API-->>Caller: SampleBatch / Status
~~~

此图说明**按需重放 + 批量事务提交**，不意味着每条事件会重新采样，也不引入历史帧缓存。错误时不执行最终提交。

### 3.2 运行时交互视图：任意时间的只读 Raycast

~~~mermaid
sequenceDiagram
    participant Caller
    participant API as Simulator API
    participant Eval as HistoricalEvaluator
    participant Timeline as EventTimeline
    participant Room as Room Geometry
    Caller->>API: Raycast(time_ns, camera, bearings)
    API->>Eval: 获取指定时间的姿态和相机标定
    Eval->>Timeline: 回放 <= time_ns 的事件
    Timeline-->>Eval: 有效模式和参数历史
    Eval-->>API: 临时世界位姿/相机配置
    API->>Room: 查询每条射线的最近正向可见交点
    Room-->>API: 交点或逐项失败状态
    API-->>Caller: RayHit 向量
~~~

Raycast **不调用 Generate**，不要求图像已生成，不变更采样状态。两类流程的时序及错误优先级以 LLD §§2、7、8 为准。

## 4. 运动组合与作用域语义

### 4.1 每类恰有一个有效模式

运行时运动状态包含两个逻辑槽：

- LinearMode ∈ {Stationary, ConstantVelocity, ConstantAcceleration, UniformCircular}
- AngularMode ∈ {Stationary, FixedAxisConstantRotation}

同类新设置为**替换**，不可并行叠加；跨类共同作用于 p_WB(t) 和 R_WB(t)。默认两个槽均 Stationary。线运动独立定义物体在**世界坐标系**中的位置规律；角运动独立定义朝向变化。改变朝向本身**不自动改变**已指定世界系的线速度。若希望匀速圆周且镜头持续转向某方向，可同时配置兼容的角运动模式及初始朝向。

### 4.2 运动作用域

对外提供 BeginMotionMode / EndMotionEnd。作用域保存进入时的两类模式及其参数，后进先出；类内更新覆盖当前类、跨类互不干扰。退出作用域恢复旧**运动规律**，并以退出时刻的运动状态继续前进，不能产生非显式的位置/朝向跳变。

事件历史包含操作生效时间和稳定顺序。架构层只规定“按生效时间分段累计解析运动、可回溯重算”，**同刻事件排序、圆周重锚细则、边界导数、事件历史冻结和配置失败事务规则**均由 LLD §3–4 定义。

## 5. 可定义的解析运动状态

统一世界 W / 载体 B / 相机 C / IMU S 的约定及转换方向见 [SRS §1.1](SRS.md#11-术语与符号)。历史解析器对外返回完整 MotionState：世界系位置、朝向、线速度、线加速度，载体系角速度、角加速度；各量来自同一个解析规律而非相互冲突的独立输入。

内建线运动支持静止、恒速直线、恒加速直线和匀速圆周；角运动支持静止与定轴匀速纯旋转，两类可组合。多段运动在切换时累计位姿，采用可诊断的不连续导数规则。具体公式与时间端点约定见 LLD §4；本概要设计不复制逐段解析公式。

## 6. 相机与 IMU：历史化注册和标定

### 6.1 动态相机注册

相机注册表容纳任意数量相机，以稳定身份而非可变名称关联采样序号。默认 cam0 的外参为 (I3,0)，且默认/仅输入外参注册的相机均须具备**可立即成像的完整默认标定**（K、图像尺寸、零畸变和标称频率）；读取与渲染采用同一配置。若默认 cam0 已有显式修改或采样历史，不得由后续首次 CreateCamera 覆盖；自动 camX 名称遇已有名称占用须跳过冲突而继续唯一分配。创建、重命名和标定变化都是带生效时间的事件，历史查询只读取该时刻有效的相机和名称。具体默认值和身份分配规则由 LLD §5 统一冻结。

### 6.2 传感器标定对计算生效

相机的独立内参、外参、畸变与分辨率实际影响生成图像和有效视域；IMU 外参、噪声及**世界系固定重力 (0,0,-9.81) m/s²**参与观测模型。重力在 v1 中属于不随事件更改的物理常量，不提供配置项；因此无需记录重力修改历史。所有传感器在同一历史运动真值基础上采样，不能用最新标定重解释旧时间。详细变换公式、噪声模型及时间合同见 LLD §5–6。

## 7. Room 唯一场景、图像与求交

场景只包含 Room：四面墙壁、地板、天花板以及与这些有限面一致的可见纹理。没有 ForwardDepthScene，也不保留前向三深度路标场作为可选场景。

- 图像生成：每台在该时刻注册的相机，使用该时刻的 p/R 和本机内参、畸变对 Room 渲染；camera gate 一次生成全部当前相机。
- 视线查询：给定 camera_id、time_ns、相机系单位方向 std::vector，**按事件历史重算**此时相机的世界位姿，求 Room 最近正向、在该相机有效视域内的可见表面点，按输入索引返回对应世界坐标或无命中原因。
- **不要求此前在 time_ns 生成过图像**；没有 pose cache/cache miss 的概念。历史参数和模式完整即可求交。
- 查询只读，可无副作用查询过去、未来以及同一时刻多次；生成帧则受每传感器时间严格递增的限制。

## 8. 生成时序与必要的最小控制状态

调用者输入纳秒时间、camera 和 imu 两个门控。camera 覆盖该时刻存在的全部相机，imu 控制 imu0；每传感器时间必须严格递增。同一调用对全部目标传感器按“验证—计算—提交”原子处理，失败不产生部分输出或消耗编号。只保存拒绝重复和递增编号所需的最小协议状态，不保存任何历史帧。

公共 Generate 和只读 Raycast 使用同一历史解析器，但**Raycast 不依赖 Generate，且不改变采样计数**。确切门控状态、重置和原子性条件见 LLD §8。

## 9. 数据与安装边界

### 9.1 平台、依赖与环境假设

运行/验收平台 Ubuntu 24.04；明确 Linux-only，非 Linux 构建被拒绝。用户现有编译工具链满足两个参考仓库的构建要求，**无需强制降级**。Eigen 用于几何类型，OpenCV 用于图像，Sophus 可用于 SE(3)/SO(3) 计算，yaml-cpp 可用于配置解析；仅为实际使用的组件添加依赖，不引入任何 ROS/ament/cv_bridge。精确版本在首次构建验收前冻结并登记于实施计划，不在此编造版本号。



公开头文件与共享库不引用 ROS 类型或链接目标。Linux .so 通过 CMAKE_INSTALL_PREFIX 安装，并提供可重定位 CMake 配置与独立消费端示例。用户现有高版本编译工具链可以保留；CMake 在非 Linux（特别是 Windows）时明确拒绝。不得附带声称可工作的 Windows build preset/PowerShell 入口。

### 9.2 部署视图：安装树和外部消费者

~~~mermaid
flowchart LR
    Consumer["独立 C++ 消费端"] --> Package["VisualSimConfig.cmake / VisualSimTargets.cmake"]
    Package --> Shared["libvisualsim.so (SONAME 1)"]
    Consumer --> Headers["安装后的公开头文件"]
    Shared --> Eigen["Eigen3"]
    Shared --> OpenCV["OpenCV"]
    Shared -. "按实际功能可选" .-> Sophus["Sophus / yaml-cpp"]
    CLI["可选 CLI / DatasetExporter"] --> Shared
~~~

所有实体均来自**目标仓库的 Linux 安装产物或非 ROS 依赖**。消费者在安装前缀外使用 find_package 和 VisualSim::VisualSim，运行时由 Linux 动态装载器解析 .so；部署不依赖 ROS 工作空间。图中的 Sophus / yaml-cpp 表示按实际使用选择，不意味着必须引入未使用的链接依赖；构建标准和实际库版本在首次实际构建时记录，禁止强制降低用户已有工具链版本。

可选 CLI/Exporter 从配置文件中读时间戳运动和标定事件，再使用同一库 API 采样/写盘；原有前向匀速/纯旋转作为配置示例保留**运动能力**，但两种运动均在 Room 场景上运行。不得在 main.cpp 中重新写死运动轨迹，也不得无提示递归删除输出目录。

## 10. 非功能设计与质量属性

| NFR | 架构性选择 | 验证入口 |
| --- | --- | --- |
| NFR-01 持久内存 | 仅事件时间线 + 当前设置 + 必需协议控制量；按需重算，不保留历史结果 | 状态对象审查、重复请求前后驻留量对比 |
| NFR-02 查询性能 | 仅保留测量延迟、驻留内存和事件重放工作量的能力，当前**未实施正式性能验收**；首次生成有效数据后再考虑预算 | **延期项目；目前不通过也不阻断功能交付** |
| NFR-03 并发隔离 | 单实例采用外部串行化合同；不同实例无可变共享模拟状态 | 多实例交叉生成、同实例互斥验证 |
| NFR-04 失败回滚 | 配置事件与生成结果先在临时事务中完整准备；结果返回与状态提交必须采用不抛异常路径或具备回滚，禁止部分成功 | 错误注入 + 已提交配置、帧编号和时间不变检查 |
| NFR-05 可观测性 | 返回稳定程序化状态码和可诊断消息，日志可选且无隐式写盘 | 错误码覆盖和无文件副作用检查 |
| NFR-06 二进制部署 | Linux .so、安装式 CMake package、导出依赖；公开 ABI 边界显式声明 | 安装树外消费者、依赖扫描 |
| NFR-07 复现 | 整数纳秒事件、同刻稳定次序、按时间确定性计算与随机过程 | 重放事件序列、固定种子对比 |

这里的 O(E) 为最直接的线性扫描上界，不是已实测性能；不允许为提速引入历史位姿/帧缓存。程序首次成功运行并生成有效数据之前不设吞吐量或内存预算，也不进行性能验收。

## 11. 风险、备选方案及权衡

| 风险 | 采用的设计及理由 | 代价、缓解或待决事项 |
| --- | --- | --- |
| 无结果缓存造成反复查询成本高 | 坚持用户禁止历史结果缓存的约束，直接基于事件重算 | 首次有效运行后再决定是否做基准；可优化解析公式/事件遍历，不准悄加缓存；当前性能验收延期 |
| 浮点秒与整数纳秒一致性 | 事件索引和输出使用 std::int64_t，解析公式使用明确转换的秒 | 同构工具链可复现，跨架构逐位一致不承诺；**LLD §2.4 已冻结 SO(3)/法向/四元数容差，§7 已冻结 Room 求交容差**；验证入口为 PLAN V04/V08 |
| 公共接口依赖 Eigen/OpenCV | 首期 C++ 原生接口直接满足用户调用需求 | 二进制兼容限定构建/运行环境；SONAME 与 ABI 改动受控 |
| “过去查询”与历史冻结冲突 | 查询允许随机读取；配置的回溯修改必须保护已输出数据一致性 | 会话重置与历史替换是不同操作，错误明确；精确边界见 LLD |
| YAML 配置长期演进 | 版本化 schema，与函数调用转换为同一事件流 | 未知字段/版本显式报错，扩展采用受控版本升级 |
| 解析噪声随机游走不缓存 | 用种子、时间和规则导出可重复过程 | 大时域随机访问性能风险，具体过程/单位由 LLD 定义，不存历史 IMU 结果 |

## 12. 设计重点与验证

- 分段累计、出栈恢复及圆周重新锚定必须兼顾参数合同与 p/R 连续性。
- 事件历史可供历史查询；配置更新须保护已经输出的数据定义不被事后改写。
- 动态相机创建/改名与逐传感器防重复规则必须共存。
- 配置变化后历史查询照用当时有效标定，不使用旧位姿缓存或最新配置回填。
- 非零 IMU 噪声与偏置游走必须真实生效，且禁止以历史样本缓存实现。
- 详细 API、时间边界、误差处理、解析表达式见 LLD.md；测试与任务分期见 IMPLEMENTATION_PLAN.md。
