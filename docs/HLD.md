# VisualSim 运行时库概要设计（HLD）

> 状态：设计提案，尚未实施。功能验收以 [SRS.md](SRS.md) 为准，逐接口及公式见 [LLD.md](LLD.md)；**源文件迁移、排期及任务清单移至 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md)**。本文件不包含迁移方向表。
>
> 目标：Ubuntu 24.04 / Linux 独立可安装共享库；无 ROS；Windows 配置失败；不为迁就发行版默认工具链而强制降级。

## 1. 核心设计：带时间戳的命令历史驱动解析仿真

采用**即时式命令 API + 带时间戳的不可随意改写的配置事件历史**：

1. 调用者提交 BeginMotionMode、设置线/角运动模式及参数、EndMotionEnd、相机创建/重命名/标定变更、Room 参数变更等事件，每条携带生效时间。
2. 系统保存当前有效设置与**全部用户修改事件**；不会保存按帧计算出的物体位姿、图像、IMU 数据或射线交点。
3. 查询时以输入时间 t 为截点，从初始设置开始按事件时间顺序解析，并对被更改时间分隔出的每段运动应用对应解析规律，累积到 t。
4. 只读查询运动状态/射线交点不依赖某一图像已生成，也不改变事件历史、运动作用域或采样计数。

与上一版架构不同：**删除 PoseCache、StereoPoseSnapshot、最近 64 组缓存及需要先产生图像才能求交的约束**。历史记录属于用户修改日志，不是计算结果缓存。

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

## 3. 运动组合与作用域语义

### 3.1 每类恰有一个有效模式

运行时运动状态包含两个逻辑槽：

- LinearMode ∈ {Stationary, ConstantVelocity, ConstantAcceleration, UniformCircular}
- AngularMode ∈ {Stationary, FixedAxisConstantRotation}

同类新设置为**替换**，不可并行叠加；跨类共同作用于 p_WB(t) 和 R_WB(t)。默认两个槽均 Stationary。线运动独立定义物体在**世界坐标系**中的位置规律；角运动独立定义朝向变化。改变朝向本身**不自动改变**已指定世界系的线速度。若希望匀速圆周且镜头持续转向某方向，可同时配置兼容的角运动模式及初始朝向。

### 3.2 BeginMotionMode / EndMotionEnd

- BeginMotionMode(t)：把当前两个槽的模式与参数压入一层作用域。
- SetLinearMode / SetAngularMode / SetMotionParameter：在当前层对应槽内覆盖；另一槽不受影响。
- EndMotionEnd(t)：弹出最近层，恢复进入时保存的两个槽的模式及参数。

这种“恢复”**只恢复运动规律**，不是恢复 Begin 之前的物体绝对位姿。每次切换以当时已累计的 p/R 为新起点，保证普通模式切换及退出作用域时位置/朝向连续。速度、加速度可以在切换处跳变；这些导数在不连续的边界只按明确的单侧约定输出。

圆周参数必须有能随新起点恢复的表示法：将圆周的有向半径、平面、相位、角速率视为**相对于激活时刻的模式参数**。若调用者指定世界系圆心，则在首次激活时得到其相对半径；作用域退出重新激活旧圆周模式时，以当前世界位置重新锚定同样的圆周规律，从而避免把物体瞬间拉回旧圆心。若用户显式要求绝对世界圆心不变且与当前轨迹连续性矛盾，应报告冲突而非隐式跳位姿。

### 3.3 变更时间与解析累计

事件记录排序键为（TimestampNs, stable insertion_order）。系统从已定义初始状态开始，逐段计算运动：

~~~text
initial state at simulation origin
 -> integrate with active Linear/Angular modes to event_1.timestamp
 -> apply all events at event_1 timestamp in sequence
 -> integrate with updated modes to event_2.timestamp
 -> ...
 -> integrate to query_time t
 -> return motion state at t
~~~

每个事件作用于该时间左侧累积的位姿；普通模式/参数切换成为新的局部解析段。绝对位姿显式设置是独立的状态重设事件，可以改变连续性，必须标明。事件时间戳相同按提交顺序处理，最后更新的同类参数生效；空作用域/未配对 End 报错。

若用户已经成功生成了数据，不能事后在其时间戳及之前插入会改变历史含义的事件。第一版采用**append-only、时间非递减的配置事件序列**；对已输出过传感器数据的时间点及之前的后续事件编辑拒绝（可通过 reset/new session 后重新构造整段历史）。

## 4. 可定义的解析运动状态

统一 W 世界系、B 载体系。时间点 t 的 MotionState 包含：

- p_WB，R_WB；
- v_W、a_W；
- ω_B、α_B。

p/v/a 在世界系，ω/α 在载体系。运动学状态须自洽；以下模式定义为解析规律，而不是一组互相独立且可任意冲突的真值字段：

| 线运动模式 | 解析位置 | 相关导数 |
| --- | --- | --- |
| 静止 | p(t)=p_anchor | v=a=0 |
| 匀速直线 | p(t)=p_anchor+v·Δt | v 常量、a=0 |
| 匀加速直线 | p(t)=p_anchor+v0·Δt+½a·Δt² | v=v0+a·Δt |
| 匀速圆周 | p_anchor 加上轨道圆周的相对解析位移 | v、a 由同一圆周规律导出 |

| 角运动模式 | 解析朝向 | 相关导数 |
| --- | --- | --- |
| 静止 | R(t)=R_anchor | ω=α=0 |
| 定轴匀速旋转 | R(t)=R_anchor Exp([uωΔt]×) | ω_B=uω，α_B=0 |

这两种角模式不支持段内非零角加速度；若设置非零值，不接受与现有模式不一致的配置。六项物理量都可以查询；不连续切换点的单侧状态规则在 LLD 中固定。

## 5. 相机与 IMU：历史化注册和标定

### 5.1 自动相机命名与默认相机

- 初始未显式创建相机时，查询/生成视作拥有一个 cam0，其 T_BC 为 (I3,0)。
- 如果在第一次使用前先显式注册相机，首个注册名为 cam0，不额外创建第二台默认 cam0。
- 如果已通过默认机制实际使用 cam0，之后每次新增相机依次得到 cam1、cam2……；不回收曾用过的自动编号，即使改名也不回收。
- 相机身份为不可变内部 ID，名称为按时间生效的别名。传感器流的唯一性和帧编号按 ID 追踪，改名不重置。
- 配置历史需记载创建、改名、内参、外参、畸变等每次更改。按时间查询 t 时仅枚举 t 当时已存在的相机与当时的名称和参数。

### 5.2 坐标与实际生效

T_BC 是相机坐标系到载体系的刚体变换；T_BS 是 IMU 到载体系的刚体变换。对每个 t，用同一个历史轨迹得到 R_WB、p_WB，再结合该时刻的传感器外参得到相机光心/朝向与 IMU 比力。相机各自用独立的标定、畸变进行成像。改变参数后以事件生效时间为界，过去查询使用旧参数，未来查询使用新参数。没有镜头个数上限硬编码。

IMU 默认无噪；启用后根据噪声密度、偏置游走和固定种子按请求时间产生数值。为了遵守“不缓存 IMU 样本或偏置历史”，随机过程必须能够按种子和时间**确定性重算**；仅用新的输出帧号而丢失时间相关性的白噪或偏置状态方案不合格。额外不建立运动/图像结果缓存。

## 6. Room 唯一场景、图像与求交

场景只包含 Room：四面墙壁、地板、天花板以及与这些有限面一致的可见纹理。没有 ForwardDepthScene，也不保留前向三深度路标场作为可选场景。

- 图像生成：每台在该时刻注册的相机，使用该时刻的 p/R 和本机内参、畸变对 Room 渲染；camera gate 一次生成全部当前相机。
- 视线查询：给定 camera_id、time_ns、相机系单位方向 std::vector，**按事件历史重算**此时相机的世界位姿，求 Room 最近正向、在该相机有效视域内的可见表面点，按输入索引返回对应世界坐标或无命中原因。
- **不要求此前在 time_ns 生成过图像**；没有 pose cache/cache miss 的概念。历史参数和模式完整即可求交。
- 查询只读，可无副作用查询过去、未来以及同一时刻多次；生成帧则受每传感器时间严格递增的限制。

## 7. 生成时序与必要的最小控制状态

调用者一次输入 time_ns:int64、camera gate、imu gate。系统先解析该时刻有效传感器集合及参数，检查所有参与传感器的最近成功时间；只有当该传感器此前未输出，或 time_ns 严格大于其最近成功时间时才允许输出。任何失败**整批回滚**；同一时间的不同传感器分别允许输出。

每个传感器仅保存：（last_successful_time_ns, next_frame_number）等最小控制变量；这些是门控序号状态，**不允许将其扩展成按帧历史缓存**。camera gate 覆盖所有当时存在的相机，数目可为 1、2、N；各自编号，按稳定创建顺序组织输出；imu0 最后。改名不更改其 ID 与编号。全关闭无副作用。reset 用于启动新采样序列；是否清空已定义运动历史应由单独的重新配置操作决定，不能把二者混为一谈。

## 8. 数据与安装边界

公开头文件与共享库不引用 ROS 类型或链接目标。Linux .so 通过 CMAKE_INSTALL_PREFIX 安装，并提供可重定位 CMake 配置与独立消费端示例。用户现有高版本编译工具链可以保留；CMake 在非 Linux（特别是 Windows）时明确拒绝。不得附带声称可工作的 Windows build preset/PowerShell 入口。

可选 CLI/Exporter 从配置文件中读时间戳运动和标定事件，再使用同一库 API 采样/写盘；原有前向匀速/纯旋转作为配置示例保留**运动能力**，但两种运动均在 Room 场景上运行。不得在 main.cpp 中重新写死运动轨迹，也不得无提示递归删除输出目录。

## 9. 设计重点与验证

- 分段累计、出栈恢复及圆周重新锚定必须兼顾参数合同与 p/R 连续性。
- 事件历史可供历史查询；配置更新须保护已经输出的数据定义不被事后改写。
- 动态相机创建/改名与逐传感器防重复规则必须共存。
- 配置变化后历史查询照用当时有效标定，不使用旧位姿缓存或最新配置回填。
- 非零 IMU 噪声与偏置游走必须真实生效，且禁止以历史样本缓存实现。
- 详细 API、时间边界、误差处理、解析表达式和测试见 LLD.md；任务分期及代码融合见 IMPLEMENTATION_PLAN.md。
