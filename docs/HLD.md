# VisualSim 运行时库概要设计（HLD）

> 状态：计划设计，尚未实施/验证。需求依据：[SRS.md](SRS.md)；接口、公式与测试细则：[LLD.md](LLD.md)。
>
> **代码唯一目标：** https://github.com/ncut-d1z/VisualSim 。**只读迁移来源：** https://github.com/ncut-d1z/ros2-vio-node/tree/main/src/VisualSim 。严禁修改来源仓库。运行平台 Ubuntu 24.04；无 ROS；Windows 禁止构建；不得为适配环境主动降级工具链。

## 1. 设计原则

- **库优先、CLI 复用**：可安装 .so 负责全部计算；命令行工具是可选调用者，负责主动导出数据集。
- **单一运动真值**：所有图像、IMU、姿态缓存、求交和数据集真值都从一套解析轨迹读取；姿态与其导数物理一致。
- **显式时间、显式副作用**：使用调用者传入的 std::int64_t 纳秒，不隐式推进仿真；每个传感器只允许时间严格递增，拒绝重复/回退；门控控制是否生成。
- **标定真正生效**：生成模块只读取当前配置；所有变更经验证后原子提交，并使受影响缓存失效；输出元数据与真实计算使用的配置一致。
- **渲染与几何同源**：可视化网格、遮挡、深度真值与射线求交共享场景几何和相机姿态，不能各自维护矛盾的平面/坐标约定。
- **不迁入 ROS 运行时**：允许 Eigen、Sophus、OpenCV、yaml-cpp 等非 ROS 依赖，公开头文件与 .so 均不得引用 ROS 类型/链接目标。

## 2. 组件及依赖边界

~~~text
Caller (C++ API)                CLI / DatasetExporter (显式导出)
           \                    /
            \                  /
           Simulator Facade / Configuration & Session
              |            |
       Trajectory       Timestamp Gate & Counters
       analytic W/B      |          |
              \          |          +-- PoseCache (最新 64 组双目)
               \         |
               Scene / Geometry (真实表面、纹理/路标、可见性)
                  |               |
            Stereo Renderer   RayCaster
                  |
           Camera calib / distort
              |
           SampleBatch  <---- ImuSynth (rigid-body + gravity + optional noise)
~~~

| 组件 | 职责 | 主要输入/输出 |
| --- | --- | --- |
| Simulator | 参数、轨迹、门控、帧号、缓存/随机状态和错误语义的唯一协调者 | generate / intersect / configure / reset |
| Trajectory | 解析位置与旋转规律及自洽运动学，含分段轨迹 | evaluate(t) → MotionState |
| Scene | 房间有限表面、离散路标/纹理、前向三深度场景、可见/遮挡几何 | World geometry、surface ID |
| StereoRenderer | 根据 W/B/C 位姿与内参、畸变生成 cam0 和 cam1 图像 | Render(scene, pose, calib) → cv::Mat |
| ImuSynth | 从同一运动真值生成 imu0 角速度、比力，应用标定、白噪及偏置游走 | MotionState + IMUConfig + dt → IMUFrame |
| PoseCache | 以相机名 + 精确整数时间键存储最近 64 组双目位姿/标定关联 | snapshot / exact lookup |
| RayCaster | 单位视线与场景有限表面求第一正向可见交点 | CameraPose + Scene + bearing → RayHit |
| DatasetExporter | 仅在显式 CLI/API 请求时写 EuRoC 风格 PNG/CSV/YAML | SampleBatch + metadata → files |
| CMake package | 共享库构建、头文件/依赖导出、安装、Linux guard、外部调用示例 | VisualSim::VisualSim |

**依赖方向**由 Facade 向计算组件单向流动；Renderer 与 RayCaster 均依赖 Scene，不相互依赖；Exporter 不进入共享库核心采样路径。不得为了复用来源仓库的 VisualSim.cpp main、头文件预编译器或 Reflect.hpp，把 ROS/common_pch 一起迁进来。

## 3. 真实坐标和运动学约定

统一：世界系 W、刚性载体系 B、左右相机 C0/C1、IMU 系 S。+X 像素向右、+Y 向下、+Z 光轴向前。**R_WB** 作用于 B 系向量并旋至 W 系；载体世界位置 **p_WB**。每个传感器使用相对于 B 的**T_BS（传感器 S → 载体 B）**，即 p_B = R_BS p_S + t_BS；相机的 S 分别为 C0/C1。和来源仓库中带 rotation_/translation_ 字段的投影代码逐项核实，不能因名字相似而直接视为同向变换。

- 相机世界位姿：R_WC = R_WB R_BC；c_W = p_WB + R_WB t_BC。
- 场景点进相机：p_C = R_WC^T (p_W - c_W)。
- 相机视线进世界：d_W = R_WC u_C；射线 p_W(λ) = c_W + λ d_W，λ > 0。
- 轨迹真值：p_WB、R_WB、v_W、a_W、ω_B、α_B。角速度/角加速度表达于 B 系；v/a 表达于 W 系。
- 一致性：v_W = dp_WB/dt、a_W = d²p_WB/dt²、Ṙ_WB = R_WB [ω_B]×、α_B = dω_B/dt（B 系分量随时刻导出）。不承诺从任意独立矛盾输入自动修复物理量。
- 重力 g_W 默认 (0, 0, -9.81) m/s²，支持显式调整并记录。

## 4. 轨迹体系

沿用目标 src/Trajectory.hpp 的 Trajectory 多态接口思想，扩展为面向单一时刻返回完整 MotionState 的解析模型：Trajectory::evaluate(time_seconds) 或等价形式。纳秒在 Facade 边界精确保存，内部只对相对秒数做解析求值。

内建轨迹组合：

| 轨迹 | 源/目标对应 | 解析状态要求 |
| --- | --- | --- |
| 静止 | 来源 PathStationary | p、R 恒定；v/a/ω/α = 0 |
| 前向匀速 | 目标 ForwardConstantVelocity | p = p0 + vt；R 恒定；a/ω/α = 0 |
| 匀加速直线 | 来源 PathAcceleration | p = p0 + v0t + ½at²；v = v0 + at；a 恒定 |
| 定轴匀速纯旋转 | 目标 OpticalAxisConstantRotation | R = R0 Exp([u ω t]×)，p 恒定；ω_B 恒定，α_B = 0 |
| 匀速圆周及朝向模式 | 来源 PathCircle | 圆周 p/v/a，R(t)、ω_B(t)、α_B(t) 从同一解析式求得 |
| 分段拼接 | 来源 PathManager | 每段使用局部时间，给出段边界连续性规则，支持组合运动 |
| 自定义解析轨迹 | 新增公开扩展接口 | 调用方定义解析函数；必须返回一致的 p/R/v/a/ω/α |

分段边界应至少保证位姿连续，若要求速度/加速度连续则在轨迹定义中明确；不连续导数的边界不得伪称角加速度可用。对无穷持续、越界、非有限输入规定明确失败状态；避免现有 clampTime 在无提示情况下隐藏非法时间。

## 5. 场景、投影和交点的一致性

### 5.1 场景模式

- **Room**：边界为 x=0/depth、y=0/width、z=0/height 的六个有限矩形面，包含现有 Room 离散网格/渲染纹理；可将来源 MeshPlot 的棋盘/三角面纹理按无 GUI 的 OpenCV 离屏方式迁入。
- **ForwardDepthScene**：保留目标仓库前向三深度稀疏路标的可视效果，但明确区分真实的有限几何片/离散目标与纯绘制符号。若要对任意视线返回“场景交点”，需构造与渲染和遮挡一致的有限表面；不能将装饰性绘制点无限扩展成实体墙面。两场景共享统一的 Scene::firstHit 接口。
- 房间几何的厘米/米、全局原点与相机位置须保持显式；相机从房间内部观测时取射线第一次正向出界命中。

### 5.2 相机模型

保留 pinhole；至少支持 4 参数 radial-tangential (k1,k2,p1,p2)，实际作用于渲染投影/重采样与可见性判断。cam0/cam1 可以有不同分辨率、内参、畸变和 T_BC。左右目作为一组在同一真实时间采样，但各自从不同光心投影，不能仅复制同一张图。默认右目基线 0.1 m，但在 T_BC（传感器到载体）约定下正确换算其正负号；最终约定以实际投影/三角几何测试验证。

**图像/求交一致性**：查询 bearing 是相机系单位物理方向，不是尚未去畸变的像素；用该方向做射线，随后按实际畸变模型投影以验证图像视域。相同场景表面的遮挡判定采用最近有效交点。

### 5.3 射线求交

针对长方体六个轴对齐有限面，分别求解 λ=(plane−c_axis)/d_axis，过滤 |d_axis| 近零、λ≤epsilon 和交点超出二维边界者；选最小 λ>0，按固定 face ID 处理棱/角等距结果。更一般 Scene 几何可用有限三角形/面片求交，但必须返回与渲染相同的最近可见表面。结果包含 input_index、world_point_m、ray_distance_m、surface_id 和状态。

## 6. 传感器生成、时序和状态机

~~~text
generate(timestamp_ns:int64, stereo, imu)
  ├─ 校验时间/轨迹/所有启用传感器的 last_timestamp
  ├─ 轨迹解析状态（纳秒保留，运动学按秒计算）
  ├─ 按门控为 cam0/cam1 / imu0 计算，事务临时结果
  ├─ 若失败：整批丢弃；时间/帧号/PRNG/缓存不改变
  └─ 若成功：一起提交帧号、last_timestamp、噪声状态和图像位姿缓存
                         ↓
                SampleBatch (0/2/1/3)
~~~

每个传感器有 last_timestamp_ns（初始未设置）及 next_frame_number（初始 0）。若目标时间等于上次成功时间，返回 DuplicateTimestamp；若更早，返回 OutOfOrderTimestamp；不创建新帧。**允许**在同一时间先请求 stereo，后请求之前未在该时刻生成的 imu（反向亦然），因为每个传感器独立记时。任何参与批量输出的传感器校验失败则整个请求失败。全部门控关闭返回空结果且无副作用。用户重启序列须显式调用 reset 或创建新实例。

IMU 使用真实采样间隔 Δt 和显式标称 rate_hz 定义初始噪声离散化；无噪模式下只输出确定性理想值。随机种子/偏置游走状态按每实例保存，不允许将异常失败的半批数据推进随机状态。

## 7. 参数变更与缓存策略

- Setter 检验后替换当前配置，从**下一次成功生成**即使用新值；不回改历史帧、不自动重发相同时间戳。读取接口返回当前生效值。
- 所有改变相机/IMU/场景/轨迹坐标关系的配置均需有明确处理。第一版采用**配置改变后使既有图像缓存失效**，让旧帧查询得到 CacheMiss / StaleConfiguration；避免把新配置套在旧帧上。
- 默认缓存 64 组**成功生成的双目图像**；按生成顺序淘汰最旧组，按（camera_id, exact timestamp_ns）检索。缓存包含左右目有效位姿（至少光心和 R_WC）、所用图像配置与场景版本，IMU-only 不写入。
- 若调用者设置相同值、改变仅 IMU 噪声或导出配置，是否需要清空图像缓存由实现按依赖关系决定，但不得使查询使用错误版本。
- 单实例默认不保证并发修改/生成线程安全；调用方需串行或外部同步。不同实例的时间、随机状态、计数器、缓存独立。

## 8. 实际 IMU 观测模型

对于刚性安装于 B 的 IMU，角速度从 ω_B 经 R_SB 转至 S；传感器原点有杆臂 t_BS：

a_WS = a_WB + R_WB (α_B × t_BS + ω_B × (ω_B × t_BS))。

比力：f_S = R_BS^T R_WB^T (a_WS − g_W)，角速度 ω_S = R_BS^T ω_B。若采用非刚性时间变化外参则不属于本版范围。

实际输出为理想值 + 对应零偏 + 白噪声；启用噪声时，白噪声标准差随 Δt 采用噪声密度离散化，偏置按随机游走推进。噪声参数必须进入真实数值计算；禁用噪声时保留无噪确定性输出。角加速度既用于运动真值返回，也影响非零杆臂的 IMU 加速度，不能只存字段不计算。

## 9. 构建与部署

目标仓库新增可安装 SHARED 库（示意名称 visualsim，导出目标 VisualSim::VisualSim），公开头文件放入 include/visualsim/，实现放 src/，可选 CLI 仅链接库。使用 CMake install(EXPORT ...)、GNUInstallDirs、CMakePackageConfigHelpers，支持 CMAKE_INSTALL_PREFIX 与可重定位的消费端依赖查找。

- CMake 配置阶段显式拒绝非 Linux（包括 Windows）；Ubuntu 24.04 为验收平台。
- 删除或禁用 Windows 专用 CMakePresets、PowerShell 构建/清理/运行入口及 README Windows 操作说明；不要保留看似可用的 Windows build 路径。
- 继续采用用户环境中**可正常编译的工具链**；因源代码真正使用 C++ 新特性而设定需要的标准，不以“兼容发行版默认版本”为由降低版本。
- 消费端只链接非 ROS 动态/静态依赖；检查导出 include、pkg 配置、链接依赖与安装运行时寻找的 .so，必要时按 Linux 规则设置 RPATH。

## 10. 迁移清单与风险

| 目标/来源文件 | 处理原则 |
| --- | --- |
| 目标 src/Trajectory.hpp | 保留前向运动与纯旋转，扩展解析 MotionState 和分段功能 |
| 目标 src/Room.hpp | 保留房间边界网格，统一与求交、渲染的实际表面几何 |
| 目标 src/main.cpp | 拆出轨迹、渲染、IMU、YAML、CSV 的库逻辑，CLI 改为调用安装式接口 |
| 来源 src/VisualSim/Camera.hpp + StereoRig.hpp | 迁移双目与投影计算，修正外参约定，增加实际畸变 |
| 来源 src/VisualSim/Path.hpp | 复用静止/匀加速/圆周/组合构想，补齐角加速度与导数一致性 |
| 来源 src/VisualSim/Room.hpp + MeshPlot.hpp | 吸收房间网格、离屏纹理生成，不携带 GUI 副作用 |
| 来源 src/VisualSim/VisualSim.cpp | 参考采样、右目、IMU、EuRoC 输出结构；不要迁入 main、文件删除副作用、宏控制开关 |
| 来源 src/VisualSim/SensorYaml.hpp | 参考 IMU 配置项，重新明确 T_BS、噪声开关及数据有效性 |
| 来源 ROS 相关配置/VisualInertial 等 | 排除 ROS 节点与算法求解器，独立仿真无需这些耦合 |

主要风险：**同名外参方向混淆、源代码畸变/噪声仅为占位、目标仅单目、分段导数不连续、重复纳秒导致帧号失真、前向点场的渲染与真实几何不一致、Windows 预设残留、库安装后依赖路径不完整**。详细缓解措施及测试见 LLD.md。

## 11. 实施分期及验证

1. P0：建立只写目标仓库的迁移清单，冻结坐标/纳秒/帧号/失败语义。
2. P1：建立纯 Linux .so、公开 API、安装导出和独立消费端示例；完成 Windows guard，不降级工具链。
3. P2：合并解析轨迹、分段规律和完整 MotionState；统一 Scene 几何。
4. P3：迁移双目离屏渲染、实际畸变、动态相机/IMU 标定；理想 IMU + 启用的噪声。
5. P4：实现帧门控与原子提交、精确纳秒缓存、可见表面射线求交。
6. P5：CLI 改用核心库，验证旧场景、数据集格式和增量测试；核验来源仓库未改动。

每期由 LLD 中对应验收条目决定是否达标，**本次文档写入不代表执行了这些阶段或通过测试**。