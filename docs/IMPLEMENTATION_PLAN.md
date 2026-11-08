# VisualSim 独立运行时库实施计划（IMPLEMENTATION_PLAN）

> **状态：v1 文档设计基线（经本轮审计整改后冻结），工程实施 P0–P6 尚未开展。** 本文件专门管理工程实施和源码融合。功能需求见 [SRS.md](SRS.md)，概要架构见 [HLD.md](HLD.md)，API/算法合同见 [LLD.md](LLD.md)。文档审计不代表生产代码实现或编译/测试通过。

## 1. 仓库写入边界与代码基线

| 角色 | 仓库 | 权限与用途 |
| --- | --- | --- |
| **唯一目标仓库** | https://github.com/ncut-d1z/VisualSim | **允许后续修改源码、CMake、示例及 docs，并交付独立库** |
| **仅供读取的来源仓库** | https://github.com/ncut-d1z/ros2-vio-node/tree/ftr-eskf/src/VisualSim | **只读，用于审查并把适用新代码融合入目标仓库。禁止任何提交、推送、创建 PR 或文件更改** |

历史文档基线：目标仓库四份设计文档修订提交 69d772719aca217456ac936ae7718cdbdc4afb5c。**本次源码差异清点冻结比较提交**：目标 main d4b38f2ea73fa5aef700b303517a45ec21281561；只读来源 **ftr-eskf** fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03（不可再用旧 main f1cb7a... 作为本轮迁移版本）。审计提交及当前文档状态见 [AUDIT_PROGRESS.md](AUDIT_PROGRESS.md)。P0 实际实施前须重新读取两个仓库当前 HEAD，并说明与本次固定快照的增量。

**来源 ftr-eskf 的 src/VisualSim 全部代码文件及关联引用见 §3 逐文件表；不能再用旧 main 分支的文件列表判断迁移完成。** 源仓库中的 ROS 相关组件只可参考。任何文件若有 ROS/ament/rclcpp/cv_bridge 或与 ROS 消息相关的调用，不允许未经解耦直接移到独立库。只允许把与独立仿真相关的新实现整合到**目标仓库**中。

## 2. 生效需求、设计合同与审计状态（引用而不复制）

- **唯一产品需求来源**：[SRS.md](SRS.md) 的 FR-M/FR-S/FR-G/FR-C、NFR-01..07 与 AC-01..12；其中 Ubuntu 24.04、Linux-only、无 ROS、只限 Room、仅用户修改历史、0..N 相机、严格递增时间等均为已批准的边界。
- **架构决策来源**：[HLD.md](HLD.md) 的历史解析器、两类模式与作用域、动态相机注册、门控生成、无计算结果缓存和风险权衡。
- **实现合同来源**：[LLD.md](LLD.md) 的 API v1、Result/错误、时间与边界、相机/IMU 模型、Brownian-bridge 噪声、YAML schema v1 与 ABI/生命周期。
- **审计依据与实际状态**：[AUDIT.md](AUDIT.md) 和 [AUDIT_PROGRESS.md](AUDIT_PROGRESS.md)，两文件当前在 docs/ 中可访问；审计/文档整改状态与代码开发、测试结果分别记录，不互相替代。**若 P0 所读取的版本缺少审计文件或链接不可用，暂以 SRS/HLD/LLD/本计划和两个仓库当时的 HEAD 为只读基线，并记录缺失，先恢复审计文件再宣称审计闭环。**

本计划**不重新定义**已批准的运动模式、坐标方向、校准公式或噪声算法；如发现设计冲突，应先在对应规范文件审计，再更新本计划任务。此前的文档审计对**尚未发行的软件 v1 设计**完善默认标定、名称冲突、布朗桥边界等遗漏，不代表任何现存二进制 ABI 已被破坏；后续若实际公开发行并发生破坏性接口变化，须依 LLD 升级 API/SONAME/schema 并更新四份文档。本项目是**单人开发**，不设置人员分工、日期排期、工时审批等多人项目管理要求。性能预算与性能验收已明确推迟到首次成功运行程序之后再讨论，不属于当前必须决定的事项。

## 3. 两仓库源码清点、同名差异及融合决策（以固定 commit 为准）

**清点范围和计数**：从两仓库 Git tree 递归列举。目标仓库与运行/测试直接相关的头/源文件 **4 个**（src/ 下 3 个、tests/ 下 1 个）。来源 ftr-eskf 的 src/VisualSim/ 下头/源文件 **15 个**，并继续追踪其构建依赖 src/CameraIO/ **5 个**、同一依赖闭包内 include/euroc_vio/ **13 个**、测试依赖 src/VisualSlam/ImageDataLoader.hpp **1 个**，合计 **34 个来源头/源**。两仓库本轮分类表一共 **38 个文件**；CMake 等构建文件单独说明。**不是**把 ros2-vio-node 所有 VIO/ROS 源码统统计作待迁移代码。

比对的固定依据为目标 `main@d4b38f2ea73fa5aef700b303517a45ec21281561` 与只读来源 `ftr-eskf@fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03`。后续开始实施时重新检查 HEAD/变更，但本表不可静默改成旧的 main 来源。下表“是：改写/吸收”表示将功能**在目标仓库重新实现**，不等于将来源文件原样复制过去；“否”表示不纳入 v1。

### 3.1 目标仓库：当前全部直接相关头/源/测试（4/4）

| 相对路径 | 当前实际职责 / 关键差异 | 未来目标仓库是否纳入 | 决策与目标位置 / 拒绝原因 |
| --- | --- | --- | --- |
| `src/Room.hpp` | 房间六面网格、默认尺寸固定；包含 Points() | 是：保留并改写 | 继续采用 visualsim::Room，融合可配尺寸/校验，添加六个有限表面真值 |
| `src/Trajectory.hpp` | ForwardConstantVelocity、OpticalAxisConstantRotation；double 秒、单条硬编码运动对象 | 是：保留运动公式，重构接口 | 提取基础公式至事件驱动 Linear/AngularMotion、MotionState，移除静默 clamp |
| `src/main.cpp` | CLI 硬编码两个场景、单目/IMU 数据导出，内含渲染、YAML、CSV 与 remove_all | 是：重写为薄 CLI | 转到显式配置文件/公共库调用；渲染/IMU/导出拆出；禁隐式删除目录 |
| `tests/TrajectoryTests.cpp` | 既有前向匀速、纯旋转与 Room 网格检查 | 是：保留并扩展 | 替换过时类调用，扩至事件历史、跨类组合、边界与导数测试 |

### 3.2 来源 ftr-eskf：src/VisualSim 全部头/源文件（15/15）

| 相对路径 | 当前实际职责 / 关键差异 | 未来目标仓库是否纳入 | 决策与目标位置 / 拒绝原因 |
| --- | --- | --- | --- |
| `src/VisualSim/Camera.hpp` | Sophus::SE3 相机、针孔 K、投影；外参字段是 T_SB（B→相机），不支持真实畸变 | 是：选择性改写 | 新 CameraCalibration/CameraRenderer；对 T_BC=C→B 必须求逆，使用冻结默认 K 与畸变 |
| `src/VisualSim/ErrorEvaluation.hpp` | 估计误差评估，依赖 DatumTruth，非仿真观测生成 | 否 | 不进入库；不引入估计器评价代码 |
| `src/VisualSim/EvoSim3.hpp` | 离线真值/轨迹加载工具，依赖 AbstractLoader、C++ reflection | 否 | 不进入按需运行库；轨迹来源为用户事件及 YAML |
| `src/VisualSim/ImuNoiseModel.cpp` | 独立 IMU CSV 噪声后处理 CLI，具有自身 main | 否 | 不并入核心或增加离线加噪可执行程序 |
| `src/VisualSim/ImuNoiseModel.hpp` | mt19937 序列推进、按固定 sample_rate 离散白噪与 bias 游走，持久 bias | 否（仅数学定义参考） | 必须重新实现 LLD §6.2 固定种子、随机时间索引、Brownian-bridge 无偏置历史缓存模型 |
| `src/VisualSim/LinearKalmanFilter.hpp` | OpenCV 2D 跟踪/滤波辅助算法 | 否 | 不是相机渲染或 IMU 物理仿真需求 |
| `src/VisualSim/MeshPlot.hpp` | Room 六面棋盘网格填色、固定双目、HighGUI namedWindow/destroyAllWindows | 是：仅渲染功能改写 | RoomRenderer 无窗口副作用、适配 0..N 相机与 CV_8UC1；不用原固定双目 GUI 类 |
| `src/VisualSim/Path.hpp` | Sophus::SE3 的 Stationary/Acceleration/Circle 和 PathManager 顺序轨迹（浮点秒） | 是：吸收解析运动公式 | 重写为带 TimestampNs 历史的两类即时式模式；不直接迁 PathManager 和朝向耦合模式 |
| `src/VisualSim/Preintegrate.hpp` | IMU 预积分，依赖 DatumImu；用于估计 | 否 | 不迁移 VIO 后端运算 |
| `src/VisualSim/Room.hpp` | 带 depth/width/height 构造参数、有限正数检查、GetIndex、六面网格；构造打印 stderr | 是：合并到目标 src/Room.hpp | 沿用目标命名空间与 Points()，融合动态尺寸/校验/GetIndex 所需部分；去除 stderr 副作用 |
| `src/VisualSim/StereoRig.hpp` | 固定左右两台相机，共同可见路标的索引交集 | 是：相机集合语义重写 | 替换为 0..N CameraRegistry；逐相机独立可见性，不做固定双目交集 |
| `src/VisualSim/StressModel.hpp` | 额外的像素异常点/噪声、脉冲平移、非零角加速度压力运动 | 否 | v1 无额外像素压力模式，角加速度脉冲不属于冻结的两种角模式 |
| `src/VisualSim/StressTests.cpp` | StressModel 的运动/像素噪声测试 | 否 | 不引入未批准的 StressModel；有效的通用数值断言另写运行库测试 |
| `src/VisualSim/VisualInertial.cpp` | rclcpp 节点、ESKF/预积分/数据估计 | 否 | 严格禁止 ROS/VIO/ESKF 代码进入独立仿真库 |
| `src/VisualSim/VisualSim.cpp` | 双目 CLI、Room 渲染、10:1 IMU、噪声/Stress、CameraFrameWriter；double→ns；写盘及 remove_all | 是：功能重构而非复制文件 | 迁离 main 的算法至 Simulator/Renderer/ImuSynth/Exporter；按请求时间门控 N 相机、无缓存与写盘副作用 |

### 3.3 来源 ftr-eskf：CameraIO 与引用闭包/测试辅助（19/19）

| 相对路径 | 当前实际职责 / 关键差异 | 未来目标仓库是否纳入 | 决策与目标位置 / 拒绝原因 |
| --- | --- | --- | --- |
| `src/CameraIO/CameraFrameReader.cpp` | 外部 CSV/PNG/FFV1 时间索引读取及逐帧解码 | 否 | v1 运行时不回放外部相机序列 |
| `src/CameraIO/CameraFrameWriter.cpp` | CSV/PNG 以及 FFV1 VideoWriter；时间戳用 int64 | 是：仅 PNG/CSV 思路重写 | 改写到目标 DatasetExporter，输出显式请求的 PNG/CSV，不携 FFV1 动态链接 |
| `src/CameraIO/FFV1Video.hpp` | FFV1 编解码器接口 | 否 | 视频不是 v1 验收范围；现有 FFV1 要 CV_8UC3 |
| `src/CameraIO/FFV1Video.cpp` | libavcodec/libavformat/libavutil FFV1 Matroska 编解码实现 | 否 | 无 FFmpeg 强制依赖，后续如需视频另设扩展合同 |
| `src/CameraIO/CameraFrameIOTests.cpp` | PNG/CSV/FFV1 导出读取回归，关联 VisualSlam ImageDataLoader | 是：抽取 PNG/CSV 测试思想 | 写新的目标 ExportTests；不复制依赖 FFmpeg/VisualSlam 的全套测试 |
| `include/euroc_vio/CameraFrameIO.hpp` | CameraFrameWriter/Reader、CameraOutputMode 视频和图片接口 | 是：仅输出接口理念改写 | 新目标 DatasetExporter 公共/内部接口；不照搬 FastVIO 命名与 FFV1 选项 |
| `include/euroc_vio/AbstractLoader.hpp` | CSV/真值加载基类，被排除的 EvoSim3/Datum 系列引用 | 否 | 不迁入数据回放/估计器加载器 |
| `include/euroc_vio/DatumFast.hpp` | 估计器/加载器的 Fast 状态字段 | 否 | 与传感器生成无关 |
| `include/euroc_vio/DatumImu.hpp` | IMU 数据集读取/预积分数据结构 | 否 | 运行库自定义 ImuMeasurement，不依赖载入器 |
| `include/euroc_vio/DatumTruth.hpp` | 历史轨迹/估计误差读取 | 否 | 真值来自事件解析，无需外部 TruthLoader |
| `include/euroc_vio/ErrorStateKalmanFilter.hpp` | VIO ESKF 状态与更新 | 否 | 非仿真生产范围且引入估计器内部依赖 |
| `include/euroc_vio/SensorState.hpp` | 估计器状态数组类型 | 否 | 不替代 MotionState/传感器标定合同 |
| `include/euroc_vio/SensorYaml.hpp` | EuRoC 相机/IMU YAML 校准读取、字段有效性校验 | 是：仅字段校验/导出语义改写 | 目标 YAML baseline→events schema v1 与按时刻标定，不能直接调用现有 SensorYaml |
| `include/euroc_vio/ZUPT.hpp` | 估计器零速度检测/缓冲 | 否 | 不需要 ZUPT 或历史估计缓存 |
| `include/euroc_vio/StereoObservation.hpp` | ESKF 用双目观测结构（ErrorStateKalmanFilter 的依赖） | 否 | 不强制双目或引入 ESKF |
| `include/euroc_vio/CircularBuffer.hpp` | ZUPT 环形缓存容器 | 否 | 用户禁止历史帧/位姿/观测结果缓存，且无 ZUPT |
| `include/euroc_vio/VisionMode.hpp` | VisualSlam ImageDataLoader 测试路径的模式类型 | 否 | 源测试间接依赖，非 v1 运行库类型 |
| `include/euroc_vio/pch.hpp` | 来源根 CMake common_pch 预编译头，含 rclcpp/cv_bridge/ROS 消息 | 否：严格禁止 | 任何 ROS 头、ament/common_pch 不可进入目标项目 |
| `src/VisualSlam/ImageDataLoader.hpp` | CameraFrameIOTests 的外部图像读取测试依赖 | 否 | 不移植 VisualSlam 读取器/其他估计器部件 |

**关联范围说明**：CameraIO 的 5 个文件为 CameraFrameReader.cpp、CameraFrameWriter.cpp、FFV1Video.hpp、FFV1Video.cpp、CameraFrameIOTests.cpp；include/euroc_vio 的 13 个文件是对 src/VisualSim 和相关 CameraIO/测试的直接、构建传递、必要二级本地 include 的查证记录。其余 ROS/VIO 业务目录并非 VisualSim 的功能候选，不在迁移表中逐一枚举。来源根目录 src/main.cpp 仅在下方“同名”误判防护中列为排除项，不并入上述 34 个相关文件计数。

### 3.4 同名文件的**实际比较**及语义对应文件的方案

| 对照关系 | 实际文件与哈希 | 代码差异 | 决策 |
| --- | --- | --- | --- |
| 同名 Room.hpp（必须逐文件比较） | 目标 src/Room.hpp (blob b6dfb2fc) ↔ 来源 src/VisualSim/Room.hpp (blob e7f63817) | 同为六面离散网格、去重；**目标固定 10×10×3m** 且有 Points()，来源构造可指定深/宽/高并校验有限正数，有 GetIndex()、stderr 打印；两者命名空间不同 | 保留目标文件/namespace，融合来源构造参数及校验与所需索引；RoomConfig 在每个查询时间生效；新增六面真实交点算法，**不能直接覆盖目标 Room.hpp** |
| 同职责但不同名的轨迹 | 目标 src/Trajectory.hpp (252c7a5e) ↔ 来源 src/VisualSim/Path.hpp (3b5035bf) | 目标只含前向恒速、纯旋转及 clampTime；来源多静止/加速/圆周及顺序 PathManager，Sophus pose、浮点秒；均不具两类独立栈/事件历史/六导数完整合同 | 保留适用解析式，**重写双类别时间事件+BeginMotionMode/EndMotionEnd**；不把旧 PathManager、朝向耦合圆周模型直接作为最终 API |
| 同职责但不同名的生成器 | 目标 src/main.cpp (53f58afd) ↔ 来源 src/VisualSim/VisualSim.cpp (d8f0aac7) | 目标单目 PNG/CSV、硬编码两场景；来源固定双目、两目交集、CV_8UC3/FFV1、Stress/IMU；双方都有隐式写盘、固定频率和浮点秒生成 ns | 拆分共享库和薄 CLI，统一 Room、0..N 相机、CV_8UC1、整型纳秒门控；不照搬 main 或 remove_all |
| 同名 CMakeLists（构建文件非源码） | 目标根 CMakeLists.txt (be122ceb) ↔ 来源根 CMakeLists.txt (fae40d06) 与 src/VisualSim/CMakeLists.txt (2c96b178) | 目标 C++20 可执行程序并允许 Windows/MSVC；来源根经 ament/ROS/common_pch 串依赖，来源子目录 standalone 使用 cxx_std_23、camera_frame_io/FFmpeg 与 Windows 选项；均非目标 .so 安装方案 | 只重写目标根 CMake，Linux-only SHARED + install/export/consumer；不复制两份来源 CMake；不强制降低用户现有 GCC/CMake |
| 同名 main.cpp 但**不同项目职责** | 目标 src/main.cpp ↔ 来源根 src/main.cpp (2f21f1e1) | 来源根 src/main.cpp 是 rclcpp/cv_bridge 与同步器的 ROS 主程序，不是 VisualSim 生成入口 | 来源根 src/main.cpp **排除**，不能因为同名覆盖目标；只把 src/VisualSim/VisualSim.cpp 视为功能对应入口 |

### 3.5 必须遵守的融合约束（不能以“复制后能编译”为验收）

1. **姿态/外参方向**：来源 Camera.hpp 的 sensor_from_body_ 是 T_SB（B→传感器），LLD 公共 T_BC、T_BS 是传感器→载体；复制时如不求逆将导致错投影、错求交/IMU。以 LLD §5/§6 公式为最终准则，并做非单位姿态/非零杆臂验收。
2. **动态相机和灰度**：来源 StereoRig 两台相机共享可见点交集、VisualSim.cpp 的 CV_8UC3/FFV1 管线与冻结的“任意 0..N 相机，各自独立可见性、CV_8UC1”不一致；必须重写注册/渲染，不导入固定双目类型。
3. **噪声**：来源 ImuNoiseModel.hpp 使用 mt19937、持久 bias 和固定采样频率，且默认噪声非零；与 LLD §6.2 按时间键的无缓存 Brownian-bridge、默认关闭噪声及固定 v1 测试流不兼容。只能参考 N/√Δt 和 K√Δt 物理统计关系，**不得直接复制噪声状态机**；来源 ImuNoiseModel.cpp 是独立离线 CSV 加噪程序，排除。
4. **Stress 模型**：来源 StressModel 提供单独的像素 outliers 和脉冲式**非零角加速度**，不属于当前冻结的两种角运动模式；StressTests 仅测试该模型。因此两文件均排除，不能隐式扩充 SRS。
5. **CameraIO/FFmpeg**：来源 camera_frame_io 是以 FFV1/libavcodec/libavformat/libavutil 为必需编译依赖的静态库；FFV1Encoder::Write 要求 CV_8UC3，与 v1 CV_8UC1 不一致。v1 仅保留显式 PNG/CSV 导出语义，不引入 libav*、FFV1 视频、CameraFrameReader 或 VisualSlam。若未来要求无损视频，应另行设计与改验收，不把 FFmpeg 作为本轮强制依赖。
6. **Room 真值**：两个 Room.hpp 都只生成离散网格点而非连续六面真实求交；必须在目标实现按指定时刻 RoomConfig 解析的有限六面最近射线交点，不能返回网格顶点充当任意 bearing 的真值；不保留帧/位姿缓存。
7. **时间与副作用**：目标/来源旧生成器均按 double 秒迭代并转纳秒，且与 CLI 写盘相耦合；必须按 TimestampNs 原样传递并以事件历史分段重算。拒绝同一传感器重复/回退时间；采样和 Raycast 不自动 remove_all、写 CSV、输出 GUI 或修改来源仓库。
8. **构建依赖与编译特性**：来源根 CMake 的 ament/common_pch 含 ROS；来源独立子目录仍链接 camera_frame_io/FFmpeg。两者均不能作为目标 SHARED 构建配方照抄；允许 Eigen/Sophus/OpenCV/yaml-cpp 按实际使用引入，遵守 Linux-only 与已有 GCC/CMake，不做强制降级。

### 3.6 构建配置/脚本清单（非头/源文件）

| 仓库 | 路径 | 未来处理 |
| --- | --- | --- |
| 目标 | `CMakeLists.txt` | **改写**为 Linux-only、可安装 SHARED 库、配置化 CLI、单元测试及独立消费者；不降低 CMake/GCC |
| 目标 | `CMakePresets.json` | **重写/移除 Windows preset**，按需保留 Linux preset |
| 目标 | `build.ps1`, `run.ps1`, `clean.ps1` | **删除/禁用** Windows 专用脚本 |
| 目标 | `vcpkg.json` | 按 Linux 依赖安装策略决定是否保留 manifest；**不得维持有效 Windows 构建路径** |
| 目标 | `README.md` | 更新编译、安装、传感器 API、配置示例、数据导出和 Linux-only 说明 |
| 来源只读 | `CMakeLists.txt` | ROS/ament/common_pch 主工程，**不迁入** |
| 来源只读 | `src/VisualSim/CMakeLists.txt` | standalone C++23 但带 CameraIO/FFmpeg 和 Windows 兼容分支，**不迁入** |
| 来源只读 | `src/CameraIO/CMakeLists.txt` | 静态 camera_frame_io 与 libav 必需依赖，**不迁入** |

**合规范围**：只修改目标仓库；以上文件的未来处置是**实施计划**，并非本轮已经修改源码。根目录构建支持脚本不算入上述 38 个头/源清点数。

### 3.7 冻结 v1 功能与**实际代码**的一致性结论

下表区分“设计已冻结”和“代码已经实现”。这些差距从固定快照的源码得到，并非运行测试结论；不得把文档完成写成实现通过。

| 冻结需求 / 计划目标 | 目标 main 的现状 | 来源 ftr-eskf 的现状 | P0 后的实施决定 |
| --- | --- | --- | --- |
| 纯 Linux 可安装 SHARED .so、无 ROS | 只定义 visual_sim 可执行程序，有 Windows/MSVC 分支 | ROS 根工程 ament/common_pch；standalone 仍生成可执行程序且联 FFmpeg | P1 在目标新建 SHARED/install/export/独立消费者，CMake 禁 Windows，排除 ROS/FFmpeg 强制依赖 |
| BeginMotionMode/EndMotionEnd、两类模式历史、解析六项真值 | 仅 Trajectory 对象的固定轨迹/三角参数，未提供事件 API | PathManager 按浮点秒顺序拼接，角运动与圆周朝向耦合 | P2 新建独立 Linear/Angular 当前模式、作用域事件历史及按时刻重算，不复制旧管理器 |
| 任意 N 相机、自动 camX/改名、历史标定真实生效 | 只有 cam0，标定在 main 内写死 | Camera/StereoRig 固定 cam0/cam1，T_SB 方向，只有针孔投影 | P3 实现 ID 注册/重命名、T_BC 方向、实际内参与畸变，逐相机独立渲染 |
| IMU 外参、比力杆臂、默认无噪及无缓存 Brownian-bridge | imu CSV 仅理想比力/固定传感器假设，noise YAML 常量非零但未叠加 | ImuNoiseModel 用逐次 mt19937+内部 bias/固定步长，不满足任意时刻无缓存 | P3 重新实现 IMU 数值模型及 LLD §6.2 随机过程，不迁移旧状态机 |
| Room 唯一场景与连续真实求交 | Room 网格点已存在，但 main 另有前向三深度离散路标；无 Raycast | Room 六面网格，有可指定尺寸，但无任意 bearing 最近有限面求交 | P4 只保留 Room 六面真实连续几何，场景历史版本生效并实现 Raycast |
| 传感器门控、无重复时间、int64 纳秒原样输入/输出、无计算结果缓存 | for 循环按 double 秒采样转纳秒，缺 Generate/事件历史 | 同样 double 秒步进并带固定左右目/IMU 频率比；没有 v1 原子事务 | P5 实现 Generate/FrameSequencer 和历史求值；最小 last_successful_ns/帧号，拒绝重复与回退 |
| 配置文件驱动 CLI、保留 PNG/CSV、无隐式写盘 | 两条内置场景、启动自动输出且会 remove_all | CLI 含 FFV1、Stress 和隐式输出，摄像机帧通过 CameraIO | P6 薄 CLI+配置输入；仅显式导出；保持 CV_8UC1、PNG/CSV，不删除目录 |

**判定**：现行计划的产品目标与 SRS/HLD/LLD 相符，但其原 §1 来源分支、原 §3 源码迁移清单与真实 ftr-eskf **不一致**；本轮修订后二者在**文件范围、迁移/排除决定和工具链来源**上具备静态对齐的执行依据。**并不意味着目标代码已经满足任何新 v1 API 或通过 P0–P6 实施验收**。

## 4. 实施阶段

| 阶段 | 任务 | 可核验交付 |
| --- | --- | --- |
| P0 — 差异审查 | **显式核验 docs/AUDIT.md、docs/AUDIT_PROGRESS.md 存在且可读，并读取其当前状态**；以本轮 §3 对照的目标 main / 只读来源 **ftr-eskf** 固定快照为起点，实际实施时重新查看两仓库 HEAD；复核**已冻结的 LLD v1 event API 合同**与实际源码的差异、外参坐标方向、依赖和 Room 边界；若需破坏性变更，先提出并明确升级 API/SONAME/schema 版本且同步 SRS/HLD/LLD/PLAN，不重复执行“冻结” | 实际迁移差异和版本记录，仅在目标仓库；保持来源仓库只读 |
| P1 — Linux 独立库 | 共享库、公共头文件、CMake install/export、消费端示例、无 ROS；禁用 Windows，保留用户现有工具链 | .so 可按前缀安装，外部工程独立构建调用 |
| P2 — 运动历史 | 两类互斥模式、BeginMotionMode/EndMotionEnd、同类覆盖、跨类组合；时间事件记录、段解析、物理量与作用域恢复；函数/YAML 同构 | 每个时间的 p/R/v/a/ω/α 正确，无运动中间结果缓存 |
| P3 — 传感器历史 | 自动 camX 创建及改名、0..N 相机、默认 cam0、历史 K/T_BC/畸变与 imu0/T_BS/噪声，均实际影响输出 | 各相机个性化成像与 IMU 真值/噪声按历史修改生效 |
| P4 — Room + 求交 | 唯一 Room，取消 ForwardDepthScene；同源渲染/表面求交；任意时间按历史重算位姿，不依赖生成图像 | 任意时间可查询每条视线的最近真实 Room 交点 |
| P5 — 门控与无帧缓存 | camera/imu 门控、动态相机集合、各传感器编号/严格递增时间、失败事务性；不缓存采样结果 | 多相机帧及 IMU 帧可在同时间独立生成；重复拒绝 |
| P6 — CLI/兼容与正式验收 | 配置化替代现有 main.cpp 两条硬编码运动；Room 内前进/旋转示例；PNG/CSV/YAML 输出、错误处理和安装验收 | AC-01–AC-10、AC-12 对应的 V01–V11、V13 功能和安装检查；AC-11/V12 性能验收延期，不影响当前阶段完成；来源仓库只读 |

依赖顺序可以在同阶段内交叉实现，但**验收不得跳过** “无帧/位姿缓存、事件历史可回放、多相机/Room only” 的硬性要求。不得仅通过复制来源的 VisualSim.cpp 或 StereoRig.hpp 而宣称迁移完成。

## 5. 文档一致性与禁止事项核对

工程实施过程中，每次涉及公开 API 的修改，必须同时核对 SRS（外部行为）、HLD（组件合同）、LLD（公式与错误条件）和本计划（迁移实施）。不可在 SRS 中加入迁移文件表或源代码实施步骤。

禁止事项：

- 修改 ncut-d1z/ros2-vio-node 仓库；
- 重新引入 ROS 运行时、ament、cv_bridge 或 ROS 消息依赖；
- 重新允许 Windows 编译，或要求为 Ubuntu 24.04 强制降低构建工具链；
- 把固定的两条轨迹硬编码为唯一的轨迹来源；
- 允许类内模式并行叠加，或者将 EndMotionEnd 改名而不兼容用户指定 API；
- 为了求交再引入最近 64 组图像位姿/帧/交点缓存；
- 固定只生成双目、默认创建两台相机或改名后覆盖其他相机；
- 引入 ForwardDepthScene；
- 重复传感器时间生成新帧、新编号；
- setter 只改元数据、不改变真实观测；
- 在采样/查询时未经明确指令写盘、删除文件。

## 6. V 测试与验收追踪（仅规划，未执行）

以下测试用例从 LLD 移入本**实施与验收计划**。V06 分为 V06-D（确定性逐元素）与 V06-S（冻结统计检验），其余编号保持不变。测试命令与输出证据将在后续实际运行后登记；除延期的 V12 外，其他用例均为“未执行”。

| V 用例 | 验收检查 | 对应 AC | 状态 |
| --- | --- | --- | --- |
| V01 | Ubuntu 安装 .so、无 ROS 独立消费者；Windows 配置失败；工具链不强制降级 | AC-01 | 未执行 |
| V02 | 相同函数调用与 YAML baseline→events 序列等价；同时间事件排序稳定；未给相机标定时 YAML 和 API 均读取 LLD §5.1 默认 K/尺寸/畸变/20 Hz | AC-02 | 未执行 |
| V03 | 同类覆盖、跨类组合、嵌套 BeginMotionMode/EndMotionEnd，出栈不使 p/R 意外跳变 | AC-03 | 未执行 |
| V04 | 静止、恒速、恒加速、圆周、纯旋转、多段拼接 p/R/v/a/ω/α 与解析公式一致 | AC-04 | 未执行 |
| V05 | 核验默认 cam0 及仅给 T_BC 的 CreateCamera 可直接按 K=(458.654,457.296,367.215,248.375)、752×480、零畸变、20 Hz、CV_8UC1 成像；默认 cam0 在已 SetCameraCalibration 或 Rename 后的首次 CreateCamera 不得覆盖它；改名 cam0→cam1 后新建相机自动跳至 cam2、SensorId/帧号不变 | AC-05 | 未执行 |
| V06-D | **确定性检查**：不同相机 K/T_BC/畸变、IMU T_BS/杆臂确实改变观测；g_W 固定为 (0,0,-9.81) m/s²、无可变重力接口；禁噪理想值、启噪复现及固定 seed 逐元素重放；布朗桥 t_epoch 的 W(0)=0、INT64_MAX 边界相对纳秒不溢出 | AC-06 | 未执行 |
| V06-S | **统计检查**：隔离 gyro/accel white 与 gyro/accel walk 四组噪声，分别执行双侧 99% 卡方检验；**独立种子、采样间隔、样本量、自由度和阈值均以 LLD §6.2 冻结表为唯一准据**，此处不重复数值以免漂移 | AC-06 | 未执行 |
| V07 | 任意 N 相机门控、逐传感器重复/回退拒绝；整批失败时序号不变 | AC-07 | 未执行 |
| V08 | 未生成图像的任意有效时刻，Room 六个有限面真实最近交点及视域失败状态；空 bearings_C+有效时间/相机须 Ok 空列表，空 bearings_C+无效时间/相机仍返回整体错误 | AC-08 | 未执行 |
| V09 | 事件改变可重算历史，源代码审查与内存量测证明不保留按帧/位姿/IMU/交点结果 | AC-09 | 未执行 |
| V10 | 纳秒类型 static_assert，配置/调用运行旧运动行为，唯一 Room；cv::Mat 为 CV_8UC1 且与有效标定尺寸一致，数据所有权正确、输出可跨下一帧与模拟器析构存活 | AC-10 | 未执行 |
| V11 | 故障注入（OpenCV 渲染失败、YAML 配置解析失败、结果向量分配失败/可捕获异常）及成功返回路径检查；错误后下一合法帧仍使用原帧号、历史/随机过程不变；提交点后的结果返回不得抛出并留下半提交 | AC-07、AC-09 | 未执行 |
| V12 | 性能预算及性能验收：首次成功运行程序并得到有效传感器数据后，再视实测结果决定是否执行性能基准与提出验收指标 | AC-11 | **延期，非 P0–P6 功能验收门槛** |
| V13 | 多实例互不共享可变状态；单实例并发遵守公开合同；只读并发与析构交错规则正确 | AC-12 | 未执行 |

## 7. 单人实施步骤、依赖取证与本地检查

项目由一人实施。**不设置负责人、分工、排期、工时预算或审批流程。** P0–P6 仅描述代码依赖与验收顺序，不构成时间承诺。

### 7.1 阶段依赖与功能完成条件

| 阶段 | 前置条件 | 功能完成证据 |
| --- | --- | --- |
| P0 | 设计文档和两仓库实际代码可读取 | HEAD 差异、工具链、实际依赖版本与坐标约定记录完成 |
| P1 | P0 | V01 Linux 共享库构建、安装及独立消费者通过 |
| P2 | P1 | V02/V03/V04 运动解析和事件历史通过 |
| P3 | P2 | V05/V06-D/V06-S 多相机、IMU 标定与噪声正确性通过 |
| P4 | P3 | V08 Room 场景和任意时刻射线交点通过 |
| P5 | P4 | V07/V09/V11 门控、无缓存与原子性通过 |
| P6 | P5 | V01–V11、V13 的功能与安装检查通过，**不等待延期的 V12 性能验收** |

### 7.2 工具链与依赖版本取证

| 项目 | 已知事实 | 首次构建时记录 |
| --- | --- | --- |
| 系统 | Ubuntu 24.04 | 系统版本与架构 |
| CMake / 编译器 / 标准库 | 用户现有可用工具链，无需降级 | cmake --version、c++ --version、CXX_STANDARD、libstdc++ ABI |
| Eigen / OpenCV / Sophus | 可使用的非 ROS 库 | 实际链接组件、find_package 版本及安装位置 |
| yaml-cpp | 可用于 YAML v1 配置 | 如采用则记录版本及严格字段校验 |
| 构建结果 | 尚未进行 | git HEAD、编译选项、依赖树、符号与 SONAME |

版本号是从运行环境采集的**复现记录**，不需要人为预设审批。发现 ABI 变化时，依设计更新主版本并复验独立消费者。



**用户已经提供的 Ubuntu 24.04 编译工具链事实（非本轮实机验证）**：

| 命令 | 用户提供的输出 | 状态与用途 |
| --- | --- | --- |
| `cmake --version` | `cmake version 4.3.4` | 已知当前环境版本；P0 核对时保留，不提出版本降级 |
| `c++ --version` | `c++ (GCC) 16.2.0` | 已知当前环境版本；P0 复核实际 C++ 特性/链接和 ABI |
| 目标 OS | Ubuntu 24.04 | 仅面向 Linux，不扩展 Windows 构建支持 |

上述文字是**用户终端输出**的记录，非本轮执行命令的结果，也不代表 CTest 已通过。目标仓库目前 CMakeLists 设 C++20；来源 standalone 子目录要求 cxx_std_23，来源 ROS 主 CMake 有不同标准/反射要求；v1 实现须根据真正使用的语言特性选择标准和编译参数，**不强制降低现有的 GCC 16.2.0 或 CMake 4.3.4**。若改动 C++ 标准需在 P1 记录理由与实际验收结果。
### 7.3 本地验证范围

| 环境 | 检查内容 |
| --- | --- |
| Ubuntu 24.04 / Release | 构建、CTest、独立安装和消费端、无 ROS 依赖、V01–V11 与 V13 |
| Ubuntu 24.04 / Debug | 输入校验、异常回滚、断言，支持时的 Sanitizer |
| Windows（仅配置负例） | 核查 CMake 明确拒绝 Windows；不编译 Windows 可执行文件 |

本项目默认使用本地构建和测试，**不要求 CI 工作流或多人审核机制**。V12 的性能测量与验收延期到首次成功运行程序之后另行决定。

## 8. FR/NFR/AC/LLD/V/P 追踪矩阵

本表是**完整的单一追踪主表**；SRS §10 只维护需求到 AC 的概览。LLD §10 引用本表而不复制测试计划。

| 需求 ID | 验收 AC | LLD 对应接口或算法 | V 测试 | 实施阶段 |
| --- | --- | --- | --- | --- |
| FR-M01 | AC-03 | §2 运动模式类型，§3 | V03、V04 | P2 |
| FR-M02 | AC-03 | §2 BeginMotionMode/EndMotionEnd、§3 | V03 | P2 |
| FR-M03 | AC-02、AC-04 | §3.3 事件顺序、§4 Evaluate | V02、V04 | P2 |
| FR-M04 | AC-04 | §2 MotionState、§4 解析式 | V04 | P2 |
| FR-S01 | AC-05 | §2 CreateCamera/RenameCamera、§5 | V05 | P3 |
| FR-S02 | AC-06 | §5 投影、§6 IMU 与噪声 | V06-D、V06-S | P3 |
| FR-S03 | AC-07 | §2 Generate、§8 | V07、V11 | P5 |
| FR-G01 | AC-08 | §7 Room 几何 | V08 | P4 |
| FR-G02 | AC-08 | §2 Raycast、§7 | V08 | P4 |
| FR-C01 | AC-09 | §3 事件、§4 状态重放 | V09 | P2 |
| FR-C02 | AC-09 | §1 无历史结果缓存、§8 最小序号 | V09 | P2、P5 |
| NFR-01 | AC-09 | §1 存储结构 | V09 | P2、P5 |
| NFR-02 | AC-11 | §4 事件重放、§7 查询、§8 Generate | V12（延期） | 首次有效运行后再决定；非 P0–P6 门槛 |
| NFR-03 | AC-12 | §2.2 并发/隔离 | V13 | P1、P6 |
| NFR-04 | AC-07 | §2.1 错误、§8 原子性 | V07、V11 | P5 |
| NFR-05 | AC-01、AC-10 | §2.1 状态码、§9 输出 | V01、V10、V11 | P1、P6 |
| NFR-06 | AC-01 | §2.3 ABI、§9.2 安装 | V01 | P1、P6 |
| NFR-07 | AC-02、AC-04、AC-06、AC-08 | §4 解析、§5/§6 观测、§7 交点 | V02、V04、V06-D、V06-S、V08 | P2–P6 |

检查要求：每个 FR/NFR 至少有 AC、LLD 和 V/P 入口；每个 AC-01..AC-12 都有 V；V01–V13 的序列中 V06 由 V06-D 与 V06-S 两条实测用例共同覆盖。若增加需求或测试必须同步更新此表和 SRS §10。

## 9. 可执行验收命令与检查单（**后续实施才执行**）

以下是后续 Ubuntu 24.04 验收的命令模板。库尚未实现；命令不构成已通过证据。

~~~bash
# P0 对照用户已提供的 CMake 4.3.4 / GCC 16.2.0，再实测并登记差异（不写来源仓库）
# 先确认审计规范及进度记录存在且可读；不可跳过后继续宣称文档审计闭环
test -r docs/AUDIT.md && test -r docs/AUDIT_PROGRESS.md || { echo "Missing or unreadable docs audit files" >&2; exit 1; }
git rev-parse HEAD
cmake --version
c++ --version

# P1 / P6：按照实际 CMake 实现构建并安装到指定前缀
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/../visualsim-install" -DBUILD_TESTING=ON
cmake --build build -j
ctest --test-dir build -N # 首先列出真正注册的测试名/数量（验收记录）
ctest --test-dir build --output-on-failure # 再执行实际注册的 V01–V11/V13 功能测试
cmake --install build

# 验证安装在源码树外可消费（路径应在 P1 实现后存在）
cmake -S tests/InstallConsumer -B build-consumer \
  -DCMAKE_PREFIX_PATH="$PWD/../visualsim-install"
cmake --build build-consumer -j

# 二进制依赖检查：以下命令中的库路径必须由实际安装产物确定
find ../visualsim-install -type f -name '*.so*' -print
# 在确定主库路径后，扫描 ELF 的直接 NEEDED 和递归运行时依赖
LIB="$(find ../visualsim-install -type f -name 'libvisualsim.so*' | head -n 1)"
test -n "$LIB" && test -f "$LIB" || { echo "libvisualsim.so not found under install prefix" >&2; exit 1; }
ldd "$LIB" | tee build-visualsim-ldd.txt
readelf -d "$LIB" | tee build-visualsim-readelf.txt
if grep -Eiq 'rclcpp|rosidl|ament|cv_bridge|sensor_msgs|geometry_msgs|nav_msgs|libros' \
    build-visualsim-ldd.txt build-visualsim-readelf.txt; then
  echo "Forbidden ROS dependency detected" >&2
  exit 1
fi
# 若消费者本身也链接了其他库，对消费者可执行文件再运行同样的 ldd/readelf 核对
~~~

Windows 的**配置负例**须在 Windows 主机上执行（仅验证拒绝，**绝不编译 Windows 程序**）：

~~~powershell
# 预期：CMake 在配置阶段返回非零，并提示 VisualSim supports Linux only
cmake -S . -B out/windows-rejected -DCMAKE_BUILD_TYPE=Release
if ($LASTEXITCODE -eq 0) { throw "FAILED: Windows configuration was accepted" }
~~~

Linux 构建日志应包含 ctest -N 的测试清单；根据实际注册的测试名称核查 V01–V11 和 V13 的覆盖情况，不能编造未实现的 CTest 名称或将“已注册”视为“已通过”。

**操作检查单**：**V01 由上方 CMake 构建、安装、独立消费者构建、ldd/readelf 与 ROS 依赖扫描，以及 Windows 配置拒绝负例共同覆盖**，不是被遗漏的测试项；核查链接目标/运行库不含 ROS/ament，Windows 仅失败配置，依赖版本已登记；当前阶段应执行 V02–V11、V13 等功能用例，**不执行已延期的 V12 性能验收**；采样失败无部分状态提交；多次 Raycast 不产生结果缓存；数据导出需显式目标路径且不会递归删除用户目录；来源仓库未被写入。若命令失败，记录退出码与日志，回退至上一通过阶段。

## 10. 风险登记与回滚

| 风险编号 | 风险 | 影响与触发 | 缓解措施 | 状态 |
| --- | --- | --- | --- | --- |
| R01 | 禁止历史位姿缓存造成事件重放慢 | 首次有效运行后发现事件重算耗时较高 | 记录实际数据；性能预算与验收延期，不引入结果缓存 | 延期观察 |
| R02 | 无缓存偏置随机游走造成大量随机访问成本 | 长时间域/多次查询耗时高 | 用 LLD §6.2 确定性 dyadic Brownian-bridge，测量极端时域 | 开放 |
| R03 | C++ Eigen/OpenCV ABI 不兼容 | 安装后消费者链接/加载失败 | P0 冻结工具链/依赖，P1 安装消费端，破坏 ABI 升 SONAME | 开放 |
| R04 | 修改过去配置改变已输出帧含义 | 对既有时间事件插入/改写 | 采样后冻结已输出时间的历史，事件全事务提交 | 开放 |
| R05 | YAML schema 更改导致重放不等价 | 配置加载语义冲突 | v1 版本校验、严格字段、回归 V02；新增字段需版本迁移 | 开放 |
| R06 | 外参方向、Room 渲染与射线求交不一致 | V06/V08 几何不符 | 统一 T_BC、T_BS 与 Room 真值基准，解析投影/交点校验 | 开放 |
| R07 | Windows/ROS 代码残留 | 负例配置或依赖扫描失败 | P1 移除入口、Linux guard、无 ROS 模块边界 | 开放 |

**回滚约定：**每一阶段在目标仓库提交并标记验收状态，失败时通过单独的修复提交或 git revert 有问题的目标仓库提交恢复先前稳定阶段；**不强推 main，不修改来源仓库**。每次回滚后复核文档追踪、重新验收受影响范围。文档审计阶段若发现内容相互矛盾，同样采用可审计的文档修订提交而非直接改写来源。

## 11. 性能验收延期与单人项目状态

| 事项 | 当前状态 | 后续处理 |
| --- | --- | --- |
| P95/P99 延迟、峰值内存预算 | **已明确延期，当前不需要决策** | 首次成功运行程序并生成有效数据之后，再根据实测情况考虑是否制定预算 |
| V12 正式性能验收 | **延期，不执行；不阻断 P0–P6** | 首次有效运行之后，再决定是否启动正式性能验收 |
| 依赖版本及 ABI 环境取证 | 常规工程取证，不需用户批准具体版本 | P0 首次实际构建时记录 |
| 本地构建、单元与功能测试 | 正常技术工作，不需要额外协作审批 | 各阶段具备可运行源码时按功能验收执行 |
| 人员分工及时间管理 | 单人项目不适用，不建立这些流程 | 无 |

**当前不存在必须在开始代码实现前由用户额外决定的事项。** 如果未来出现新的、现有 SRS/HLD/LLD 无法覆盖且确实影响功能范围的冲突，再明确列出具体选择，不把日常技术判断升级为人工审批。

本轮仅修改文档；尚未实施生产代码、编译、运行程序或性能验收。
