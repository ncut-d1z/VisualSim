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

| 仓库 | 分支 | 路径 | 未来处理 |
| --- | --- | --- | --- |
| VisualSim | main | `CMakeLists.txt` | **改写**为 Linux-only、可安装 SHARED 库、配置化 CLI、单元测试及独立消费者；不降低 CMake/GCC |
| VisualSim | main | `CMakePresets.json` | **重写/移除 Windows preset**，按需保留 Linux preset |
| VisualSim | main | `build.ps1`, `run.ps1`, `clean.ps1` | **删除/禁用** Windows 专用脚本 |
| VisualSim | main | `vcpkg.json` | 按 Linux 依赖安装策略决定是否保留 manifest；**不得维持有效 Windows 构建路径** |
| VisualSim | main | `README.md` | 更新编译、安装、传感器 API、配置示例、数据导出和 Linux-only 说明 |
| ros2-vio-node | ftr-eskf | `CMakeLists.txt` | ROS/ament/common_pch 主工程，**不迁入** |
| ros2-vio-node | ftr-eskf | `src/VisualSim/CMakeLists.txt` | standalone C++23 但带 CameraIO/FFmpeg 和 Windows 兼容分支，**不迁入** |
| ros2-vio-node | ftr-eskf | `src/CameraIO/CMakeLists.txt` | 静态 camera_frame_io 与 libav 必需依赖，**不迁入** |

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

### 3.8 逐文件实施细则（与 §3.1–§3.3 原表一一对应）

本节是 **P0–P6 将要执行的修改要求**，不是已实现的生产代码。F01–F38 按原清单次序对应：目标 4、来源 VisualSim 15、相关 CameraIO 和引用依赖 19。标明的建议新文件路径只是工程组织提案，不改变 LLD v1 已冻结公共 API。**排除项的约束是不得进入目标编译、公开头、链接依赖闭包，不是仅不复制同名文件。**

#### 3.8.1 VisualSim/main 现存头/源/测试 F01–F04

##### F01 VisualSim/main/src/Room.hpp

- **具体处理**：保留 visualsim::Room、六面网格排序去重、Points()；吸收来源动态 depth/width/height 与有限正数、网格段数校验；保留目标接口并扩展按时刻 RoomConfig。
- **目标或排除**：在 Room 几何服务中增加六个有限连续面、面 ID、有效边界；渲染棋盘网格与 Raycast 使用同一几何，不以网格顶点替代真值、不跨调用缓存求交。
- **验证**：目标仍为 src/Room.hpp，按需要分出内部 src/Room.cpp；V08 检验非默认尺寸、六面及边角交点。

##### F02 VisualSim/main/src/Trajectory.hpp

- **具体处理**：保留已有 ForwardConstantVelocity 的解析位移和 OpticalAxisConstantRotation 的 SO(3) 指数规律，替换唯一轨迹对象与静默 clampTime 行为。
- **目标或排除**：按 LLD v1 建立四个线运动模式、两个角运动模式、MotionState 六项真值；以事件时刻先累计旧段，再应用覆盖/BeginMotionMode/EndMotionEnd、圆周重激活策略和右侧导数。
- **验证**：公开定义建议置于 include/visualsim/Trajectory.hpp，src/Trajectory.hpp 作为内部桥接；V02–V04 检验历史重算、圆周一致性及非零角加速度拒绝。

##### F03 VisualSim/main/src/main.cpp

- **具体处理**：从硬编码的前向匀速/光轴旋转及三深度离散场景，改为显式读取用户 YAML 事件/采样时间、调用公共 Simulator，场景唯一 Room。
- **目标或排除**：把 Render、IMU、WriteSensorYaml/Imu、WriteGroundTruthHeader、WriteMotionYaml、GenerateDataset 拆到库内部 Renderer、ImuModel、DatasetExporter；不在 main 重复维护数学。
- **验证**：CLI 仅显式导出 PNG/CSV/YAML、支持 N 相机及真实纳秒；移除自动 remove_all 和启动即写盘；V01/V10 验证。

##### F04 VisualSim/main/tests/TrajectoryTests.cpp

- **具体处理**：保留解析位移、固定朝向、轴角纯旋转、Room 网格等已有有效断言，改用冻结的公开事件 API，不为测试保留硬编码轨迹入口。
- **目标或排除**：增加类内覆盖/跨类组合、嵌套作用域、相位/圆心恢复、事件时间与不连续边界、可查询历史的用例；按需要拆成 MotionHistoryTests 与 RoomTests。
- **验证**：V02/V03/V04/V08 负责验收，测试应与真实库链接而非直接复制核心源码。

#### 3.8.2 ros2-vio-node/ftr-eskf 的 VisualSim 头/源 F05–F19

##### F05 ros2-vio-node/ftr-eskf/src/VisualSim/Camera.hpp

- **具体处理**：来源 Camera::sensor_from_body_ 是 T_SB（B→C），冻结 API 使用 T_BC（C→B）；不能原样复制变换或固定 fx=fy=450 的默认内参。
- **目标或排除**：提取 K/投影/视域逻辑到 include/visualsim/Calibration.hpp 与 src/CameraRenderer.cpp；按 T_BC 取逆、实现独立各相机的 k1/k2/p1/p2 真实畸变及 LLD 默认 K。
- **验证**：V05/V06-D 用非平凡外参、多相机不同 K 和非零畸变核对像素/世界坐标。

##### F06 ros2-vio-node/ftr-eskf/src/VisualSim/ErrorEvaluation.hpp

- **具体处理**：估计误差评估依赖 DatumTruth，非本产品的仿真物理或图像生成接口；不复制其类、编译单元或加载链。
- **目标或排除**：如果需要比较真值，把姿态/轨迹差异仅写在目标 tests 的断言中，不能让 Simulator 链接估计器评估代码。
- **验证**：验收：目标公开 include、CMake target 和运行依赖均不含 ErrorEvaluation/DatumTruth。

##### F07 ros2-vio-node/ftr-eskf/src/VisualSim/EvoSim3.hpp

- **具体处理**：已有真值/估计结果加载，引用 AbstractLoader/反射；不是基于调用者事件/配置定义运动的 API。
- **目标或排除**：明确不纳入目标，不能把反射式加载器变成 v1 事件来源；替代路径是 LLD §9.1 的 LoadMotionConfig（只允许新实例）和时间戳即时式 API。
- **验证**：V02 确认 YAML baseline→events 与函数调用一致；目标无 EvoSim3 依赖。

##### F08 ros2-vio-node/ftr-eskf/src/VisualSim/ImuNoiseModel.cpp

- **具体处理**：这是另带 main() 的离线 CSV 加噪程序，读写先前生成的 IMU 序列，不属于按时刻直接生成传感器数据的库。
- **目标或排除**：不把该源文件/离线工具编进目标，不允许借此在 Generate 后隐式改写文件；噪声只能在新 ImuModel 的单次计算里数值生效。
- **验证**：V06-D/S 验证噪声，V10 检查无隐式写盘与额外可执行入口。

##### F09 ros2-vio-node/ftr-eskf/src/VisualSim/ImuNoiseModel.hpp

- **具体处理**：来源持有 mt19937、normal_distribution、gyro_bias_/accel_bias_ 并按固定 sqrt_rate 顺序推进，与 LLD §6.2 的按纳秒重算、无历史偏置缓存不兼容。
- **目标或排除**：只借鉴连续白噪声 N/√Δt 和偏置游走 K√Δt 的物理密度含义；在 src/ImuModel.cpp 重新实现 seed/epoch/整数时刻键控高斯与 Brownian-bridge。
- **验证**：禁止复制 Corrupt() 有状态时间序列；V06-D 固定种子重放、V06-S 四组 99% 卡方检验。

##### F10 ros2-vio-node/ftr-eskf/src/VisualSim/LinearKalmanFilter.hpp

- **具体处理**：OpenCV 的滤波/跟踪辅助组件，不是 Room 渲染、IMU 真值或真实交点求解；不纳入共享库。
- **目标或排除**：不得在生成像素或 Raycast 后附加“滤波真值”步骤，也不通过依赖关系导入 OpenCV video/tracking 估计模块。
- **验证**：V01 检查最终 CMake 依赖；V08 比对未经估计器修正的解析真值。

##### F11 ros2-vio-node/ftr-eskf/src/VisualSim/MeshPlot.hpp

- **具体处理**：来源按固定左右目共同可见网格绘制黑白三角面，且调用 namedWindow、imshow、destroyAllWindows；不能原样用在无 GUI 的 N 相机库。
- **目标或排除**：提取六面棋盘纹理规则到 src/RoomRenderer.cpp 或 CameraRenderer.cpp；逐相机 CV_8UC1 离屏渲染、按各自 K/畸变投影并处理近裁剪面和部分可见面。
- **验证**：图像遮挡与 Room 首次求交一致，无 GUI 生命周期副作用；V05/V08/V10。

##### F12 ros2-vio-node/ftr-eskf/src/VisualSim/Path.hpp

- **具体处理**：PathStationary/Acceleration/Circle 含有可复用解析式，但 PathManager 按浮点秒排队且圆周朝向和线运动相耦合。
- **目标或排除**：只吸收解析运动公式到新双类别模式/事件解释器，不复制 PathManager；圆周位置与相机朝向独立，段边界先积分旧规律再切换，支持栈恢复。
- **验证**：F02 与本文件共同覆盖 V03/V04，保证六项真值、右侧导数、同刻操作次序。

##### F13 ros2-vio-node/ftr-eskf/src/VisualSim/Preintegrate.hpp

- **具体处理**：引用 DatumImu 的惯性预积分，用于里程计/状态估计，非真实 IMU 测量生成。
- **目标或排除**：不纳入运行时库或其测试；目标 ImuMeasurement 只含三轴角速度/比力，不暴露滤波预积分状态。
- **验证**：V01 依赖核验：无 Preintegrate/DatumImu。

##### F14 ros2-vio-node/ftr-eskf/src/VisualSim/Room.hpp

- **具体处理**：该版可配置房间尺寸、按有限正数检验、维护 GetIndex；但带 stderr 构造打印，目标 Room 已有 Points() 与 visualsim 命名空间。
- **目标或排除**：合并进目标 src/Room.hpp 而不是保留两份类；保留必要网格索引、去重，删除自动日志，补连续六面与动态 RoomConfig，按事件时刻取值。
- **验证**：V08 验证不同房间尺度、退化参数拒绝、六面命中，且无跨调用几何结果缓存。

##### F15 ros2-vio-node/ftr-eskf/src/VisualSim/StereoRig.hpp

- **具体处理**：固定 camera_left_/camera_right_ 与两目公共可见索引交集，无法表达任意 0..N 相机及单目可见点。
- **目标或排除**：重写为稳定 SensorId 的 CameraRegistry；按时刻注册/改名、每台各自渲染，camera gate 一次包含所有当时存在相机。
- **验证**：默认虚拟 cam0 外参 (I,0)、自动 camX、改名不重置帧编号；V05/V07。

##### F16 ros2-vio-node/ftr-eskf/src/VisualSim/StressModel.hpp

- **具体处理**：额外 PixelStress 像素离群模型、脉冲线/角运动中含非零角加速度，与冻结 v1 仅有定轴匀速角模式相冲突。
- **目标或排除**：严格排除；不借旧 StressMotion 新增隐藏模式，也不能让压力噪声改变 Room 真实交点。
- **验证**：日后若扩充需要先升级 SRS/LLD；当前 V03/V04 模式枚举无 Stress。

##### F17 ros2-vio-node/ftr-eskf/src/VisualSim/StressTests.cpp

- **具体处理**：只验证被排除的 StressModel/PixelStress 与脉冲运动，不应作为 v1 新功能测试直接复制。
- **目标或排除**：有限差分和固定种子测试思想可在新的 MotionHistoryTests/ImuTests 中重写，且只针对合法 v1 模式。
- **验证**：目标测试构建不得链接 StressModel；V04/V06-D。

##### F18 ros2-vio-node/ftr-eskf/src/VisualSim/VisualInertial.cpp

- **具体处理**：含 rclcpp、ESKF、ZUPT、Datum/状态估计与 ROS 数据处理，即使路径在 src/VisualSim 中也不属于仿真运行库。
- **目标或排除**：不复制文件、include 或 ROS 目标；与之相关的 ROS 依赖和估计器数据结构均必须从目标 public API/编译链中排除。
- **验证**：V01 检查 ROS/ament/cv_bridge/rosidl 无直接或传递链接。

##### F19 ros2-vio-node/ftr-eskf/src/VisualSim/VisualSim.cpp

- **具体处理**：含 Room 双目渲染、IMU 理想比力、CSV/YAML、固定采样比与 main；同时耦合 Stress、ImuNoiseModel、CameraFrameWriter 和 remove_all。
- **目标或排除**：按职责分解到建议的 src/Simulator.cpp、CameraRenderer.cpp、ImuModel.cpp、DatasetExporter.cpp；只参考物理式与数据组织，不复制 main/宏/自动文件删除。
- **验证**：改为输入原始 TimestampNs、0..N 相机、事务性门控、无结果缓存；V05–V11。

#### 3.8.3 ros2-vio-node/ftr-eskf 的 CameraIO 与依赖 F20–F38

##### F20 ros2-vio-node/ftr-eskf/src/CameraIO/CameraFrameReader.cpp

- **具体处理**：读取既有 data.csv/PNG/FFV1 与视频时间索引，和按需仿真生成不同；不纳入目标。
- **目标或排除**：保留 CSV 的权威时间是整数 timestamp_ns 这一测试思想，不引入图像回放或 FFV1Decoder。
- **验证**：V10 的新导出测试独立解码刚生成的 PNG。

##### F21 ros2-vio-node/ftr-eskf/src/CameraIO/CameraFrameWriter.cpp

- **具体处理**：参考 ImageSequenceWriter 的 PNG 命名和 CSV 索引写入顺序，但不复制 FFV1 VideoWriter。
- **目标或排除**：在 src/DatasetExporter.cpp 重新实现显式输出目录与每个相机单独数据文件、真实 int64 时间；禁止在 Generate 内写盘或覆盖已有文件。
- **验证**：V10 核查 PNG 读取、CSV 顺序、N 相机、错误时无半写。

##### F22 ros2-vio-node/ftr-eskf/src/CameraIO/FFV1Video.hpp

- **具体处理**：FFV1Encoder/Decoder 属于视频扩展，不是 v1 可安装 API 的必需组成。
- **目标或排除**：排除头与公开依赖，防止安装树要求 FFmpeg 和视频帧格式。
- **验证**：V01 检查导出头/链接中无 FFV1Video。

##### F23 ros2-vio-node/ftr-eskf/src/CameraIO/FFV1Video.cpp

- **具体处理**：依赖 libavcodec/libavformat/libavutil，写入 FFV1 需要 CV_8UC3/BGR0，与冻结 CV_8UC1 不一致。
- **目标或排除**：不迁移到 libvisualsim.so、CLI 或默认测试；如需视频须单独升版并设可选插件。
- **验证**：V01 ELF NEEDED 不因本模块出现强制 libav 依赖。

##### F24 ros2-vio-node/ftr-eskf/src/CameraIO/CameraFrameIOTests.cpp

- **具体处理**：现有测试覆盖 PNG/CSV 和 FFV1/VisualSlam 读写，不适合原样复制。
- **目标或排除**：提取 timestamp_ns 索引、空/损坏文件、PNG 可读性断言到目标 tests/ExportTests.cpp；去掉 FFmpeg/VisualSlam 部分。
- **验证**：V10 测像素值、格式和目录覆盖失败。

##### F25 ros2-vio-node/ftr-eskf/include/euroc_vio/CameraFrameIO.hpp

- **具体处理**：同时暴露 Images/Video Writer/Reader、索引与视频模式，不应替代 Simulator 的纯内存传感器帧。
- **目标或排除**：参考按纳秒写入契约，在目标建议的 include/visualsim/DatasetExporter.hpp 只公开用户授权的 PNG/CSV 导出；不提供 FFV1/Reader。
- **验证**：V01/V10 检查 API/写盘时机与内存所有权。

##### F26 ros2-vio-node/ftr-eskf/include/euroc_vio/AbstractLoader.hpp

- **具体处理**：来源 CSV 加载基类，是 EvoSim3/Datum* 的附属工具。
- **目标或排除**：不复制，不用该加载器处理 schema v1；改由 src/MotionConfigLoader.cpp 校验 YAML baseline→events 并原子追加。
- **验证**：V02/LLD §9.1 验证 LoadMotionConfig 新实例限制。

##### F27 ros2-vio-node/ftr-eskf/include/euroc_vio/DatumFast.hpp

- **具体处理**：面向估计状态或外部数据加载，不属于运行时物理状态。
- **目标或排除**：不迁移，公开状态采用冻结 MotionState 而非来源 Fast 状态。
- **验证**：V01 确认头文件依赖不含 DatumFast/AbstractLoader。

##### F28 ros2-vio-node/ftr-eskf/include/euroc_vio/DatumImu.hpp

- **具体处理**：已有 IMU 数据读取/预积分协作结构，不是所需的瞬时比力与角速度输出。
- **目标或排除**：不纳入，目标 ImuMeasurement 自带严格单位和 SensorId/时间；禁止链接 Preintegrate。
- **验证**：V06-D 验证理想 IMU 与非零杆臂项。

##### F29 ros2-vio-node/ftr-eskf/include/euroc_vio/DatumTruth.hpp

- **具体处理**：加载既有真值/轨迹以供估计比较，不能作为即时仿真的主运动来源。
- **目标或排除**：不迁移，真实运动由事件历史按给定时间重算并在需要时即时导出。
- **验证**：V04/V09 核对事件重放无需历史真值缓存。

##### F30 ros2-vio-node/ftr-eskf/include/euroc_vio/ErrorStateKalmanFilter.hpp

- **具体处理**：含后端状态、滤波更新和历史帧缓冲，不属于 VisualSim。
- **目标或排除**：排除其头、对象、命名空间和任何隐含历史路标地图；不编进独立库。
- **验证**：V01/V09 检查无估计器模块、无结果缓存。

##### F31 ros2-vio-node/ftr-eskf/include/euroc_vio/SensorState.hpp

- **具体处理**：定义估计器 ImuState/ImuDerivative 数组，与最终六项物理真值的坐标系合同不同。
- **目标或排除**：不复用为 MotionState，明确 p/v/a 属于世界系、ω/α 属于 B 系。
- **验证**：V04/V06-D 通过坐标变换一致性验证。

##### F32 ros2-vio-node/ftr-eskf/include/euroc_vio/SensorYaml.hpp

- **具体处理**：可参考 EuRoC 的 K、T_BS、噪声字段与非法值检查，但未定义事件时间线或动态重命名。
- **目标或排除**：重写目标 schema v1 解析和导出字段：严格整数纳秒、四元数 wxyz、K 行主序、Unknown 字段拒绝、新实例 LoadMotionConfig/EventHistoryFrozen。
- **验证**：建议 src/MotionConfigLoader.cpp、DatasetExporter.cpp；V02/V05/V11。

##### F33 ros2-vio-node/ftr-eskf/include/euroc_vio/ZUPT.hpp

- **具体处理**：估计器零速检测及窗口缓冲，会把观测推断成运动模式，背离调用者的显式定义。
- **目标或排除**：不复制，Stationary 由 API/YAML 事件给出而不是 ZUPT 自动判断。
- **验证**：V03/V09 检查无 ZUPT 和窗口缓存。

##### F34 ros2-vio-node/ftr-eskf/include/euroc_vio/StereoObservation.hpp

- **具体处理**：ESKF 固定双目观测结构不适合多相机 SensorFrame。
- **目标或排除**：不纳入，单目/双目/N 相机由 CameraRegistry 统一按 SensorId 输出。
- **验证**：V05/V07 检查不做左右公共可见点强制交集。

##### F35 ros2-vio-node/ftr-eskf/include/euroc_vio/CircularBuffer.hpp

- **具体处理**：主要供 ZUPT 使用的滑窗缓存，不符合本版禁止历史结果缓存的约束。
- **目标或排除**：不迁移，不用它缓存 IMU 样本、计算位姿或 Raycast 命中；只允许持久事件历史及最小序号状态。
- **验证**：V09 内存结构/源码审查。

##### F36 ros2-vio-node/ftr-eskf/include/euroc_vio/VisionMode.hpp

- **具体处理**：仅来源 VisualSlam 图像读取测试依赖的模式类型，不是 camera/imu SensorGate。
- **目标或排除**：不复制或加入公共头；门控按 LLD §2 的 camera、imu 布尔字段。
- **验证**：V07 验证门控组合，V01 扫描排除 VisionMode。

##### F37 ros2-vio-node/ftr-eskf/include/euroc_vio/pch.hpp

- **具体处理**：源工程预编译头包含 rclcpp、cv_bridge、ROS 消息与 message_filters。
- **目标或排除**：严格排除；不得让目标 target_include_directories/target_precompile_headers 引入 common_pch 或 ROS include。
- **验证**：V01 进行头文件独立编译、ELF NEEDED/ldd ROS 扫描。

##### F38 ros2-vio-node/ftr-eskf/src/VisualSlam/ImageDataLoader.hpp

- **具体处理**：是 CameraFrameIOTests 间接依赖的图像读取器，不属于数据生成库。
- **目标或排除**：不迁移；目标 ExportTests 自行解码当次 PNG 验证，不链接 VisualSlam。
- **验证**：V01/V10 检查无 VisualSlam 依赖。


### 3.9 构建文件与脚本逐项修改细则（对应 §3.6 的 8 行）

以下是实施阶段的文件级修改合同。§3.6 表中的“未来处理”结论不变，新增说明细化怎样实施、怎样判定完成；来源仓库仍只读。

##### B01 VisualSim/main/CMakeLists.txt

- **修改步骤**：在配置阶段拒绝非 Linux；分离 SHARED libvisualsim.so、薄 CLI、CTest 与安装树外消费者。使用 GNUInstallDirs、CMakePackageConfigHelpers、install(EXPORT)、带版本的 VisualSimConfig.cmake/Targets，设置可安装公开头路径与实际非 ROS 依赖，并明确 SONAME/ABI 边界。
- **边界和取舍**：目标库以 find_package(VisualSim CONFIG REQUIRED)、VisualSim::VisualSim 被独立工程消费；不把来源的 ament、common_pch、ROS 包、camera_frame_io/FFmpeg 或全局 -march=native/-freflection 无条件移入。按真实使用的语言特性设标准，不强制降低现有 GCC 16.2.0/CMake 4.3.4。
- **完成检查**：P1/V01：Ubuntu 24.04 构建、指定前缀安装、运行独立消费者、扫描依赖；当前不执行。

##### B02 VisualSim/main/CMakePresets.json

- **修改步骤**：删除 windows 名称、Visual Studio 生成器、x64-windows triplet、Windows 盘符目录的固定值；改为按需提供 Linux Ninja/Unix Makefiles 的 Debug/Release configure/build/test presets。
- **边界和取舍**：Presets 的 CMAKE_INSTALL_PREFIX/依赖前缀可由调用者覆盖，禁止机器专属硬编码；不得内含 Windows 可用构建入口。
- **完成检查**：P1：cmake --list-presets 和 Ubuntu 配置验证，CMake 非 Linux 配置明确失败。

##### B03 VisualSim/main/build.ps1、run.ps1、clean.ps1

- **修改步骤**：这三个都是 Windows/PowerShell 入口，实施时从目标仓库移除并删除 README 调用示例；如需 Linux 自动化则另外提供不强制指定路径的 .sh 或说明命令。
- **边界和取舍**：任何新执行脚本都不能隐式 remove_all 用户输出目录，不得预先清除用户数据；独立 .so 及 CLI 的构建/运行入口由 Linux CMake 主导。
- **完成检查**：P1/P6 检查三个 .ps1 已无可运行的 Windows 构建路径。

##### B04 VisualSim/main/vcpkg.json

- **修改步骤**：先检查它是否仍由 Linux 实际构建使用；首选 CMake find_package 已安装的 Eigen/OpenCV/Sophus/yaml-cpp，若未实际使用 vcpkg 则删除 manifest。
- **边界和取舍**：如果因用户 Linux 环境需要而保留，应标成可选 Linux 包管理方式，不使用 x64-windows/盘符路径、不把 FFmpeg 默认引入，也不修改用户现有编译器版本。
- **完成检查**：P1 的干净安装前缀消费者验收；不能把是否启用 vcpkg 当作功能先决条件。

##### B05 VisualSim/main/README.md

- **修改步骤**：替换 Windows 11 + PowerShell 指南，明确 Ubuntu 24.04 Linux-only 的编译、指定前缀安装、外部 C++ find_package/链接/运行命令。
- **边界和取舍**：增加函数与 YAML 定义运动事件、BeginMotionMode/EndMotionEnd、动态 camX、Room-only、int64 纳秒、Generate/Raycast、IMU 噪声与显式 PNG/CSV 数据集导出示例；严格区分设计计划和真实实现状态。
- **完成检查**：P1/P6 按 README 复做命令，拒绝 Windows 编译和隐式覆盖文件。

##### B06 ros2-vio-node/ftr-eskf/CMakeLists.txt

- **修改步骤**：这是 ROS 主工程，含 ament、rclcpp、ROS 消息以及 common_pch；保留为只读参考，仅用于确认来源真实依赖。
- **边界和取舍**：**不迁移、不修改源仓库**；目标从零建立独立 Linux SHARED 构建，不通过 add_subdirectory 把 ROS 根工程编进来。
- **完成检查**：V01 通过源码依赖、公开头与 ELF 动态依赖检查无 ROS。

##### B07 ros2-vio-node/ftr-eskf/src/VisualSim/CMakeLists.txt

- **修改步骤**：来源独立子目录定义 VisualSim 可执行程序、C++23、Sophus/Eigen/OpenCV、CameraIO 子目录及 StressTests，仍非可安装运行时库。
- **边界和取舍**：不直接复制；目标按所需源码建立真正的共享库，排除固定双目/FFmpeg/Stress 额外模块；C++23 特性仅在真实需要时采用。
- **完成检查**：P1/V01 检查 SHARED target 的安装路径、依赖与消费端，无来源目录路径残留。

##### B08 ros2-vio-node/ftr-eskf/src/CameraIO/CMakeLists.txt

- **修改步骤**：来源构建 camera_frame_io STATIC，依赖 OpenCV videoio、FFmpeg pkg-config libavcodec/libavformat/libavutil，并提供 Windows SDK fallback。
- **边界和取舍**：不迁入、不复制其平台选择；目标仅由 DatasetExporter 输出 CV_8UC1 PNG/CSV，绝不把 libav* 或 Windows 编码器 SDK 作为 v1 强制依赖。
- **完成检查**：P1/P6 通过 link target/ldd/readelf 检查 FFmpeg 非必需。

### 3.10 文件级改动与 v1 端到端功能闭环

| 功能链 | 需要一起完成的文件工作（原表 F 编号） | 实际验收入口 |
| --- | --- | --- |
| 独立库 | F03 拆 CLI、F25/F32 重建导出/配置接口，B01–B08 控制 Linux/无 ROS；F18/F37 严格排除 | P1/V01 |
| 运动事件 | F02/F12 组合六模式解析式与时刻历史、F04 测试，F07 不引入读取式轨迹 | P2/V02–V04 |
| N 相机 | F05/F15/F19 重建注册、标定、生成；不引入 F34 的双目估计观测 | P3/P5/V05、V07 |
| IMU | F09 重做无缓存噪声，F19 迁移理想比力物理式，F08/F28/F30 不进入库 | P3/V06-D、V06-S |
| Room + 视线 | F01/F14 唯一 Room 真值、F11 离屏渲染，F10 不能改变解析交点 | P4/V08 |
| 显式导出 | F03/F19/F21/F25/F32 整合 PNG/CSV/YAML，F20/F22/F23/F24/F38 仅按允许范围取用测试思想 | P6/V10、V11 |
| 排除性检查 | F06/F07/F08/F10/F13/F16/F17/F18/F20/F22/F23/F26–F31/F33–F38 等不应出现为目标库强制依赖 | P1/P6/V01、V09 |

完成某个单独文件的改写，不等于上表功能链通过。所有生产代码的实现、编译与测试结果须在后续 P0–P6 记录；本轮仅完善实施计划。

### 3.11 运行环境未确认信息与只读取证命令

**原始输出及后续命令台账见 [BUILD_ENVIRONMENT.md](BUILD_ENVIRONMENT.md)，其 E001–E016 已记录此前两轮用户反馈。** 本节的 shell 指令仅为环境查询参考，不构成“待再次执行”的要求；不要重复要求用户已经完成的命令。任何新检查应先在取证文件登记待执行编号，再提出给用户。



**已知**：用户提供 Ubuntu 24.04、CMake 4.3.4、GCC C++ 16.2.0，禁止为兼容发行版默认软件包而降低已有版本。**尚待实际机器核实**：Eigen/OpenCV/Sophus/yaml-cpp 的版本与 CMake 包查找路径、glibc/libstdc++ ABI、处理器架构和真实链接依赖。下面命令不编译、不安装、不运行 CTest、不修改两仓库，也不要求 sudo；获取输出后再用于 P0/P1 取证。

~~~bash
# OS、架构、编译器、CMake 与标准库
cat /etc/os-release | grep -E '^(NAME|VERSION|VERSION_ID)='
uname -m
command -v cmake c++
cmake --version
c++ --version
c++ -dumpmachine
c++ -print-file-name=libstdc++.so.6
ldd --version | head -n 1

# pkg-config 能识别的非 ROS 开发库；缺失信息不等于库未安装
if command -v pkg-config >/dev/null 2>&1; then
  for pkg in eigen3 opencv4 yaml-cpp; do
    printf '%s: ' "$pkg"
    pkg-config --modversion "$pkg" 2>&1 || true
  done
fi

# Sophus 等 CMake Config 实际路径；只读查找
find /usr /usr/local /opt -type f   ( -name 'SophusConfig.cmake' -o -name 'sophus-config.cmake'      -o -name 'Eigen3Config.cmake' -o -name 'OpenCVConfig.cmake'      -o -name 'yaml-cpp-config.cmake' ) -print 2>/dev/null | head -n 80
~~~

若没有找到某个 Config 文件，只能说明该查询路径中未找到，**不能直接断言缺少依赖**。后续实施 P0 时再按实际 CMAKE_PREFIX_PATH 和 find_package 进行检查。当前不要求安装新库或修改 C++ 标准。



### 3.12 已有文件的真实改动前/拟议改动后代码对照

本节为前面 38 个头/源文件、8 项构建/脚本登记提供逐项代码级证据。**“改动前”片段是固定提交中真实存在的原代码**，每个附原仓库、commit、文件路径与 GitHub 行号。**“改动后”是目标仓库将来实施的示例片段**，涉及新私有辅助函数时已注明尚须实现；本轮没有编译、没有修改 C++ 或 CMake。对于不迁移的来源文件，展示的是目标替代代码或明确的构建排除，不是篡改来源文件。

固定源码快照：VisualSim/main 为 dbfab8e6d6b7619c72a0050c8a760d748d6e9628，ros2-vio-node/ftr-eskf 为 fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03。若后续这些来源提交有变化，P0 应重新核对实际源码后再使用本章，不得盲目按旧行号修改。

### 3.12.1 F01–F10：真实源码对照与拟议代码

本节“改动前”是来自固定提交的**实际文件逐行原文**；“计划改动后”是未编译、未实施的目标片段，引用的私有辅助函数将在后续 P1–P6 实现。对于排除项，后半段是目标替代/编译排除，不是修改来源文件。

#### F01 — src/Room.hpp

**改动前（逐字摘录）**：[ncut-d1z/VisualSim@dbfab8e6d6b7 src/Room.hpp L21–L26](https://github.com/ncut-d1z/VisualSim/blob/dbfab8e6d6b7619c72a0050c8a760d748d6e9628/src/Room.hpp#L21-L26)

```cpp

  const value_type width_{10.0};
  const value_type depth_{10.0};
  const value_type height_{3.0};
  const Point3 center_{depth_ * static_cast<value_type>(0.5),
                       width_ * static_cast<value_type>(0.5),
```

**计划改动后（未写入源码的上下文片段）**：

```cpp
// 目标：Room 的尺寸随指定时刻有效的 RoomConfig 改变。
template<class T> T CheckedStep(T length, int subdivisions) {
  if (!std::isfinite(length) || length <= T{0} ||
      subdivisions <= 0 || subdivisions % 2 != 0)
    throw std::invalid_argument("invalid Room dimensions/grid");
  return length / static_cast<T>(subdivisions);
}
// Room(T depth, T width, T height, ...):
//   depth_(depth), width_(width), height_(height),
//   step_d_(CheckedStep(depth, count_depth)), ...
// 唯一连续真值面仍是 x=0/depth、y=0/width、z=0/height。
```

**取舍及验收**：保留目标 Room 的六面格点/Points()；按时间取得 RoomConfig，使用 CheckedStep 防止非法边长/分割数，求交计算连续有限面而非格点。V08。

#### F02 — src/Trajectory.hpp

**改动前（逐字摘录）**：[ncut-d1z/VisualSim@dbfab8e6d6b7 src/Trajectory.hpp L41–L46](https://github.com/ncut-d1z/VisualSim/blob/dbfab8e6d6b7619c72a0050c8a760d748d6e9628/src/Trajectory.hpp#L41-L46)

```cpp
 protected:
  double clampTime(double time_s) const noexcept {
    return std::clamp(time_s, 0.0, duration_s_);
  }

 private:
```

**计划改动后（未写入源码的上下文片段）**：

```cpp
// 目标：以用户的整型事件时间分段，而不是静默 clampTime。
Result<MotionState> Simulator::GetMotionState(TimestampNs at_ns) const {
  if (at_ns < 0) return {Status::InvalidTimestamp, std::nullopt,
                         "negative timestamp"};
  return impl_->EvaluateAt(at_ns); // 新增：按历史事件顺序解析每个运动段
}
// EvaluateAt 内：先累计 [previous_event_ns, event_ns]，
// 再按 insertion_order 应用 Begin/Set/End，并积分剩余区间。
// 此处仅展示接口替换点；完整 Evaluate 合同见 LLD §4。
```

**取舍及验收**：旧 duration_s/clampTime 不能决定历史查询，未来实现需按 LLD §4 的时间事件解析出六项 MotionState。V02–V04。

#### F03 — src/main.cpp

**改动前（逐字摘录）**：[ncut-d1z/VisualSim@dbfab8e6d6b7 src/main.cpp L207–L212](https://github.com/ncut-d1z/VisualSim/blob/dbfab8e6d6b7619c72a0050c8a760d748d6e9628/src/main.cpp#L207-L212)

```cpp

  fs::remove_all(root.parent_path());
  fs::create_directories(image_dir);
  fs::create_directories(imu0);
  fs::create_directories(gt_dir);

```

**计划改动后（未写入源码的上下文片段）**：

```cpp
// 目标：生成和求交接口不写文件；只有显式 CLI 导出才创建目录。
Result<void> ExportDataset(const std::filesystem::path& root,
                           const SampleBatch& batch) {
  if (std::filesystem::exists(root))
    return {Status::InvalidArgument, "output directory already exists"};
  // 在已验证的新目录中提交 PNG/CSV/YAML 导出事务，
  // 出错清理由专属导出目录管理，不调用 remove_all(user_root)。
  return WriteNewDatasetFiles(root, batch);
}
```

**取舍及验收**：将原 main 的 Render/IMU/YAML/CSV 拆进库与 Exporter，CLI 只读取 YAML 和显式输出路径；“WriteNewDatasetFiles”为未来导出私有函数，非现存 API。V10。

#### F04 — tests/TrajectoryTests.cpp

**改动前（逐字摘录）**：[ncut-d1z/VisualSim@dbfab8e6d6b7 tests/TrajectoryTests.cpp L22–L29](https://github.com/ncut-d1z/VisualSim/blob/dbfab8e6d6b7619c72a0050c8a760d748d6e9628/tests/TrajectoryTests.cpp#L22-L29)

```cpp
      ForwardConstantVelocity path(
          10.0, 0.5, Eigen::Vector3d(1.0, 2.0, 3.0),
          Eigen::Matrix3d::Identity());

      const auto p0 = path.pose(0.0);
      const auto p2 = path.pose(2.0);
      const auto k = path.kinematics(7.0);

```

**计划改动后（未写入源码的上下文片段）**：

```cpp
// 目标测试：检验已经冻结的事件 API，而非硬编码 Trajectory 对象。
visualsim::Simulator sim;
auto set = sim.SetLinearMotion(
    0, visualsim::LinearConstantVelocity{
        Eigen::Vector3d{0.0, 0.0, 0.5}});
Require(set.status == visualsim::Status::Ok, "SetLinearMotion failed");
auto state = sim.GetMotionState(2'000'000'000LL);
Require(state.status == visualsim::Status::Ok, "motion query failed");
Require((state.value->position_W_m -
        Eigen::Vector3d{0.0, 0.0, 1.0}).norm() < 1e-12,
        "constant-velocity displacement incorrect");
```

**取舍及验收**：这是 Test 函数内的示例片段；另须测试初始位置、作用域、圆周、角位移、右侧导数与错误返回，保留旧数学真值基准。V02–V04。

#### F05 — src/VisualSim/Camera.hpp

**改动前（逐字摘录）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/VisualSim/Camera.hpp L25–L31](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSim/Camera.hpp#L25-L31)

```cpp
  // 相机外参: 载具坐标系 → 传感器坐标系的变换 (T_SB), p_S = T_SB · p_B
  Pose sensor_from_body_{};
  int width_{752};
  int height_{480};

  Camera()
  {
```

**计划改动后（未写入源码的上下文片段）**：

```cpp
// 目标 src/CameraRenderer.cpp：冻结 T_BC 是 C -> B，不是来源 T_SB。
Eigen::Isometry3d T_WB = BodyPoseAt(at_ns);
Eigen::Isometry3d T_WC = T_WB * camera.T_BC;
Eigen::Vector3d X_C = T_WC.inverse() * X_W;
if (!(X_C.z() > 0.0)) return ProjectedPoint::BehindCamera;
const double x = X_C.x() / X_C.z(), y = X_C.y() / X_C.z();
const Eigen::Vector2d xd = DistortRadtan(x, y, camera.radtan);
const Eigen::Vector3d pixel_h = camera.K * Eigen::Vector3d{xd.x(),xd.y(),1.0};
// DistortRadtan 是待实现的 LLD §5.2 畸变公式，不是来源已有函数。
```

**取舍及验收**：目标将 T_BC 统一为 C→B，进入相机则逆变换；相机 K/畸变参数从该时刻配置获得，不能复制源 fx=450 默认值。V05/V06-D。

#### F06 — src/VisualSim/ErrorEvaluation.hpp

**改动前（逐字摘录）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/VisualSim/ErrorEvaluation.hpp L11–L15](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSim/ErrorEvaluation.hpp#L11-L15)

```cpp

#include "euroc_vio/DatumTruth.hpp"

class ErrorEvaluation
{
```

**计划改动后（未写入源码的上下文片段）**：

```cmake
# 目标 CMakeLists.txt：仅编入仿真源文件，不编入 ErrorEvaluation.hpp 的估计器。
add_library(visualsim SHARED
  src/Simulator.cpp src/MotionHistory.cpp src/CameraRenderer.cpp
  src/ImuModel.cpp src/RoomRenderer.cpp src/RayCaster.cpp)
# 估计误差由 tests 单独基于 MotionState 计算；
# ErrorEvaluation.hpp、DatumTruth.hpp 不在目标 public includes 或 target 中。
```

**取舍及验收**：来源文件保持原样、不会修改；这是目标将采用的源文件白名单示例，不是删除来源文件。V01。

#### F07 — src/VisualSim/EvoSim3.hpp

**改动前（逐字摘录）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/VisualSim/EvoSim3.hpp L23–L27](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSim/EvoSim3.hpp#L23-L27)

```cpp

#include "euroc_vio/AbstractLoader.hpp"

class EvoSim3
{
```

**计划改动后（未写入源码的上下文片段）**：

```cpp
// 目标使用 v1 YAML 事件而非来源 EvoSim3 数据加载器。
visualsim::Simulator simulator;
auto loaded = simulator.LoadMotionConfig("motion.yaml");
if (loaded.status != visualsim::Status::Ok)
  throw std::runtime_error(loaded.message);
auto truth = simulator.GetMotionState(2'000'000'000LL);
// 只有 LoadMotionConfig 新实例限制成立才允许加载；见 LLD §9.1。
```

**取舍及验收**：不迁移 EvoSim3.hpp，也不引入 AbstractLoader/反射 API。V02。

#### F08 — src/VisualSim/ImuNoiseModel.cpp

**改动前（逐字摘录）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/VisualSim/ImuNoiseModel.cpp L230–L235](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSim/ImuNoiseModel.cpp#L230-L235)

```cpp

int InjectImuNoise(const std::filesystem::path &path_mav0)
{
  const std::filesystem::path path_imu_data{path_mav0 / "imu0" / "data.csv"};
  const std::filesystem::path path_sensor_yaml{
      path_mav0 / "imu0" / "sensor.yaml",
```

**计划改动后（未写入源码的上下文片段）**：

```cpp
// 目标：IMU 加噪在本次 Generate 的临时事务完成，不另写 CSV。
visualsim::Simulator sim;
auto batch = sim.Generate(5'000'000LL, {.camera = false, .imu = true});
if (batch.status != visualsim::Status::Ok)
  throw std::runtime_error(batch.message);
// CSV 导出若需要须另调用显式的 DatasetExporter，
// 不能复制来源独立 main() / InjectImuNoise(path_mav0)。
```

**取舍及验收**：禁复制来源离线 CSV 加噪 CLI，禁在 Generate 中隐式文件操作。V06/V10。

#### F09 — src/VisualSim/ImuNoiseModel.hpp

**改动前（逐字摘录）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/VisualSim/ImuNoiseModel.hpp L76–L83](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSim/ImuNoiseModel.hpp#L76-L83)

```cpp
  {
    gyro_bias_ += SampleIsotropicGaussian(sigma_gyro_bias_walk_);
    accel_bias_ += SampleIsotropicGaussian(sigma_accel_bias_walk_);
    return Measurement{
        gyro_truth + gyro_bias_ + SampleIsotropicGaussian(sigma_gyro_noise_),
        accel_truth + accel_bias_ + SampleIsotropicGaussian(sigma_accel_noise_),
    };
  }
```

**计划改动后（未写入源码的上下文片段）**：

```cpp
// 目标 src/ImuModel.cpp 的片段：样本按整数时间重算，不持久保存 bias。
const double dt_s = last_imu_ns
    ? SecondsChecked(at_ns - *last_imu_ns)
    : 1.0 / calibration.nominal_rate_hz;
const Eigen::Vector3d white = Gaussian3ByKey(
    calibration.seed, noise_epoch_id, sensor_id, at_ns)
    * (calibration.gyroscope_noise_density / std::sqrt(dt_s));
const Eigen::Vector3d bg = calibration.initial_gyro_bias_S
    + calibration.gyroscope_random_walk * BrownianBridgeAt(
        calibration.seed, noise_epoch_ns, at_ns, 0);
// 以上 helpers 按 LLD §6.2 实现 SplitMix64/Box–Muller/布朗桥树。
const Eigen::Vector3d gyro_m = gyro_truth_S + bg + white;
```

**取舍及验收**：旧 mt19937、顺序偏置和固定步长不得迁移；Gaussian3ByKey/BrownianBridgeAt 是必须新实现的私有函数，不能仅声明。V06-D/S。

#### F10 — src/VisualSim/LinearKalmanFilter.hpp

**改动前（逐字摘录）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/VisualSim/LinearKalmanFilter.hpp L6–L11](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSim/LinearKalmanFilter.hpp#L6-L11)

```cpp
#include <opencv2/core/types.hpp>
#include <opencv2/video/tracking.hpp>

// @see: https://docs.opencv.org/4.x/dc/d2c/tutorial_real_time_pose.html
// @see: https://docs.opencv.org/4.x/dd/d6a/classcv_1_1KalmanFilter.html

```

**计划改动后（未写入源码的上下文片段）**：

```cpp
// 目标公共生成接口直接返回合成图像，不调用特征跟踪滤波。
Result<SampleBatch> Simulator::Generate(TimestampNs at_ns, SensorGate gate) {
  auto snapshot = impl_->EvaluateWithCalibration(at_ns);
  if (snapshot.status != Status::Ok)
    return {snapshot.status, std::nullopt, snapshot.message};
  return impl_->GenerateAtomic(*snapshot.value, gate);
}
// 生成路径不含 LinearKalmanFilter、cv::KalmanFilter 或光流跟踪器。
```

**取舍及验收**：未来示意涉及的新私有 EvaluateWithCalibration/GenerateAtomic 必须在 P3/P5 实现；源文件保持不变，不进入目标 CMake。


### 3.12.2 F11–F19：来源 VisualSim 的真实片段与计划代码

#### F11 — src/VisualSim/MeshPlot.hpp

**改动前（来源真实原文，来源仓库不修改）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/VisualSim/MeshPlot.hpp L57–L64](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSim/MeshPlot.hpp#L57-L64)

```cpp
#ifdef VISUALSIM_HAS_HIGHGUI
    cv::destroyAllWindows();
#endif
  }

  // 使用索引表示三角形
  struct Triangle : public std::array<std::size_t, 3>
  {
```

**迁移后的目标片段（未实施代码草案）**：

```cpp
// 目标 src/RoomRenderer.cpp：离屏、逐相机，无 highgui 生命周期。
cv::Mat RoomRenderer::Render(const MotionState& state,
                             const CameraCalibration& cam,
                             const RoomConfig& room) const {
  cv::Mat image(cam.height, cam.width, CV_8UC1, cv::Scalar(18));
  for (int v = 0; v < cam.height; ++v)
    for (int u = 0; u < cam.width; ++u)
      image.at<std::uint8_t>(v, u) =
          ShadeRoomFirstHit(u, v, state, cam, room);
  return image;
}
// ShadeRoomFirstHit: 反畸变得单位视线，调用同一有限 Room 面求交。
```

**边界与验证**：不得带入 cv::namedWindow、destroyAllWindows 或左右共享可见点交集；`ShadeRoomFirstHit` 为需实现的私有离屏求交/着色帮助函数。V05/V08/V10。

#### F12 — src/VisualSim/Path.hpp

**改动前（来源真实原文，来源仓库不修改）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/VisualSim/Path.hpp L39–L46](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSim/Path.hpp#L39-L46)

```cpp

  virtual Pose GetPose(value_type time) const = 0;

  virtual Kinematics GetKinematics(value_type time) const = 0;

  virtual value_type GetDefaultDuration() const
  {
    return std::numeric_limits<value_type>::infinity();
```

**迁移后的目标片段（未实施代码草案）**：

```cpp
// 目标 src/MotionHistory.cpp：双类别分别更换，不在 PathManager 排队。
Result<void> Simulator::SetAngularMotion(TimestampNs ns,
                                         const AngularMotionSpec& spec) {
  return impl_->AppendValidatedEvent(ns, SetAngularEvent{spec});
}
// EvaluateAt 对每个事件先调用 AdvanceAnalytic(state, linear, angular, dt);
// 然后设置 active.angular = spec，不改变 active.linear。
// 角状态满足 R_WB = R_anchor * Exp([axis_B * rate * dt]x)。
```

**边界与验证**：必须补完整的事件校验、作用域栈与圆周重入；不复用旧的浮点 duration 作为事件时间键。V03/V04。

#### F13 — src/VisualSim/Preintegrate.hpp

**改动前（来源真实原文，来源仓库不修改）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/VisualSim/Preintegrate.hpp L9–L16](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSim/Preintegrate.hpp#L9-L16)

```cpp

#include "euroc_vio/DatumImu.hpp"

struct Preintegration
{
  // 范围内总的预积分旋转量
  Eigen::Quaterniond delta_attitude_{Eigen::Quaterniond::Identity()};
  // 范围内总的预积分平移量
```

**迁移后的目标片段（未实施代码草案）**：

```cpp
// 目标 include/visualsim/Types.hpp：传感器直接输出，而非预积分。
struct ImuMeasurement {
  Eigen::Vector3d angular_velocity_S_radps;
  Eigen::Vector3d specific_force_S_mps2;
};
struct SensorFrame {
  TimestampNs timestamp_ns;
  SensorId sensor_id;
  std::string sensor_name;
  std::uint64_t frame_number;
  std::variant<cv::Mat, ImuMeasurement> payload;
};
```

**边界与验证**：来源预积分文件保留在只读仓库，不编进目标。目标 CMake 不引入 DatumImu、Preintegrate 或 ODE/EKF。V01/V06。

#### F14 — src/VisualSim/Room.hpp

**改动前（来源真实原文，来源仓库不修改）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/VisualSim/Room.hpp L56–L63](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSim/Room.hpp#L56-L63)

```cpp
   */
  Room(int cnt_sep_depth = 20, int cnt_sep_width = 20, int cnt_sep_height = 6,
       value_type depth = static_cast<value_type>(10.0),
       value_type width = static_cast<value_type>(10.0),
       value_type height = static_cast<value_type>(3.0)) :
    width_{width}, depth_{depth}, height_{height},
    cnt_sep_depth_{cnt_sep_depth}, cnt_sep_width_{cnt_sep_width},
    cnt_sep_height_{cnt_sep_height}, step_d_{depth_ / cnt_sep_depth_},
```

**迁移后的目标片段（未实施代码草案）**：

```cpp
// 目标 src/Room.hpp 扩展：先拒绝非法动态尺寸/网格数。
template<class T> struct Room {
  static T Step(T length, int divisions) {
    if (!(std::isfinite(length) && length > T{0}) ||
        divisions <= 0 || divisions % 2 != 0)
      throw std::invalid_argument("invalid Room");
    return length / static_cast<T>(divisions);
  }
  // 实例化时由 GetRoom(at_ns) 取得该时刻 depth/width/height；
  // 保留目标 Points()，无自动 stderr 打印；连续表面由 Room::firstHit 表达。
};
```

**边界与验证**：将来源可变尺寸与校验合并进**目标单一** Room，不建立第二套同名 namespace 类型。V08。

#### F15 — src/VisualSim/StereoRig.hpp

**改动前（来源真实原文，来源仓库不修改）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/VisualSim/StereoRig.hpp L20–L27](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSim/StereoRig.hpp#L20-L27)

```cpp
template <typename value_type>
struct StereoRig
{
  Camera<value_type> camera_left_;
  Camera<value_type> camera_right_;

  using Point2 = Eigen::Vector<value_type, 2>;
  using Pose   = typename Camera<value_type>::Pose;
```

**迁移后的目标片段（未实施代码草案）**：

```cpp
// 目标 src/CameraRegistry.cpp：传感器实例不是固定 left/right。
Result<CameraHandle> Simulator::CreateCamera(
    TimestampNs ns, const Eigen::Isometry3d& T_BC) {
  auto state = impl_->ValidateCameraCreation(ns, T_BC);
  if (state.status != Status::Ok)
    return {state.status, std::nullopt, state.message};
  return impl_->CommitCameraCreationNoexcept(*state.value);
}
// CameraRegistry 用稳定 SensorId 和严格不回收的 next_auto_camera_index；
// camera gate 逐个输出时刻 t 有效的所有 CameraHandle。
```

**边界与验证**：新注册/命名/历史化标定的时序与默认 cam0 规则按 LLD §5.1；禁止仅构建 StereoRig 两目。V05/V07。

#### F16 — src/VisualSim/StressModel.hpp

**改动前（来源真实原文，来源仓库不修改）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/VisualSim/StressModel.hpp L36–L43](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSim/StressModel.hpp#L36-L43)

```cpp
  };
  angular_pulse(4,.2*strength);angular_pulse(4.5,-.4*strength);angular_pulse(5,.2*strength);
  return m;
}
class PixelStress {
public:
  explicit PixelStress(const StressOptions &options):options_(options),random_(options.seed) {
    if(!std::isfinite(options.pixel_sigma)||options.pixel_sigma<0||!std::isfinite(options.outlier_ratio)||options.outlier_ratio<0||options.outlier_ratio>1||!std::isfinite(options.motion_strength)||options.motion_strength<0)throw std::invalid_argument("Invalid VisualSim stress parameters");
```

**迁移后的目标片段（未实施代码草案）**：

```cpp
// 目标 include/visualsim/Trajectory.hpp：v1 只有两种角运动分支。
struct AngularStationary {};
struct AngularFixedAxisRotation {
  Eigen::Vector3d axis_B;
  double rate_radps;
};
using AngularMotionSpec =
    std::variant<AngularStationary, AngularFixedAxisRotation>;
// 不加入 AngularAcceleration/PixelStress 等隐藏模式。
```

**边界与验证**：不迁 StressModel，尤其不将脉冲非零角加速度伪装成 FixedAxisConstantRotation。新模式需要 SRS/LLD 升版。V03/V04。

#### F17 — src/VisualSim/StressTests.cpp

**改动前（来源真实原文，来源仓库不修改）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/VisualSim/StressTests.cpp L1–L8](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSim/StressTests.cpp#L1-L8)

```cpp
#include "StressModel.hpp"
#include <iostream>
#include <vector>
#include <stdexcept>
using namespace FastVIO::VisualSim;
int main() {
  try {
    StressOptions options;PixelStress off(options);
```

**迁移后的目标片段（未实施代码草案）**：

```cpp
// 目标 tests/MotionHistoryTests.cpp：对允许的匀加速直线运动做导数检查。
auto expected_p = p0 + v0 * dt + 0.5 * a_W * dt * dt;
auto expected_v = v0 + a_W * dt;
Require((motion.value->position_W_m - expected_p).norm() < 1e-10,
        "analytical displacement");
Require((motion.value->velocity_W_mps - expected_v).norm() < 1e-10,
        "analytical speed");
// 不测试未批准的角加速度脉冲与像素离群 Stress 模式。
```

**边界与验证**：示意用例中的 p0/v0/a_W/dt/motion 为测试建立的局部变量；新测试不引用源 StressModel.hpp。V04。

#### F18 — src/VisualSim/VisualInertial.cpp

**改动前（来源真实原文，来源仓库不修改）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/VisualSim/VisualInertial.cpp L35–L42](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSim/VisualInertial.cpp#L35-L42)

```cpp

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include "euroc_vio/DatumFast.hpp"
#include "euroc_vio/DatumImu.hpp"
#include "euroc_vio/DatumTruth.hpp"
#include "euroc_vio/ErrorStateKalmanFilter.hpp"
```

**迁移后的目标片段（未实施代码草案）**：

```cmake
# 目标 CMakeLists.txt：只发现非 ROS 依赖。
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs)
# 按需要 find_package(Sophus CONFIG REQUIRED)
# 不执行 find_package(rclcpp)、ament_cmake 或 rosidl。
target_link_libraries(visualsim PUBLIC Eigen3::Eigen ${OpenCV_LIBS})
```

**边界与验证**：此后半片段是目标构建方案，不是修改 VisualInertial.cpp；源文件仍只读且被排除。V01。

#### F19 — src/VisualSim/VisualSim.cpp

**改动前（来源真实原文，来源仓库不修改）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/VisualSim/VisualSim.cpp L184–L191](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSim/VisualSim.cpp#L184-L191)

```cpp
    std::error_code ec;
    std::filesystem::remove_all(path_mav0_, ec);
    if (std::filesystem::create_directories(path_mav0_, ec))
    {
      std::print(stderr, "[INFO] 工作目录创建成功: {}\n",
                 std::filesystem::absolute(path_mav0_).string());
    }
    else
```

**迁移后的目标片段（未实施代码草案）**：

```cpp
// 目标 src/Simulator.cpp：所有参与传感器在最终提交前完成计算。
Result<SampleBatch> Simulator::Generate(TimestampNs ns, SensorGate gate) {
  if (!gate.camera && !gate.imu)
    return {Status::Ok, SampleBatch{}, {}};
  auto snapshot = impl_->EvaluateWithCalibration(ns);
  if (snapshot.status != Status::Ok)
    return {snapshot.status, std::nullopt, snapshot.message};
  auto staged = impl_->StageAllSensors(*snapshot.value, ns, gate);
  if (staged.status != Status::Ok)
    return {staged.status, std::nullopt, staged.message};
  impl_->CommitCountersNoexcept(*staged.value);
  return std::move(staged.value->ready_result);
}
// 必须先准备最终 Result 的所有可抛异常分配，再提交计数器。
// 只有显式 DatasetExporter 会处理文件目录，不执行 remove_all(mav0)。
```

**边界与验证**：StageAllSensors/CommitCountersNoexcept 是拟议私有事务例程；其强异常保证和返回 noexcept 条件需按 LLD §2.1 实现与故障注入验证。V07/V11。


### 3.12.3 F20–F29：CameraIO 与加载器的真实片段、迁移及排除代码

#### F20 — src/CameraIO/CameraFrameReader.cpp

**改动前（源文件实际原文，来源只读）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/CameraIO/CameraFrameReader.cpp L32–L39](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/CameraIO/CameraFrameReader.cpp#L32-L39)

```cpp

std::vector<CameraIndexRecord> ReadCameraIndex(const std::filesystem::path &directory)
{
  const auto index_path = directory / "data.csv";
  std::ifstream input{index_path};
  if (!input) throw std::runtime_error("Cannot read camera index: " + index_path.string());
  std::optional<bool> video;
  std::vector<CameraIndexRecord> records;
```

**拟议替代或迁移后片段（未实施）**：

```cpp
// 目标仅负责新生成数据的导出，不再通过读取历史图像驱动仿真。
Result<SampleBatch> Simulator::Generate(TimestampNs ns, SensorGate gate) {
  auto state = impl_->EvaluateWithCalibration(ns);
  if (state.status != Status::Ok)
    return {state.status, std::nullopt, state.message};
  return impl_->GenerateAtomic(*state.value, gate);
}
// CameraFrameReader / FFV1Decoder / ReadCameraIndex 不出现在目标运行库。
```

**排除/融合边界**：来源 Reader 不修改也不复制；目标可在独立测试中读取自己输出的 PNG，不能在生成器内缓存/回放旧帧。V01/V10。

#### F21 — src/CameraIO/CameraFrameWriter.cpp

**改动前（源文件实际原文，来源只读）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/CameraIO/CameraFrameWriter.cpp L58–L65](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/CameraIO/CameraFrameWriter.cpp#L58-L65)

```cpp

class ImageSequenceWriter final : public IndexedWriter
{
public:
  ImageSequenceWriter(const std::filesystem::path &directory, cv::Size size) :
    IndexedWriter{directory, size, "#timestamp [ns],filename"}
  {
    std::filesystem::create_directories(directory_ / "data");
```

**拟议替代或迁移后片段（未实施）**：

```cpp
// 目标 src/DatasetExporter.cpp：仅被显式导出 API 调用。
Result<void> WriteCameraPng(const std::filesystem::path& output,
                            const SensorFrame& frame) {
  if (std::filesystem::exists(output))
    return {Status::InvalidArgument, "output already exists"};
  const cv::Mat& img = std::get<cv::Mat>(frame.payload);
  if (img.type() != CV_8UC1)
    return {Status::InvalidArgument, "expected grayscale image"};
  if (!cv::imwrite(output.string(), img))
    return {Status::InternalError, "PNG encoding failed"};
  return {Status::Ok, {}};
}
// CSV 行必须使用 std::to_string(frame.timestamp_ns) 原值。
```

**排除/融合边界**：不得复制该文件的 VideoWriter/FFV1 分支；完整多文件导出还须设计暂存与提交，不能因中途写失败覆盖已有文件。V10/V11。

#### F22 — src/CameraIO/FFV1Video.hpp

**改动前（源文件实际原文，来源只读）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/CameraIO/FFV1Video.hpp L9–L16](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/CameraIO/FFV1Video.hpp#L9-L16)

```cpp
{
class FFV1Encoder
{
public:
  FFV1Encoder(const std::filesystem::path &path, cv::Size size, double fps);
  ~FFV1Encoder();
  void Write(const cv::Mat &image);
  void Close();
```

**拟议替代或迁移后片段（未实施）**：

```cmake
# 目标仅链接成像/PNG 所需 OpenCV 组件，FFV1 头不安装。
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs)
install(DIRECTORY include/visualsim/ DESTINATION include/visualsim)
# 不安装 FFV1Video.hpp，不导出 FFV1Encoder/Decoder。
```

**排除/融合边界**：来源 FFV1Video.hpp 保持只读；如需视频另提需求，不能污染 v1 API。V01。

#### F23 — src/CameraIO/FFV1Video.cpp

**改动前（源文件实际原文，来源只读）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/CameraIO/FFV1Video.cpp L10–L17](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/CameraIO/FFV1Video.cpp#L10-L17)

```cpp
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/mathematics.h>
#include <libavutil/opt.h>
}

```

**拟议替代或迁移后片段（未实施）**：

```cmake
# 目标 libvisualsim 的源与依赖白名单（不含 FFV1Video.cpp）。
add_library(visualsim SHARED
  src/Simulator.cpp
  src/CameraRenderer.cpp
  src/RoomRenderer.cpp
  src/ImuModel.cpp)
target_link_libraries(visualsim PUBLIC Eigen3::Eigen ${OpenCV_LIBS})
# 无 libavcodec/libavformat/libavutil、CameraIO 子目录。
```

**排除/融合边界**：不是删来源文件，且目标并不因为需要 PNG 就引入 libav*。V01 ELF NEEDED 检查。

#### F24 — src/CameraIO/CameraFrameIOTests.cpp

**改动前（源文件实际原文，来源只读）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/CameraIO/CameraFrameIOTests.cpp L1–L8](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/CameraIO/CameraFrameIOTests.cpp#L1-L8)

```cpp
#include "euroc_vio/CameraFrameIO.hpp"
#include "../VisualSlam/ImageDataLoader.hpp"

#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
```

**拟议替代或迁移后片段（未实施）**：

```cpp
// 目标 tests/ExportTests.cpp：只针对本库显式生成的 PNG/CSV。
const auto expected_ns = std::int64_t{123456789};
auto frames = simulator.Generate(expected_ns, {.camera = true, .imu = false});
Require(frames.status == Status::Ok, "generate failed");
auto decoded = cv::imread(saved_png.string(), cv::IMREAD_UNCHANGED);
Require(decoded.type() == CV_8UC1, "must retain CV_8UC1");
Require(csv_timestamp_ns == expected_ns, "no re-derived timestamp");
// 不引用 VisualSlam/ImageDataLoader、FFV1/libav*。
```

**排除/融合边界**：`simulator/saved_png/csv_timestamp_ns` 是该未来测试场景准备的对象；禁止复制源测试的 FFmpeg/VisualSlam 部分。V10。

#### F25 — include/euroc_vio/CameraFrameIO.hpp

**改动前（源文件实际原文，来源只读）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f include/euroc_vio/CameraFrameIO.hpp L13–L20](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/include/euroc_vio/CameraFrameIO.hpp#L13-L20)

```cpp
{
enum class CameraOutputMode { Video, Images };

// Storage references contain no sensor time. Only the CSV supplies timestamp_ns.
struct CameraImageReference
{
  std::filesystem::path file;
  std::optional<std::uint64_t> frame_index;
```

**拟议替代或迁移后片段（未实施）**：

```cpp
// 目标 include/visualsim/DatasetExporter.hpp：v1 只支持显式 PNG/CSV。
struct DatasetExportOptions {
  std::filesystem::path output_root;
  bool allow_replace = false;
};
Result<void> ExportDataset(const DatasetExportOptions& options,
                           const SampleBatch& frames);
// 不导出 CameraImageReader、CameraOutputMode::Video 或 FFV1Encoder。
```

**排除/融合边界**：输出调用必须是显式 API；真正 exporter 要在写入前校验全部路径和帧，并保留 int64 时间。V01/V10。

#### F26 — include/euroc_vio/AbstractLoader.hpp

**改动前（源文件实际原文，来源只读）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f include/euroc_vio/AbstractLoader.hpp L13–L20](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/include/euroc_vio/AbstractLoader.hpp#L13-L20)

```cpp

struct AbstractLoader
{
  static std::string_view trim(std::string_view str)
  {
    const auto first = str.find_first_not_of(" \t\n\r\v\f");
    if (first == std::string_view::npos)
    {
```

**拟议替代或迁移后片段（未实施）**：

```cpp
// 目标 src/MotionConfigLoader.cpp：配置按 LLD §9.1 原子导入。
Result<void> Simulator::LoadMotionConfig(
    const std::filesystem::path& config) {
  if (impl_->HasExplicitEventsOrSamples())
    return {Status::EventHistoryFrozen, "instance already used"};
  auto pending = ParseAndValidateV1Yaml(config);
  if (pending.status != Status::Ok)
    return {pending.status, pending.message};
  return impl_->CommitTimelineNoexcept(std::move(*pending.value));
}
// 不继承 AbstractLoader，不接受混合历史输出作为运动真值。
```

**排除/融合边界**：Parser/Commit helpers 为必须实现的目标私有函数；实例状态先于文件解析判断，错误不提交半个事件。V02/V11。

#### F27 — include/euroc_vio/DatumFast.hpp

**改动前（源文件实际原文，来源只读）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f include/euroc_vio/DatumFast.hpp L22–L29](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/include/euroc_vio/DatumFast.hpp#L22-L29)

```cpp

struct DatumFast
{
  std::int64_t timestamp_;
  // 角位移向量
  Eigen::Vector3d angular_displacement_;
  // 单位化平移向量
  Eigen::Vector3d normalized_translation_;
```

**拟议替代或迁移后片段（未实施）**：

```cpp
// 目标 include/visualsim/Types.hpp：解析真值不是来源估计器 DatumFast。
struct MotionState {
  Eigen::Vector3d position_W_m;
  Eigen::Matrix3d rotation_WB;
  Eigen::Vector3d velocity_W_mps;
  Eigen::Vector3d acceleration_W_mps2;
  Eigen::Vector3d angular_velocity_B_radps;
  Eigen::Vector3d angular_acceleration_B_radps2;
  bool derivative_discontinuous;
};
```

**排除/融合边界**：此处摘录冻结的 LLD §2 字段，不从 DatumFast 继承，也不链接 AbstractLoader。V04。

#### F28 — include/euroc_vio/DatumImu.hpp

**改动前（源文件实际原文，来源只读）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f include/euroc_vio/DatumImu.hpp L21–L28](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/include/euroc_vio/DatumImu.hpp#L21-L28)

```cpp

struct DatumImu
{
  std::int64_t timestamp_;
  Eigen::Vector3d angular_velocity_;
  Eigen::Vector3d linear_acceleration_;

  static std::vector<DatumImu>
```

**拟议替代或迁移后片段（未实施）**：

```cpp
// 目标 include/visualsim/Types.hpp：IMU 当次观测直接值类型。
struct ImuMeasurement {
  Eigen::Vector3d angular_velocity_S_radps;
  Eigen::Vector3d specific_force_S_mps2;
};
Result<SampleBatch> Simulator::Generate(TimestampNs at_ns, SensorGate gate);
// 本库不接收 DatumImu 的历史积分/CSV 索引来替代物理真值。
```

**排除/融合边界**：必须匹配世界重力、T_BS、刚性杆臂计算公式而非预积分结果；V06-D。

#### F29 — include/euroc_vio/DatumTruth.hpp

**改动前（源文件实际原文，来源只读）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f include/euroc_vio/DatumTruth.hpp L21–L28](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/include/euroc_vio/DatumTruth.hpp#L21-L28)

```cpp

struct DatumTruth
{
  std::int64_t timestamp_;
  Eigen::Vector3d position_;
  Eigen::Quaterniond attitude_;
  Eigen::Vector3d velocity_;
  Eigen::Vector3d bias_gyro_;
```

**拟议替代或迁移后片段（未实施）**：

```cpp
// 目标：任何时刻的真值都按事件历史重算，不读取旧 DatumTruth。
auto truth = simulator.GetMotionState(query_ns);
if (truth.status != Status::Ok) return truth.status;
const auto& p_W = truth.value->position_W_m;
const auto& R_WB = truth.value->rotation_WB;
// 导出时只写本次求值结果，不存储真值序列或帧位姿缓存。
```

**排除/融合边界**：此处局部 `simulator/query_ns` 由未来 caller 提供；源文件不迁移。V04/V09。


### 3.12.4 F30–F38：估计器与 ROS 关联头的真实片段、替代及排除

#### F30 — include/euroc_vio/ErrorStateKalmanFilter.hpp

**改动前（真实来源代码）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f include/euroc_vio/ErrorStateKalmanFilter.hpp L39–L46](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/include/euroc_vio/ErrorStateKalmanFilter.hpp#L39-L46)

```cpp
template <typename value_type = double>
class ErrorStateKalmanFilter
{
#pragma region PUBLIC_TYPE

public:
  using Vector2  = Eigen::Vector<value_type, 2>;
  using Vector3  = Eigen::Vector<value_type, 3>;
```

**计划迁移后/排除后的目标片段（尚未实施）**：

```cmake
# 目标 CMake：仿真共享库不引入 ESKF 及其历史缓冲。
target_sources(visualsim PRIVATE
  src/Simulator.cpp src/MotionHistory.cpp
  src/CameraRenderer.cpp src/ImuModel.cpp
  src/RoomRenderer.cpp src/RayCaster.cpp)
# 不添加 ErrorStateKalmanFilter.hpp / msckf.cpp / ZUPT.hpp。
```

**决策**：不迁入 ESKF 的状态/landmark/map/history；目标是传感器数据生产者，可供下游估计器调用。V01/V09。

#### F31 — include/euroc_vio/SensorState.hpp

**改动前（真实来源代码）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f include/euroc_vio/SensorState.hpp L31–L38](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/include/euroc_vio/SensorState.hpp#L31-L38)

```cpp
template <typename value_type>
struct ImuState : public std::array<value_type, 10>
{
  ImuState() :
    std::array<value_type, 10>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0}
  {
  }

```

**计划迁移后/排除后的目标片段（尚未实施）**：

```cpp
// 目标 MotionState 中坐标和时间语义按 LLD v1 保持单一。
struct MotionState {
  Eigen::Vector3d position_W_m;
  Eigen::Matrix3d rotation_WB;
  Eigen::Vector3d velocity_W_mps;
  Eigen::Vector3d acceleration_W_mps2;
  Eigen::Vector3d angular_velocity_B_radps;
  Eigen::Vector3d angular_acceleration_B_radps2;
  bool derivative_discontinuous;
};
```

**决策**：不复制估计器数组 ImuState/ImuDerivative 来代替解析运动真值。V04/V06。

#### F32 — include/euroc_vio/SensorYaml.hpp

**改动前（真实来源代码）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f include/euroc_vio/SensorYaml.hpp L26–L33](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/include/euroc_vio/SensorYaml.hpp#L26-L33)

```cpp

struct SensorYaml
{
  // 姿态, 变换矩阵
  Eigen::Matrix4d transform_matrix_{Eigen::Matrix4d::Identity()};
  // 图像分辨率 [宽, 高] (相机节点专用, 默认 0x0)
  Eigen::Vector2i resolution_{Eigen::Vector2i::Zero()};
  // 针孔内参 [fu, fv, cu, cv] (相机节点专用)
```

**计划迁移后/排除后的目标片段（尚未实施）**：

```cpp
// 目标 src/MotionConfigLoader.cpp：schema v1 是 baseline + 带整数纳秒的 events。
YAML::Node document = YAML::LoadFile(path.string());
if (!document["schema_version"] ||
    document["schema_version"].as<int>() != 1)
  return {Status::InvalidConfigSchema, "unsupported schema"};
const auto events = document["events"];
// 对每项 timestamp_ns 必须验证原 YAML 标量为十进制整数，
// 再解析 op/value；先 baseline(room/pose/imu) 再同刻 events。
// 全部验证成功后原子提交，不能复用旧 ReadSensorYaml 当运动配置。
```

**决策**：补重复键/未知字段/类型和时间验证及 EventHistoryFrozen 边界；这不是来源 SensorYaml 的原代码。V02/V11。

#### F33 — include/euroc_vio/ZUPT.hpp

**改动前（真实来源代码）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f include/euroc_vio/ZUPT.hpp L92–L99](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/include/euroc_vio/ZUPT.hpp#L92-L99)

```cpp
template <typename value_type, std::size_t WindowSize = 64>
class ZUPT
{
public:
  using chunk_type  = Eigen::Vector<value_type, 3>;
  using data_type   = std::pair<chunk_type, chunk_type>;
  using window_type = CircularBuffer<data_type, WindowSize>;

```

**计划迁移后/排除后的目标片段（尚未实施）**：

```cpp
// 目标静止模式必须来自用户配置，而不是 IMU/ZUPT 推断。
Result<void> Simulator::SetLinearMotion(TimestampNs ns,
                                         const LinearMotionSpec& mode) {
  return impl_->AppendValidatedEvent(ns, SetLinearEvent{mode});
}
auto ok = simulator.SetLinearMotion(
    2'000'000'000LL, LinearStationary{});
// 不引入 ZUPT 阈值、缓存窗口或自动切换 motion mode。
```

**决策**：旧 ZUPT 与 CircularBuffer 排除；`simulator` 为未来调用者局部对象。V03/V09。

#### F34 — include/euroc_vio/StereoObservation.hpp

**改动前（真实来源代码）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f include/euroc_vio/StereoObservation.hpp L10–L17](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/include/euroc_vio/StereoObservation.hpp#L10-L17)

```cpp
template <typename value_type = double>
struct StereoObservation
{
  using Vector2 = Eigen::Vector<value_type, 2>;
  using Vector3 = Eigen::Vector<value_type, 3>;

  // 路标点 ID
  std::uint32_t feature_id_{0};
```

**计划迁移后/排除后的目标片段（尚未实施）**：

```cpp
// 目标 SensorFrame 不固定为两个相机的匹配点列表。
struct SensorFrame {
  TimestampNs timestamp_ns;
  SensorId sensor_id;
  std::string sensor_name;  // cam0、cam1、调用者自定义名
  std::uint64_t frame_number;
  std::variant<cv::Mat, ImuMeasurement> payload;
};
// 一次 camera gate 输出在 at_ns 时存在的所有 CameraHandle 的图像。
```

**决策**：不迁移双目观测及 ESKF 类型；不同相机成像无共同可见点交集限制。V05/V07。

#### F35 — include/euroc_vio/CircularBuffer.hpp

**改动前（真实来源代码）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f include/euroc_vio/CircularBuffer.hpp L17–L24](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/include/euroc_vio/CircularBuffer.hpp#L17-L24)

```cpp
template <typename T, std::size_t N>
class CircularBuffer
{
  static_assert(N > 0, "Capacity must be positive");
  static_assert((N & (N - 1)) == 0,
                "Capacity must be power of two for bitmask optimization");

public:
```

**计划迁移后/排除后的目标片段（尚未实施）**：

```cpp
// 目标 Simulator::Impl：只持久保存事件和最小逐传感器协议变量。
struct SensorSequence {
  std::optional<TimestampNs> last_successful_ns = std::nullopt;
  std::uint64_t next_frame_number = 0;
};
std::vector<ChangeRecord> change_history;
std::unordered_map<SensorId, SensorSequence> frame_sequence;
// 禁止 CircularBuffer<cv::Mat>、历史 Pose/IMU/RayHit 窗口。
```

**决策**：事件日志属于用户修改历史；不得把它改造成结果缓存。V09。

#### F36 — include/euroc_vio/VisionMode.hpp

**改动前（真实来源代码）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f include/euroc_vio/VisionMode.hpp L8–L15](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/include/euroc_vio/VisionMode.hpp#L8-L15)

```cpp
// 视觉模式: 数据集相机数目
enum class VisionMode
{
  kStereo, // cam0 + cam1
  kMono    // 仅 cam0 或仅 cam1
};

// 检测数据集模式: cam1/data.csv 存在且非空 → 双目, 否则单目
```

**计划迁移后/排除后的目标片段（尚未实施）**：

```cpp
// 目标 include/visualsim/Types.hpp：生成门控与视觉估计模式无关。
struct SensorGate {
  bool camera = false; // 当时注册的全部相机
  bool imu = false;
};
const SensorGate only_imu{.camera=false, .imu=true};
auto result = simulator.Generate(at_ns, only_imu);
// 无 VisionMode、ImageDataLoader、双目读入模式依赖。
```

**决策**：`simulator/at_ns` 为未来调用者提供。仅通过冻结 SensorGate 表达门控。V07。

#### F37 — include/euroc_vio/pch.hpp

**改动前（真实来源代码）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f include/euroc_vio/pch.hpp L14–L21](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/include/euroc_vio/pch.hpp#L14-L21)

```cpp
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.hpp>
```

**计划迁移后/排除后的目标片段（尚未实施）**：

```cmake
# 目标 CMake 只导出 visualsim 公共头和非 ROS 库。
target_include_directories(visualsim PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(visualsim PUBLIC Eigen3::Eigen)
# 不执行 target_precompile_headers(... euroc_vio/pch.hpp)。
# 不含 ament、rclcpp、cv_bridge、message_filters。
```

**决策**：严格禁止将来源 PCH/common_pch 带入目标编译依赖；Linux .so 消费者无需 ROS。V01。

#### F38 — src/VisualSlam/ImageDataLoader.hpp

**改动前（真实来源代码）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f src/VisualSlam/ImageDataLoader.hpp L9–L16](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSlam/ImageDataLoader.hpp#L9-L16)

```cpp
#include <opencv2/core.hpp>
#include "euroc_vio/CameraFrameIO.hpp"
#include "euroc_vio/VisionMode.hpp"

namespace FastVIO
{
template <typename E = CameraImageReference>
struct MonocularFrame
```

**计划迁移后/排除后的目标片段（尚未实施）**：

```cpp
// 目标 tests/ExportTests.cpp 只验证新产生的 PNG，无 VisualSlam。
const cv::Mat restored =
    cv::imread(exported_image_path.string(), cv::IMREAD_UNCHANGED);
Require(!restored.empty(), "PNG not readable");
Require(restored.type() == CV_8UC1, "PNG image must be grayscale");
Require(restored.cols == camera.width &&
        restored.rows == camera.height, "wrong image resolution");
// 此处 exported_image_path/camera 为测试提供的已初始化局部对象。
```

**决策**：不复制 VisualSlam 模块或 CameraFrameReader；导出测试不依赖来源仓库。V10。


### 3.13 B01–B08：构建配置及脚本的源代码前/后对照

**所有改动前块都来自固定 GitHub 提交**；来源 ros2-vio-node 的 CMake 文件仅用于比照，不在源仓库修改。B03 同时涉及三个目标 PowerShell 脚本，后面另补 run/clean 的原文证据。

#### B01 — ncut-d1z/VisualSim/CMakeLists.txt

**改动前（真实 目标 文件）**：[ncut-d1z/VisualSim@dbfab8e6d6b7 L12–L16](https://github.com/ncut-d1z/VisualSim/blob/dbfab8e6d6b7619c72a0050c8a760d748d6e9628/CMakeLists.txt#L12-L16)

```cmake

add_executable(visual_sim src/main.cpp)
target_include_directories(visual_sim PRIVATE src ${OpenCV_INCLUDE_DIRS})
target_link_libraries(visual_sim PRIVATE Eigen3::Eigen ${OpenCV_LIBS})

```

**拟议改动后（目标仓库新代码/删除指令；未执行）**：

```cmake
cmake_minimum_required(VERSION 4.3.4)
project(VisualSim VERSION 1.0.0 LANGUAGES CXX)
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "VisualSim supports Linux only")
endif()
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs)
add_library(visualsim SHARED
  src/Simulator.cpp src/MotionHistory.cpp src/CameraRenderer.cpp
  src/RoomRenderer.cpp src/ImuModel.cpp src/RayCaster.cpp)
add_library(VisualSim::VisualSim ALIAS visualsim)
target_compile_features(visualsim PUBLIC cxx_std_23)
set_target_properties(visualsim PROPERTIES
  EXPORT_NAME VisualSim VERSION ${PROJECT_VERSION} SOVERSION 1)
target_include_directories(visualsim PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
target_link_libraries(visualsim PUBLIC Eigen3::Eigen)
target_link_libraries(visualsim PUBLIC ${OpenCV_LIBS})
install(TARGETS visualsim EXPORT VisualSimTargets)
install(EXPORT VisualSimTargets NAMESPACE VisualSim::
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/VisualSim")
# 还需生成 VisualSimConfig.cmake/Version，使用 configure_package_config_file
# 和 write_basic_package_version_file，然后 install(FILES ...)。
```

**判断**：这是包含 SHARED 核心和安装导出骨架的草案；Config.cmake 内容、Pimpl/ABI、RPATH 与依赖 `find_dependency` 必须在 P1 补全并通过外部消费者测试。

#### B02 — ncut-d1z/VisualSim/CMakePresets.json

**改动前（真实 目标 文件）**：[ncut-d1z/VisualSim@dbfab8e6d6b7 L3–L11](https://github.com/ncut-d1z/VisualSim/blob/dbfab8e6d6b7619c72a0050c8a760d748d6e9628/CMakePresets.json#L3-L11)

```json
  "configurePresets": [{
    "name": "windows",
    "displayName": "Windows x64 / vcpkg",
    "generator": "Visual Studio 18 2026",
    "architecture": "x64",
    "binaryDir": "${sourceDir}/out/build",
    "cacheVariables": {
      "CMAKE_TOOLCHAIN_FILE": "$env{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake",
      "VCPKG_TARGET_TRIPLET": "x64-windows",
```

**拟议改动后（目标仓库新代码/删除指令；未执行）**：

```json
{
  "version": 6,
  "configurePresets": [{
    "name": "ubuntu24-release",
    "generator": "Ninja",
    "binaryDir": "${sourceDir}/build",
    "cacheVariables": {
      "CMAKE_BUILD_TYPE": "Release",
      "BUILD_TESTING": "ON"
    }
  }],
  "buildPresets": [{"name": "ubuntu24-release",
                    "configurePreset": "ubuntu24-release"}],
  "testPresets": [{"name": "ubuntu24-release",
                   "configurePreset": "ubuntu24-release"}]
}
```

**判断**：明确删除 Visual Studio/x64-windows/Windows 盘符；新预设可被调用方按需覆盖编译器和安装前缀，Ninja 须存在。P1。

#### B03 — ncut-d1z/VisualSim/build.ps1

**改动前（真实 目标 文件）**：[ncut-d1z/VisualSim@dbfab8e6d6b7 L7–L11](https://github.com/ncut-d1z/VisualSim/blob/dbfab8e6d6b7619c72a0050c8a760d748d6e9628/build.ps1#L7-L11)

```powershell

    cmake --preset windows
    if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed.' }

    cmake --build --preset windows --clean-first
```

**拟议改动后（目标仓库新代码/删除指令；未执行）**：

```bash
# 目标仓库在实施 Linux-only 构建时执行；此命令目前未执行。
git rm -- build.ps1 run.ps1 clean.ps1

# Linux 运行流程由 README/CMake Presets 表达（非自动写盘）：
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
cmake --install build --prefix "$HOME/opt/VisualSim"
```

**判断**：此组删除操作影响目标仓库三个 PowerShell 文件，来源仓库没有任何修改。还需独立核验 run.ps1/clean.ps1 的 Windows 行为。

**B03 的另一份原始文件片段：** [run.ps1 L11–L17](https://github.com/ncut-d1z/VisualSim/blob/dbfab8e6d6b7619c72a0050c8a760d748d6e9628/run.ps1#L11-L17)

```powershell
        Write-Host 'visual_sim.exe was not found. Building the project first...'
        & (Join-Path $PSScriptRoot 'build.ps1')
        if ($LASTEXITCODE -ne 0) {
            throw 'build.ps1 failed.'
        }
    }

```

**B03 的另一份原始文件片段：** [clean.ps1 L20–L26](https://github.com/ncut-d1z/VisualSim/blob/dbfab8e6d6b7619c72a0050c8a760d748d6e9628/clean.ps1#L20-L26)

```powershell

  Remove-Item -LiteralPath $resolvedBuild -Recurse -Force
  Write-Host "Removed build directory: $resolvedBuild"
} finally {
  Pop-Location
}

```

#### B04 — ncut-d1z/VisualSim/vcpkg.json

**改动前（真实 目标 文件）**：[ncut-d1z/VisualSim@dbfab8e6d6b7 L4–L12](https://github.com/ncut-d1z/VisualSim/blob/dbfab8e6d6b7619c72a0050c8a760d748d6e9628/vcpkg.json#L4-L12)

```json
  "builtin-baseline": "6e856794aebd1ee877acb74c9264d9552903c6fe",
  "dependencies": [
    "eigen3",
    {
      "name": "opencv4",
      "default-features": false,
      "features": ["jpeg", "png"]
    }
  ]
```

**拟议改动后（目标仓库新代码/删除指令；未执行）**：

```cmake
# 默认目标：Linux 使用 find_package 解析非 ROS 依赖。
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs)
# 若 v1 Loader 选用 yaml-cpp，则实际编译目标应 find_package(yaml-cpp CONFIG REQUIRED)。
# 若 Linux 构建完全不使用 vcpkg，计划中的代码改动为删除 vcpkg.json；
# 如果保留 Linux vcpkg，只能作为可选路径，禁止 x64-windows triplet。
```

**判断**：此文件后续保留/删除的实际决策须由 P0 依赖取证支持；不可因删掉 Windows preset 误认为已经完成 Linux 编译。

#### B05 — ncut-d1z/VisualSim/README.md

**改动前（真实 目标 文件）**：[ncut-d1z/VisualSim@dbfab8e6d6b7 L141–L147](https://github.com/ncut-d1z/VisualSim/blob/dbfab8e6d6b7619c72a0050c8a760d748d6e9628/README.md#L141-L147)

```powershell

### Windows 11 + CMake + vcpkg

The Windows build environment follows
`ncut-d1z/vis-spherical-flow`:

- Windows 11 x64
```

**拟议改动后（目标仓库新代码/删除指令；未执行）**：

```markdown
# README 中的 Ubuntu 24.04 Linux-only 使用示例（实施后方可执行）。
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/opt/VisualSim"
cmake --build build -j
ctest --test-dir build --output-on-failure
cmake --install build
# 下游工程：
cmake -S consumer -B consumer-build \
  -DCMAKE_PREFIX_PATH="$HOME/opt/VisualSim"
cmake --build consumer-build -j
```

**判断**：实施时删除 README 的 Windows 11/PowerShell/vcpkg x64 教程，新增按 YAML 定义轨迹、动态 camX、Generate/Raycast 的完整示例，并注明禁隐式数据覆盖。

#### B06 — ncut-d1z/ros2-vio-node/CMakeLists.txt

**改动前（真实 来源只读 文件）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f L28–L35](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/CMakeLists.txt#L28-L35)

```cmake
# --- 查找依赖 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
```

**拟议改动后（目标仓库新代码/删除指令；未执行）**：

```cmake
# 只修改目标 VisualSim/CMakeLists.txt：不复用来源 ROS 主工程。
project(VisualSim LANGUAGES CXX)
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs)
# 不调用 ament_package()、ament_target_dependencies()；
# 不 find_package(rclcpp/rosidl/cv_bridge)。
add_library(visualsim SHARED src/Simulator.cpp src/MotionHistory.cpp)
```

**判断**：来源顶层 CMake 只读；新项目不使用 ROS workspace 作为构建或安装前提。V01。

#### B07 — ncut-d1z/ros2-vio-node/src/VisualSim/CMakeLists.txt

**改动前（真实 来源只读 文件）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f L7–L14](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/VisualSim/CMakeLists.txt#L7-L14)

```cmake
find_package(OpenCV REQUIRED)
add_subdirectory(../CameraIO ${CMAKE_CURRENT_BINARY_DIR}/camera_io)
add_executable(VisualSim VisualSim.cpp)
target_compile_features(VisualSim PRIVATE cxx_std_23)
target_include_directories(VisualSim PRIVATE ../../include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(VisualSim PRIVATE camera_frame_io Eigen3::Eigen Sophus::Sophus ${OpenCV_LIBS})
if(BUILD_TESTING)
  add_executable(VisualSimStressTests StressTests.cpp)
```

**拟议改动后（目标仓库新代码/删除指令；未执行）**：

```cmake
# 只在目标工程定义安装式共享库，而非复制 standalone 可执行程序。
add_library(visualsim SHARED
  src/Simulator.cpp
  src/RoomRenderer.cpp
  src/CameraRenderer.cpp
  src/ImuModel.cpp)
target_compile_features(visualsim PUBLIC cxx_std_23)
target_link_libraries(visualsim PUBLIC Eigen3::Eigen)
# 不添加 ../CameraIO、VisualSimStressTests 或 FFmpeg。
```

**判断**：CMake target 新增前需完整补齐内部源文件及导出安装规则；来源的 cxx_std_23 仅作为真实语言需求线索。

#### B08 — ncut-d1z/ros2-vio-node/src/CameraIO/CMakeLists.txt

**改动前（真实 来源只读 文件）**：[ncut-d1z/ros2-vio-node@fd1e08174d6f L13–L20](https://github.com/ncut-d1z/ros2-vio-node/blob/fd1e08174d6f8e1cc398aa7ffcb87d1ed97edb03/src/CameraIO/CMakeLists.txt#L13-L20)

```cmake
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(CAMERA_FFMPEG REQUIRED IMPORTED_TARGET
    libavcodec
    libavformat
    libavutil)
  set(CAMERA_FFMPEG_TARGET PkgConfig::CAMERA_FFMPEG)
else()
  # Native Windows SDK fallback (set CMAKE_PREFIX_PATH to the FFmpeg SDK).
```

**拟议改动后（目标仓库新代码/删除指令；未执行）**：

```cmake
# 目标使用 OpenCV PNG/图像依赖，不链接来源 CameraIO/libav。
find_package(OpenCV CONFIG REQUIRED COMPONENTS core imgproc imgcodecs)
target_link_libraries(visualsim PUBLIC ${OpenCV_LIBS})
# 不使用 PkgConfig::CAMERA_FFMPEG 或 add_subdirectory(src/CameraIO)。
# 使用显式 DatasetExporter 写 PNG/CSV，不提供 FFV1 视频输出。
```

**判断**：来源 src/CameraIO/CMakeLists.txt 保持原样；目标需在 P1/V01 的 ELF NEEDED 中核对没有 FFmpeg 的强制引入。


### 3.14 新增运行时内核的进一步代码草案

F01–F38 给出了每个**已有文件**的真实片段和目标替代方案。但即时式运动栈、连续 Room 求交、Brownian-bridge 随机过程在两个原仓库中均**没有现成的符合 v1 的实现**。因此它们的“修改前”不是某个虚构函数，而是 F02/F12、F01/F14、F09 已举证的旧接口。下列示例作为**未来新增模块的代码骨架**，尚未编译，必须以 LLD v1 的公开签名、错误顺序和强状态保证为准。

#### N01 事件入栈、覆盖、出栈

修改前：目标只存在 Trajectory::pose(double)，来源 PathManager 只支持固定路径拼接（F02/F12）。

计划在 src/MotionHistory.cpp 实现：

~~~cpp
struct ActiveModes {
  LinearMotionSpec linear = LinearStationary{};
  AngularMotionSpec angular = AngularStationary{};
};
using ScopeStack = std::vector<ActiveModes>; // Evaluate 调用中的临时重放状态

Result<void> ApplyMotionEvent(ActiveModes& active, ScopeStack& stack,
                              const MotionEvent& event,
                              const MotionState& at_event) {
  if (event.op == MotionOp::Begin) {
    stack.push_back(active);
  } else if (event.op == MotionOp::End) {
    if (stack.empty())
      return {Status::UnmatchedMotionScope, "unmatched EndMotionEnd"};
    const ActiveModes restored = stack.back();
    auto check = ValidateCircularReactivation(at_event, restored.linear);
    if (check.status != Status::Ok) return check;
    active = restored;  // 恢复模式参数，不倒退当前位置和朝向
    stack.pop_back();
  } else if (event.op == MotionOp::SetLinear) {
    active.linear = event.linear;   // 同类覆盖
  } else if (event.op == MotionOp::SetAngular) {
    active.angular = event.angular; // 线运动不受影响
  }
  return {Status::Ok, {}};
}
~~~

这里的 MotionEvent/MotionOp、ValidateCircularReactivation 是**待补齐的内部结构和函数**；事件提交时还须保证全部验证成功再追加历史。重算时每到事件时间先解析积分上一段，再应用该时刻按 insertion_order 排序的所有事件，最后积分到查询时间。不得缓存已求过的姿态。V02–V04。

#### N02 完整的 Room 六面求交内核

修改前：两个 Room.hpp 都只有六面离散格点，没有任意真实方向的连续面求交（F01/F14）。

计划在 src/RayCaster.cpp 实现：

~~~cpp
struct Candidate {
  Eigen::Vector3d point_W_m;
  double range_m;
  unsigned face_id;
};
std::optional<Candidate> FirstHitRoom(
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& direction,
    const RoomConfig& room) {
  const Eigen::Vector3d dims{room.depth_m, room.width_m, room.height_m};
  const double diag = dims.norm();
  const double eps_distance = std::max(1e-9, diag * 1e-12);
  const double eps_boundary = diag * 1e-12;
  std::optional<Candidate> best;
  for (unsigned face = 0; face < 6; ++face) {
    const int axis = static_cast<int>(face / 2); // x0,xD,y0,yW,z0,zH
    if (std::abs(direction[axis]) <= 1e-12) continue;
    const double plane = (face % 2 == 0) ? 0.0 : dims[axis];
    const double lambda = (plane - origin[axis]) / direction[axis];
    if (!(lambda > eps_distance)) continue;
    const Eigen::Vector3d p = origin + lambda * direction;
    bool inside = true;
    for (int j = 0; j < 3; ++j) {
      if (j == axis) continue;
      inside &= p[j] >= -eps_boundary &&
                p[j] <= dims[j] + eps_boundary;
    }
    if (inside && (!best || lambda < best->range_m))
      best = Candidate{p, lambda, face}; // 等距时保留先遍历的面
  }
  return best;
}
~~~

调用前 Raycast 还必须依 LLD §7 的整体/逐条错误优先级检查：时间、相机身份、单位向量、正 z、实际畸变后成像视域。空 bearing 向量经整体校验后成功返回空结果。N02 是几何核，不是完整的 Raycast 公共方法。V08。

#### N03 相机径向切向畸变与外参方向

修改前：来源 Camera.hpp 只有针孔 K、T_SB 投影（F05）。

计划在 src/CameraRenderer.cpp 新增：

~~~cpp
Eigen::Vector2d DistortRadtan(
    double x, double y, const std::array<double, 4>& c) {
  const double r2 = x*x + y*y;
  const double radial = 1.0 + c[0]*r2 + c[1]*r2*r2;
  return {
    x*radial + 2.0*c[2]*x*y + c[3]*(r2 + 2.0*x*x),
    y*radial + c[2]*(r2 + 2.0*y*y) + 2.0*c[3]*x*y
  };
}
// 目标 T_BC 是相机到载体；与来源 sensor_from_body_(T_SB) 方向相反。
const Eigen::Isometry3d T_WC = T_WB * camera.T_BC;
const Eigen::Vector3d X_C = T_WC.inverse() * X_W;
if (X_C.z() > 0.0) {
  const Eigen::Vector2d q =
      DistortRadtan(X_C.x()/X_C.z(), X_C.y()/X_C.z(), camera.radtan);
  const Eigen::Vector3d pixel_h =
      camera.K * Eigen::Vector3d{q.x(),q.y(),1.0};
}
~~~

T_WB、X_W、camera 在此为调用方临时变量；真实实现需检查非有限值和图像范围，并按每个时刻、每个相机标定单独计算。V05/V08。

#### N04 IMU 外参杆臂与比力

修改前：目标旧 IMU 主要考虑同中心重力比力；来源加噪有状态，不符合需求（F03/F09）。

计划在 src/ImuModel.cpp 新增：

~~~cpp
ImuMeasurement IdealImu(const MotionState& motion,
                        const ImuCalibration& imu) {
  const Eigen::Matrix3d R_BS = imu.T_BS.rotation();
  const Eigen::Vector3d arm_B = imu.T_BS.translation();
  const Eigen::Vector3d omega_B = motion.angular_velocity_B_radps;
  const Eigen::Vector3d alpha_B = motion.angular_acceleration_B_radps2;
  const Eigen::Vector3d a_WS = motion.acceleration_W_mps2
      + motion.rotation_WB *
        (alpha_B.cross(arm_B) + omega_B.cross(omega_B.cross(arm_B)));
  const Eigen::Vector3d gravity_W{0.0,0.0,-9.81};
  return {
    R_BS.transpose() * omega_B,
    R_BS.transpose() * motion.rotation_WB.transpose() * (a_WS - gravity_W)
  };
}
~~~

默认不叠加随机噪声；启用时才在单次结果上按 LLD §6.2 加白噪声/偏置，所有计算使用查询时刻有效的 IMU 标定。V06-D。

#### N05 Brownian-bridge 的时间寻址骨架

修改前：来源 ImuNoiseModel.hpp 在每次 Corrupt 调用时将 gyro_bias_ 累加，且通过固定 sqrt_rate_ 离散白噪。无法随机访问历史时刻（F09）。

计划保留完整整数纳秒时间键，并采用已冻结的 SplitMix64/Box–Muller 流：

~~~cpp
std::uint64_t SplitMix64(std::uint64_t x) {
  x += 0x9E3779B97F4A7C15ULL;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
  return x ^ (x >> 31);
}
// GaussianForRoot 与 GaussianForNode 按 LLD §6.2 的固定域标签、
// seed、epoch、轴、节点相对起点和层级混合键，并执行 Box–Muller。
// 下方不缓存树节点或各时间偏置：
double BrownianAt(std::uint64_t relative_ns, std::uint64_t seed,
                  std::uint64_t epoch, unsigned axis) {
  std::uint64_t start = 0;
  std::uint64_t width = (1ULL << 63);
  double delta = std::sqrt(static_cast<double>(width)*1e-9)
      * GaussianForRoot(seed, epoch, axis);
  double accumulated = 0.0;
  while (width > 1) {
    const std::uint64_t half = width / 2;
    const double left = delta / 2.0
        + std::sqrt(static_cast<double>(width)*1e-9/4.0)
          * GaussianForNode(seed, epoch, axis, start, width);
    if (relative_ns >= start + half) {
      accumulated += left;
      start += half;
      delta -= left;
    } else {
      delta = left;
    }
    width = half;
  }
  return accumulated;
}
~~~

实施时必须补足 GaussianForRoot/GaussianForNode 的**全部冻结键字段与高斯比特流**，在调用前检查 epoch_ns <= query_ns 且不发生 int64 溢出，并对极值时间、8 种子、4 组卡方统计执行 V06-D/V06-S。此代码只是 Brownian 递归核心，不能据此宣布噪声模块已完成。

#### N06 逐传感器时序与强异常保证

修改前：两个旧程序均按浮点秒循环生成 CSV，并不支持 v1 对每台相机/imu0 的协议状态原子提交（F03/F19）。

计划新增明确的逐流控制结构：

~~~cpp
struct SensorSequence {
  std::optional<TimestampNs> last_successful_ns = std::nullopt;
  std::uint64_t next_frame_number = 0;
};
Status CheckTimestamp(const SensorSequence& s, TimestampNs t) {
  if (!s.last_successful_ns) return Status::Ok;
  if (t == *s.last_successful_ns) return Status::DuplicateTimestamp;
  if (t <  *s.last_successful_ns) return Status::OutOfOrderTimestamp;
  return Status::Ok;
}
// Generate 批次：先解析时间线得到 at_ns 有效的相机集合。
// 全部成员时间校验、帧图像分配、IMU 计算、Result 组装完成以后，
// 再执行不可抛异常的状态提交；部分失败不得消耗任何一个帧号。
~~~

ResetSamplingSession 将所有注册传感器（包括曾用的虚拟默认 cam0）恢复为 nullopt 和 0；运动/标定修改历史不清除。V07/V11。

#### N07 YAML 严格时间戳读取

修改前：来源 SensorYaml.hpp 只解析单个 EuRoC 标定文件，而不是事件时间线（F32）。

拟在 src/MotionConfigLoader.cpp 中实现：

~~~cpp
Result<TimestampNs> ParseTimestampNs(const YAML::Node& node) {
  if (!node || !node.IsScalar())
    return {Status::InvalidConfigSchema, std::nullopt, "ns scalar required"};
  const std::string s = node.Scalar();
  if (s.empty() || !std::all_of(s.begin(), s.end(),
         [](unsigned char c) { return c >= '0' && c <= '9'; }))
    return {Status::InvalidConfigSchema, std::nullopt, "decimal integer ns"};
  TimestampNs t = 0;
  const auto [end, ec] = std::from_chars(s.data(), s.data()+s.size(), t);
  if (ec != std::errc{} || end != s.data()+s.size())
    return {Status::InvalidConfigSchema, std::nullopt, "ns overflow"};
  return {Status::Ok, t, {}};
}
// LoadMotionConfig 必须首先检查实例未使用，否则 EventHistoryFrozen。
// 配置按 baseline(room→pose→imu) 先于同刻 events 的顺序暂存；
// 检查所有 schema、字段和作用域，成功后原子提交。
~~~

此段只覆盖时间字段解析；不能代替 LLD §9.1 对重复键、未知键、矩阵、四元数和全事务的完整校验。V02/V11。

#### N08 安装式 CMake Package Config

修改前：目标仅有 add_executable(visual_sim src/main.cpp)，没有安装式 export（B01）。

未来新增 cmake/VisualSimConfig.cmake.in：

~~~cmake
@PACKAGE_INIT@
include(CMakeFindDependencyMacro)
find_dependency(Eigen3 CONFIG REQUIRED)
find_dependency(OpenCV CONFIG REQUIRED)
include("${CMAKE_CURRENT_LIST_DIR}/VisualSimTargets.cmake")
check_required_components(VisualSim)
~~~

并在目标 CMakeLists.txt 中生成/安装：

~~~cmake
configure_package_config_file(
  cmake/VisualSimConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/VisualSimConfig.cmake"
  INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/VisualSim")
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/VisualSimConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}" COMPATIBILITY SameMajorVersion)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/VisualSimConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/VisualSimConfigVersion.cmake"
  DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/VisualSim")
~~~

由于公开 API 暴露 cv::Mat，OpenCV 必须由安装包向下游暴露为可解析依赖；其他 PUBLIC 链接依赖也必须具有相应 find_dependency，必须按最终 target_link_libraries 的 PUBLIC/PRIVATE 边界核查，不可凭该示例就声称已完成 V01。


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

> **实机取证唯一归档**：[BUILD_ENVIRONMENT.md](BUILD_ENVIRONMENT.md)（E001–E016）。该文件完整保留**用户在连续两轮对话中实际贴出的命令及原始输出**，包括首次 find 语法失败、修正后 Config 搜索、标准库 realpath、ldconfig 与 GLIBCXX 信息。下方版本表与建议命令仅作取证索引/后续工作说明，**不要把它们当成用户已经执行的全部结果**。日后提出新的环境检查命令，应先追加到 BUILD_ENVIRONMENT.md 的 E017、E018……“待执行”登记，再将用户实际返回的输出原样归档，不在本计划多处复制控制台输出。



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
