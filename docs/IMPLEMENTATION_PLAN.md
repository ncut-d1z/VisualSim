# VisualSim 独立运行时库实施计划（IMPLEMENTATION_PLAN）

> **状态：v1 文档设计基线（经本轮审计整改后冻结），工程实施 P0–P6 尚未开展。** 本文件专门管理工程实施和源码融合。功能需求见 [SRS.md](SRS.md)，概要架构见 [HLD.md](HLD.md)，API/算法合同见 [LLD.md](LLD.md)。文档审计不代表生产代码实现或编译/测试通过。

## 1. 仓库写入边界与代码基线

| 角色 | 仓库 | 权限与用途 |
| --- | --- | --- |
| **唯一目标仓库** | https://github.com/ncut-d1z/VisualSim | **允许后续修改源码、CMake、示例及 docs，并交付独立库** |
| **仅供读取的来源仓库** | https://github.com/ncut-d1z/ros2-vio-node/tree/main/src/VisualSim | **只读，用于审查并把适用新代码融合入目标仓库。禁止任何提交、推送、创建 PR 或文件更改** |

历史参考基线：目标仓库四份文档修订提交 69d772719aca217456ac936ae7718cdbdc4afb5c；只读来源仓库 main f1cb7a02587199ce2dfc588789f044ccae2945ed。审计提交及当前文档状态见 [AUDIT_PROGRESS.md](AUDIT_PROGRESS.md)。P0 迁移开始前必须重新读取两个仓库的当前 HEAD、复核差异并记录具体版本。

源仓库中的 ROS 相关组件只可参考。任何文件若有 ROS/ament/rclcpp/cv_bridge 或与 ROS 消息相关的调用，不允许未经解耦直接移到独立库。只允许把与独立仿真相关的新实现整合到**目标仓库**中。

## 2. 生效需求、设计合同与审计状态（引用而不复制）

- **唯一产品需求来源**：[SRS.md](SRS.md) 的 FR-M/FR-S/FR-G/FR-C、NFR-01..07 与 AC-01..12；其中 Ubuntu 24.04、Linux-only、无 ROS、只限 Room、仅用户修改历史、0..N 相机、严格递增时间等均为已批准的边界。
- **架构决策来源**：[HLD.md](HLD.md) 的历史解析器、两类模式与作用域、动态相机注册、门控生成、无计算结果缓存和风险权衡。
- **实现合同来源**：[LLD.md](LLD.md) 的 API v1、Result/错误、时间与边界、相机/IMU 模型、Brownian-bridge 噪声、YAML schema v1 与 ABI/生命周期。
- **审计依据与实际状态**：[AUDIT.md](AUDIT.md) 和 [AUDIT_PROGRESS.md](AUDIT_PROGRESS.md)，两文件当前在 docs/ 中可访问；审计/文档整改状态与代码开发、测试结果分别记录，不互相替代。**若 P0 所读取的版本缺少审计文件或链接不可用，暂以 SRS/HLD/LLD/本计划和两个仓库当时的 HEAD 为只读基线，并记录缺失，先恢复审计文件再宣称审计闭环。**

本计划**不重新定义**已批准的运动模式、坐标方向、校准公式或噪声算法；如发现设计冲突，应先在对应规范文件审计，再更新本计划任务。本轮第三轮审计对**尚未发行的软件 v1 设计**完善默认标定、名称冲突、布朗桥边界等遗漏，不代表任何现存二进制 ABI 已被破坏；后续若实际公开发行并发生破坏性接口变化，须依 LLD 升级 API/SONAME/schema 并更新四份文档。本项目是**单人开发**，不设置人员分工、日期排期、工时审批等多人项目管理要求。性能预算与性能验收已明确推迟到首次成功运行程序之后再讨论，不属于当前必须决定的事项。

## 3. 代码比对和融合清单

### 3.1 目标仓库现有文件

| 目标当前文件 | 已有能力 | 后续处理 |
| --- | --- | --- |
| src/Trajectory.hpp | 解析前向匀速、定轴纯旋转（现有角运动未提供完整角加速度字段） | 保留数学能力，改造为事件可激活的模式规范和按需解析运动真值 |
| src/Room.hpp | 长方体房间、墙面离散网格点 | 保留并用于**唯一** Room 场景，补充真实表面求交和一致纹理 |
| src/main.cpp | 单目渲染、IMU、固定两条轨迹及 PNG/CSV/YAML 导出 | 将核心功能移入共享库；CLI 改用配置文件/函数事件定义轨迹，删除硬编码两场景 |
| CMakeLists.txt | 目前主要生成可执行文件、测试目标、包含 Windows 分支 | 增加共享库、导出/安装规则，配置期禁止 Windows，不改成低版本工具链 |
| CMakePresets.json、build.ps1、run.ps1、clean.ps1、vcpkg.json、README.md | 保留 Windows 路径、PowerShell / vcpkg 指南 | 清除/停止对 Windows 的构建支持，新增 Ubuntu CMake install 与消费者说明 |
| tests/TrajectoryTests.cpp | 原有轨迹单元测试 | 迁移为新事件/解析轨迹合同测试，保留已有有价值的运动真值断言 |

### 3.2 来源仓库中的适用新代码（**只读分析，不对源仓库写入**）

| 来源路径（相对来源仓库） | 可融合的能力 | 处理注意 |
| --- | --- | --- |
| src/VisualSim/Camera.hpp | 相机内参、外参、投影和视域判断 | 改为 0..N 相机；核对外参方向，补足实际畸变；不得只复制数据结构 |
| src/VisualSim/StereoRig.hpp | 多相机标定与成像的共同时刻关系 | 不保留固定双目类型/左右公共可见点交集约束；广义为相机集合 |
| src/VisualSim/Path.hpp | 静止、匀加速、圆周、轨迹片段拼接 | 迁移解析物理规律，改为事件驱动的双类别作用域，补全 ω/α 与历史分段 |
| src/VisualSim/Room.hpp | 长方体表面网格、可配置几何 | 融合到目标唯一 Room 模型，拒绝其他场景 |
| src/VisualSim/MeshPlot.hpp | Room 六面网格的离屏渲染/纹理构造思路 | 保留图像内容相关代码，删除 GUI 窗口/进程状态副作用 |
| src/VisualSim/VisualSim.cpp | 双目及 IMU 的采样、EuRoC 风格输出和时间戳生成 | 重构为独立库可重入 API；按需生成全部相机；禁隐式创建、覆盖或删除数据文件 |
| src/VisualSim/SensorYaml.hpp | 标定及 IMU 噪声字段 | 统一传感器时间线、T_BS 方向、真实生效；去除任何孤立固定配置 |
| src/VisualSim/Rect.hpp | 图像区域/边界判断 | 按实际需要复用；无复用价值可排除 |

不迁移 ROS 节点与消息、ESKF/VIO 算法、预积分、可视化工程管理或其他不属于独立仿真库的代码。来源中若有对 C++26/新编译器的需求，可以保留合理需求，**不得因为 Ubuntu 版本而强制降低**。

## 4. 实施阶段

| 阶段 | 任务 | 可核验交付 |
| --- | --- | --- |
| P0 — 差异审查 | 重新查看两仓库当前 HEAD；复核**已冻结的 LLD v1 event API 合同**与实际源码的差异、外参坐标方向、依赖和 Room 边界；若需破坏性变更，先提出并明确升级 API/SONAME/schema 版本且同步 SRS/HLD/LLD/PLAN，不重复执行“冻结” | 实际迁移差异和版本记录，仅在目标仓库；保持来源仓库只读 |
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
| V02 | 相同函数调用与 YAML baseline→events 序列等价；同时间事件排序稳定；未给相机标定时 YAML 和 API 均读取 §5.1 默认 K/尺寸/畸变/20 Hz | AC-02 | 未执行 |
| V03 | 同类覆盖、跨类组合、嵌套 BeginMotionMode/EndMotionEnd，出栈不使 p/R 意外跳变 | AC-03 | 未执行 |
| V04 | 静止、恒速、恒加速、圆周、纯旋转、多段拼接 p/R/v/a/ω/α 与解析公式一致 | AC-04 | 未执行 |
| V05 | 核验默认 cam0 及仅给 T_BC 的 CreateCamera 可直接按 K=(458.654,457.296,367.215,248.375)、752×480、零畸变、20 Hz、CV_8UC1 成像；默认 cam0 在已 SetCameraCalibration 或 Rename 后的首次 CreateCamera 不得覆盖它；改名 cam0→cam1 后新建相机自动跳至 cam2、SensorId/帧号不变 | AC-05 | 未执行 |
| V06-D | **确定性检查**：不同相机 K/T_BC/畸变、IMU T_BS/杆臂确实改变观测；g_W 固定为 (0,0,-9.81) m/s²、无可变重力接口；禁噪理想值、启噪复现及固定 seed 逐元素重放；布朗桥 t_epoch 的 W(0)=0、INT64_MAX 边界相对纳秒不溢出 | AC-06 | 未执行 |
| V06-S | **统计检查**：使用 LLD §6.2 冻结的 Δt=5ms、种子 0..7、各流 4096 样本、每组 n=98304/ν=98303，对 gyro/accel white 与 gyro/accel walk 四组分别要求双侧 99% 卡方范围 [97164.6270, 99448.8862] | AC-06 | 未执行 |
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
# P0 依赖/工具链冻结，保存结果到目标仓库的验收记录（不写来源仓库）
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
test -n "$LIB" && test -f "$LIB"
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
~~~

**操作检查单**：核查链接目标/运行库不含 ROS/ament，Windows 仅失败配置，依赖版本已登记；当前阶段应执行 V02–V11、V13 等功能用例，**不执行已延期的 V12 性能验收**；采样失败无部分状态提交；多次 Raycast 不产生结果缓存；数据导出需显式目标路径且不会递归删除用户目录；来源仓库未被写入。若命令失败，记录退出码与日志，回退至上一通过阶段。

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
