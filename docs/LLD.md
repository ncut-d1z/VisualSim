# VisualSim 运行时库详细设计（LLD）

> 状态：仅设计、待实现。需求以 [SRS.md](SRS.md) 为准，架构以 [HLD.md](HLD.md) 为准。所有路径为**目标仓库 ncut-d1z/VisualSim 内拟议路径**，并非现有文件承诺。来源仓库 ncut-d1z/ros2-vio-node 全程只读。
>
> 用户环境：Ubuntu 24.04，已有满足项目要求的 CMake/编译器；**禁止为“兼容”而强制降低版本**。Windows CMake 配置应失败。库/公开头文件不得依赖 ROS。

## 1. 建议的目标目录

~~~text
CMakeLists.txt                 # Linux-only; SHARED; install + export
cmake/VisualSimConfig.cmake.in
include/visualsim/
  Types.hpp                   # TimestampNs、ErrorCode、Result、MotionState
  Calibration.hpp             # CameraCalibration、ImuCalibration
  Trajectory.hpp              # IAnalyticTrajectory、内建/分段轨迹
  Scene.hpp                   # Scene、RoomScene、ForwardDepthScene
  Simulator.hpp               # 公共入口、generate、intersect、reset
src/
  Simulator.cpp
  Trajectory.cpp
  Scene.cpp
  StereoRenderer.cpp
  ImuSynth.cpp
  RayCaster.cpp
  PoseCache.cpp
  DatasetExporter.cpp
  main.cpp                    # 保留 CLI，改为库调用方
tests/
  TrajectoryTests.cpp
  CalibrationTests.cpp
  SamplingTests.cpp
  RaycastTests.cpp
  InstallConsumer/            # 在安装树外链接库的最小 CMake 工程
docs/
  SRS.md
  HLD.md
  LLD.md
~~~

模块文件名为提案，可根据真实代码组织调整；**公开 API/语义、源仓库只读、目标仓库唯一写入限制不可调整**。不迁入 ROS 类型、ament 宏或 rclcpp 依赖。

## 2. 公共类型与 API 草案（供实现细化，不代表已存在）

使用双精度计算几何，纳秒时间一律为 std::int64_t；以下接口为可实施的示意约定：

~~~cpp
namespace visualsim {
using TimestampNs = std::int64_t;
static_assert(std::is_same_v<TimestampNs, std::int64_t>);

struct SensorGate {
  bool stereo = false;
  bool imu = false;
};

struct MotionState {
  Eigen::Vector3d p_WB_m;
  Eigen::Matrix3d R_WB;
  Eigen::Vector3d v_W_mps;
  Eigen::Vector3d a_W_mps2;
  Eigen::Vector3d omega_B_radps;
  Eigen::Vector3d alpha_B_radps2;
};

struct CameraCalibration {
  int width, height;
  Eigen::Matrix3d K;
  Eigen::Isometry3d T_BC;          // C -> B
  std::array<double, 4> radtan;   // k1,k2,p1,p2
  double nominal_rate_hz;
};

struct ImuCalibration {
  Eigen::Isometry3d T_BS;         // S -> B
  double nominal_rate_hz;
  double gyro_noise_density;
  double gyro_bias_random_walk;
  double accel_noise_density;
  double accel_bias_random_walk;
  Eigen::Vector3d initial_gyro_bias;
  Eigen::Vector3d initial_accel_bias;
  bool noise_enabled = false;
  std::uint64_t random_seed;
};

class IAnalyticTrajectory {
public:
  virtual ~IAnalyticTrajectory() = default;
  virtual double durationSeconds() const = 0;
  virtual MotionState evaluate(double t_seconds) const = 0;
};

struct ImuMeasurement {
  Eigen::Vector3d angular_velocity_S_radps;
  Eigen::Vector3d specific_force_S_mps2;
};

struct SensorFrame {
  TimestampNs timestamp_ns;
  std::string sensor_name;        // cam0, cam1, imu0
  std::uint64_t frame_number;
  std::variant<cv::Mat, ImuMeasurement> payload;
};
struct SampleBatch { std::vector<SensorFrame> frames; };

enum class ErrorCode {
  Ok, InvalidArgument, InvalidCalibration, InvalidTrajectory,
  UnknownSensor, TimeOutOfRange, TimeOverflow,
  DuplicateTimestamp, OutOfOrderTimestamp,
  CacheMiss, StaleConfiguration, UnsupportedScene, NoHit,
  InternalError
};
template<class T> struct Result {
  ErrorCode code;
  std::optional<T> value;         // code == Ok 时有值
  std::string detail;
};

struct RayResult {
  std::size_t input_index;
  bool hit;
  ErrorCode reason;               // hit 时 Ok
  Eigen::Vector3d world_point_m;  // 仅 hit 时有效
  double ray_distance_m;          // 仅 hit 时有效
  std::uint32_t surface_id;       // 仅 hit 时有效
};

class Simulator {
public:
  void setTrajectory(std::shared_ptr<const IAnalyticTrajectory>);
  void setScene(std::shared_ptr<const Scene>);
  void setCameraCalibration(std::string_view id, const CameraCalibration&);
  CameraCalibration cameraCalibration(std::string_view id) const;
  void setImuCalibration(const ImuCalibration&);
  ImuCalibration imuCalibration() const;
  void setGravityWorld(const Eigen::Vector3d& g_W);
  void setCacheCapacity(std::size_t stereo_pairs); // 默认 64
  Result<SampleBatch> generate(TimestampNs time_ns, SensorGate gate);
  Result<std::vector<RayResult>> intersect(
      TimestampNs image_time_ns,
      const std::vector<Eigen::Vector3d>& bearings_C,
      std::string_view camera_id = "cam0") const;
  void reset();                    // 显式重开一个采样会话
};
} // namespace visualsim
~~~

说明：setters 验证失败可以抛出明确的 std::invalid_argument，且不得产生部分修改；generate/intersect 用显式 Result 报告运行错误。结构、类型名可在实现中微调，但以下**语义合同**必须稳定：纳秒严格 std::int64_t；输入视线确为 std::vector；帧具有传感器名/独立帧号/原始整数时间戳；拒绝重复时不分配新帧号；求交返回输入索引映射。

## 3. 时间、时序、原子性

### 3.1 纳秒与轨迹求值边界

- 以 std::int64_t 记录 input_time、frame.timestamp_ns、last_successful_timestamp_ns、cache.timestamp_ns、所有纳秒间隔。
- 仿真时间 0 ns 对应轨迹 t=0；每次调用校验时间在 [0, duration] 内（是否包含精确末端由轨迹合同明确，默认含末端）。对绝对 timestamp_ns 的范围与转换做检查。
- 计算运动学只在内部用 double t_s = static_cast<double>(time_ns) × 1e−9；不可用 t_s × 1e9 反推输入时间；跨大纳秒值差分须做溢出检查后再转秒。
- 在 C++ 类型、单元测试和安装后消费者编译中做 static_assert(std::is_same_v<TimestampNs,std::int64_t>)。API 不得出现 long、size_t、double 或非 std::int64_t 的“纳秒”字段。
- 输出数据 timestamp_ns 等于传入 time_ns，不按频率量化/四舍五入；标称 rate_hz 作为元数据和首个 IMU 噪声离散化的参照，不决定是否生成帧。

### 3.2 每传感器时间与帧号

维护独立状态：

~~~text
cam0: { last_time: optional<int64>, next_seq: uint64 }
cam1: { last_time: optional<int64>, next_seq: uint64 }
imu0: { last_time: optional<int64>, next_seq: uint64 }
~~~

初次成功输出为 seq=0；成功后 next_seq++。stereo=true 时，必须同时通过 cam0/cam1 时间校验、同刻输出且成对编号相等。单次接口不提供“只生成单独一个相机”的门控，因此两者自然同步，但内部状态仍分别保存。

若 time_ns == 某启用传感器的 last_time，则整体返回 **DuplicateTimestamp**；若小于则整体返回 **OutOfOrderTimestamp**。不允许“相同时间再次生成但获取新的帧号”，也不允许用新的 request ID 绕过重复检查。例如：

~~~text
generate(0, {stereo:true,  imu:false}) -> cam0#0, cam1#0
generate(0, {stereo:false, imu:true }) -> imu0#0   (合法)
generate(0, {stereo:true,  imu:false}) -> DuplicateTimestamp (没有新帧)
generate(5'000'000, {stereo:false, imu:true}) -> imu0#1
generate(1'000'000, {stereo:false, imu:true}) -> OutOfOrderTimestamp
generate(0, {stereo:false, imu:false}) -> 空批次，不改变任何状态
~~~

完整处理遵循**预检 → 临时计算 → 原子提交**。临时计算中复制或以事务方式暂存 PRNG/偏置状态；任一阶段异常时不更新 last_time、next_seq、缓存、噪声状态。reset 清除上述会话状态、图像缓存并按当前种子重新初始化随机流；不重置已经设置的参数、轨迹和场景。计数器溢出必须拒绝而不回绕。

### 3.3 输出顺序/内存

返回顺序固定为 cam0、cam1、imu0（按门控过滤）；每个 cv::Mat 有正确大小、明确类型（建议 CV_8UC1 默认灰度，允许扩展），为调用方独立持有的引用计数对象或深拷贝，不暴露即将重用的可写原始缓冲。IMU 数据为值语义。多实例分别持有 PRNG、缓存、序号；同一实例所有 setter/generate/reset 需串行或外部加锁。

## 4. 轨迹：解析导数、拼接与验证

### 4.1 六个物理量与一致性

设位置 p_WB(t)、旋转 R_WB(t) 为解析函数：

$$
v_W=\dot p_{WB},\quad a_W=\ddot p_{WB},\quad
[\omega_B]_\times=R_{WB}^{T}\dot R_{WB},\quad
\alpha_B=\dot\omega_B.
$$

若调用方提供 evaluate 同时返回六个量，调用方负责解析一致性，库对 SO(3) 正交/行列式、有限数、有效时间做基本检查，并提供可选的有限差分一致性诊断测试。物理模型不应仅接受任意位置、方向和速度字段而忽略互相矛盾的导数。

### 4.2 内建轨迹建议

- Stationary：全部导数 0，p/R 常量。
- ForwardConstantVelocity：p=p0+vt、R=R0；v 可沿 R0 的 +Z，兼容旧 API。
- ConstantAcceleration：p=p0+v0t+½at²、v=v0+at、a 常量。
- FixedAxisRotation：R=R0 Exp([uωt]×)、p=p0；ω_B=uω、α_B=0，支持任意归一化 u。
- Circular：p=c+u cos(Ωt)+v sin(Ωt)，其中 u、v 正交等长；线速度 Ω(−u sin+v cos)，线加速度 −Ω²(u cos+v sin)；R(t) 的 LookAtCenter/Tangent/Upward 模式按同一解析式求 ω_B、α_B，不允许只返回一个与 R(t) 不一致的固定角速度。
- Piecewise：先累计各段整数纳秒长度并校验总和/端点；在同一选定段上取局部时间，边界默认使用右侧新段（总轨迹末端例外），要求位姿连续；对未保证的导数不连续应标明边界为不适用而非给出虚假角加速度。轨迹段持续时间的浮点秒输入在配置边界规范化为整型纳秒以避免累积索引漂移。

### 4.3 轨迹参数更新

setter 更换轨迹后，之前生成的图像缓存失效；已输出帧号/last_time 不回退，若调用方想从 0 重新使用新轨迹必须显式 reset。对改变轨迹起点、朝向/运动模式及不合法分段进行清晰的状态校验。

## 5. 双目标定、成像与外参变换

### 5.1 外参真值

定义 T_BC0、T_BC1（相机到载体）和 T_BS（IMU 到载体）。W 位姿已知时：

$$
R_{WC}=R_{WB}R_{BC},\qquad
c_W=p_{WB}+R_{WB}t_{BC},\qquad
X_C=R_{WC}^{T}(X_W-c_W).
$$

相机初始方向：+X 右，+Y 下，+Z 前。默认 cam0 与 B 共址，cam1 可配置约 0.1 m 横向基线（约定为 t_BC1 的分量）；不得把来源 Camera.hpp 的 rotation_/translation_（主体→相机投影）直接当作 T_BC（相机→主体）拷贝。非单位旋转、非法 fx/fy、无效尺寸应拒绝。

### 5.2 实际径向切向畸变

归一化坐标 x=X_C/Z_C、y=Y_C/Z_C，r²=x²+y²：

$$
h=1+k_1r^2+k_2r^4,\qquad
x_d=xh+2p_1xy+p_2(r^2+2x^2),\qquad
y_d=yh+p_1(r^2+2y^2)+2p_2xy.
$$

生成像素 u=fx x_d + cx、v=fy y_d+cy（一般 K 需考虑非零 skew 时用完整 K 的齐次映射）。支持零畸变精确退化到原针孔模型。渲染稀疏路标须用畸变投影；渲染连续纹理或三角面须使用能反映畸变的反向采样/栅格映射，避免只扭曲顶点造成图像与几何不匹配。相机 FOV/视域判定使用**实际输出像素**及 Z_C>0，而非未畸变像素。

调用方在采样后可直接按各自 sensor_name 取 cam0/cam1 图像。设置 K、T_BC、radtan 后后续帧采用新参数；cam0 与 cam1 必须视差正确，非零畸变必须实测导致像素变化，且 sensor.yaml 等导出元数据与实际值一致。

## 6. IMU 运动学、标定和噪声

### 6.1 理想传感器输出

设 IMU 在 B 中固定杆臂 t_BS、姿态 R_BS，B 系瞬时角速度 ω_B、角加速度 α_B，世界系载体原点加速度 a_WB：

$$
a_{WS} = a_{WB}
 + R_{WB}\bigl(\alpha_B\times t_{BS}
 + \omega_B\times(\omega_B\times t_{BS})\bigr).
$$

$$
\omega_S=R_{BS}^T\omega_B,\qquad
f_S=R_{BS}^TR_{WB}^T(a_{WS}-g_W).
$$

结果：ω_S [rad/s] 和 f_S [m/s²]。注意**加速度计测比力**，不是 a_W；纯旋转且 t_BS 非零时仍可能有切向/向心加速度；改变 T_BS 应改变输出。依赖 α_B 的项必须实际计算。

### 6.2 随机噪声

默认为 noise_enabled=false，理想角速度/比力输出（可保留显式固定初始 bias 的单独开关；默认零）。开启后每轴使用独立零均值高斯白噪声、偏置随机游走：

$$
y_{\omega,k}=\omega_{S,k}+b_{g,k}+n_{g,k},\quad
y_{f,k}=f_{S,k}+b_{a,k}+n_{a,k}.
$$

对于与 EuRoC 噪声密度约定相符的连续模型，时间间隔 Δt>0 时白噪声离散标准差为 density/√Δt，偏置游走增量标准差为 random_walk·√Δt；具体单位及单边/双边谱密度约定应与 YAML 文档保持一致。第一帧使用 1/nominal_rate_hz 作为 Δt，后续帧使用相邻**成功生成的 IMU 时间戳差**，不能在门控跳帧时仍强行使用固定间隔。

PRNG 固定 seed → 相同采样顺序可重复。临时批次失败必须回滚随机状态；更换噪声系数/种子应重建相关噪声状态且立即影响下一帧，不能改变已经生成的历史时间和序号。修改噪声参数而 noise_enabled=false 时，理想值不受扰动；开启后参数必须进入真实计算。

## 7. 缓存与射线真值

### 7.1 缓存

缓存一组双目快照：

~~~cpp
struct StereoPoseSnapshot {
  TimestampNs timestamp_ns;
  Eigen::Vector3d center_W_cam0, center_W_cam1;
  Eigen::Matrix3d R_WC0, R_WC1;
  CameraCalibration cam0, cam1;
  std::uint64_t scene_version;
  std::uint64_t calibration_version;
};
~~~

默认 capacity=64 **组**（不是 64 单目帧）。成功 stereo 生成后按生成顺序入队，超出容量删除最旧组；key 是 exact timestamp_ns，查某 camera_id 时从对应组读正确一侧；只允许查询成功生成且尚在缓存的图像，严格匹配原生 std::int64_t，禁止 fuzzy match、四舍五入或“最近时间替代”。配置变化/场景替换后按 HLD 失效原则清空相应缓存，错误返回 CacheMiss 或 StaleConfiguration。IMU-only 不影响缓存内容。

### 7.2 射线定义和选择

输入每条 u_C 是长度约为 1 的 3D bearing，不是像素坐标。对对应图像缓存：

$$
d_W=R_{WC}u_C,\qquad P_W(\lambda)=c_W+\lambda d_W,\qquad \lambda>0.
$$

房间边界六面定义 x∈{0,depth}、y∈{0,width}、z∈{0,height}。对每个面上的轴 j：

1. 若 |d_W[j]|≤ε_parallel，跳过（平行）。
2. λ=(plane_j−c_W[j])/d_W[j]；若 λ≤ε_distance 则跳过（光心处/后方）。
3. P_W=c_W+λd_W；另外两轴必须在对应有限矩形边界内（容差 ε_boundary）。
4. 检查 u_C 前向有效且经过实际内参/畸变映射后投影在对应图像有效像素范围内；需要与渲染一致的遮挡/表面有效性。
5. 所有候选中选择最小 λ；相同 λ（棱边/角点）按稳定 surface_id 排序保证确定性。

对非房间场景，Scene::firstHit 应使用同源的有限几何面片/对象（可考虑射线三角形或有限平面片交点）。**不允许把目标仓库中 MakeLandmarks 的绘图圆点自动视为无限实体墙面。** 如场景尚不提供与渲染一致的几何表示，返回 UnsupportedScene/NoHit，并在功能验收前补齐其明确承诺的场景求交模型。

结果长度等于输入 bearings.size()，逐项携带 input_index、命中或失败状态；全局缓存失效/未知相机属于整个接口失败，单条无效方向/无交点可在对应 RayResult 报告。命中值必须有限、λ>0、位于实体表面；不提供任意视线的虚构路标点。

### 7.3 容差与有效性

使用结合房间尺寸的绝对/相对容差（而非任意把单位米的容差与方向无量纲混用）；拒绝零向量、NaN、Inf，单位范数容差明确并测试。可以拒绝超容差向量而非自动修改其意义。非前向视线、像素不在视域内、平行/背向、距离太小、交点不可见等有明确 reason。查询不能改变状态、帧号、噪声或缓存。

## 8. 配置变化事务及生命周期

- 所有 setter 在提交前验证维数、数值、旋转 SO(3)、采样频率、噪声与传感器 ID，失败保持原配置不动。
- 设置不同内外参、畸变、轨迹、场景后，使不能再安全解释的图像快照失效；查询历史图像明确失败而不重算“新参数版本”的旧位姿。
- 已成功生成的时间/编号不能因为 setter 发生而重置。仅 reset 会复位采样时间、帧号、随机状态和缓存；配置保留。
- 返回 cv::Mat/图像对象对 caller 的生命周期独立于下一次 generate 和 simulator 销毁，不暴露临时裸指针。只读结果可共享引用计数缓冲，若共享可写缓冲则必须防止对后续模拟输出的污染。
- 错误诊断不得仅打印 stderr；提供可程序化检查的错误码及简洁说明。不得生成一半 stereo 或一半 imu 帧后报告成功。

## 9. CLI、文件导出与安装

### 9.1 CLI 复用库

保留目标仓库现有两条示例路径：forward_constant_velocity 和 optical_axis_constant_rotation 的场景输出；移入独立库后，CLI 调用 generate 获取 cam0、cam1、imu0，按显式输出选项写 YAML/CSV/PNG 和真值。保留已有 cam0、imu0、groundtruth 的有效字段；新增 cam1 并注明双目标定，不随意改变旧字段单位。

当前目标 src/main.cpp 中会执行 remove_all 删除输出目录：**新实现必须取消未授权的隐式递归删除**；目标目录已存在时由显式覆盖选项控制，否则报错/选择新目录。安装运行库时不自动导出/写盘。

### 9.2 CMake 方案

~~~cmake
# 伪代码，只表达必须具备的行为
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "VisualSim supports Linux/Ubuntu 24.04 only")
endif()
# find_package(Eigen3 ...) / find_package(OpenCV ...) / 可选 Sophus
# add_library(visualsim SHARED ...)
# add_library(VisualSim::VisualSim ALIAS visualsim)
# target_include_directories(visualsim PUBLIC BUILD_INTERFACE / INSTALL_INTERFACE)
# install(TARGETS visualsim EXPORT VisualSimTargets ...)
# install(DIRECTORY include/visualsim/ DESTINATION include/visualsim)
# install(EXPORT VisualSimTargets NAMESPACE VisualSim:: ...)
# configure_package_config_file(... VisualSimConfig.cmake ...)
# write_basic_package_version_file(...)
~~~

完整实现应支持 CMAKE_INSTALL_PREFIX、GNUInstallDirs、依赖导出、安装后的 RPATH/动态链接器发现、无源码树 include、消费者 find_package(VisualSim CONFIG REQUIRED) 与 target_link_libraries(app PRIVATE VisualSim::VisualSim)。Windows presets 和 .ps1 构建入口及 README 中 Windows 安装指令应删除/停用。不要引入 ROS find_package、ament_target_dependencies 或任何 ROS 生成头。

**不提出降低 CMAKE_CXX_STANDARD / CMake / GCC 的任务；如为提高特性使用标准，应以当前已具备的工具链及实际源代码要求为准。** 顶层禁用 Windows 构建并不意味着不得继续使用现有 GCC/C++ 新特性。

## 10. 测试与验收用例（后续实施，不在本轮执行）

| 用例 | 断言 | 对应 SRS |
| --- | --- | --- |
| T01 独立消费者 | Ubuntu 24.04 构建 .so → 非源码目录安装 → 独立 CMake 程序链接调用成功，依赖树无 ROS | AC-01/11 |
| T02 目标平台 | Windows CMake 配置期报明确错误；未遗留有效 Windows 构建入口；无降级改动 | AC-02 |
| T03 轨迹解析 | 静止/前向匀速/匀加速/定轴纯旋转/圆周/拼接轨迹的 p/R/v/a/ω/α 解析值及导数关系 | AC-03 |
| T04 动态相机标定 | 只改变 cam0 或 cam1 的 K、T_BC、radtan，观察对应投影实际改变、元数据同步，另一目不误用 | AC-04 |
| T05 动态 IMU 标定 | 非零杆臂 + 角加速度的加速度计输出符合公式；更改姿态外参可旋转三轴结果 | AC-05 |
| T06 噪声 | 默认无噪；启用后参数改变影响实测，固定 seed 完全复现；不同间隔使用正确离散化 | AC-05 |
| T07 门控/重复时间 | 0/1/2/3 帧组合、编号自 0、左右同步；同刻同传感器拒绝且无副作用，同刻另一传感器合法 | AC-06/07 |
| T08 时间类型/越界 | 编译期同类型断言、原样保持纳秒、拒绝大值溢出/越界/回退，reset 后合法重放 | AC-06/07 |
| T09 缓存 | 64 组后淘汰、cam0/cam1 分别正确、IMU-only 不入缓存、改标定或场景后拒绝陈旧查询 | AC-08 |
| T10 有限面求交 | 六面、边/角、正后向、平行、相机视域内外、相机双目偏移后与解析真值相等 | AC-09 |
| T11 图像/几何同源 | 像素投影畸变和表面遮挡与 raycast 第一命中一致，前向场景不返回伪造交点 | AC-04/09 |
| T12 保留 CLI | 两个已有场景继续生成；新增 cam1；数据文件单位/字段正确；无隐式删除目录 | AC-10 |
| T13 来源仓库只读 | 对比来源基线提交未被本次任务写入；所有变更仅在目标仓库 | AC-11 |

测试应包含拒绝重复时间后再次用**更大**的合法时间生成，核对未消耗编号与随机序列；应覆盖一个三传感器批次仅其中一个时间无效时的整批回滚。若文档所称任何新功能没有测试通过，不得宣称生产可用。

## 11. 实施顺序和完成定义

| 阶段 | 代码与验证交付 | 前置 |
| --- | --- | --- |
| P0 | 检视两个仓库，按路径建立实际迁移差异清单；确认坐标方向、场景、纳秒及 API | 本三份文档 |
| P1 | 仅目标仓库中 SHARED .so、Linux guard、install/export、独立消费者 | P0 |
| P2 | 全解析 MotionState、完整导数、内建/自定义/分段轨迹，复用目标已有两轨迹 | P1 |
| P3 | 双目/畸变、真实 IMU 外参和可开关的实际噪声、配置 getter/setter | P2 |
| P4 | 门控原子生成、严格递增时间、位姿缓存、房间与其他场景真实几何求交 | P3 |
| P5 | CLI 迁移/旧数据集兼容、新 cam1、单元/集成/安装验收、无 ROS 依赖扫描 | P4 |

本轮**只把 SRS/HLD/LLD 写入目标仓库 docs/**，不实施 P0–P5 的代码改造，不做编译、CTest 或性能宣称。