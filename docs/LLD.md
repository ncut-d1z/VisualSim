# VisualSim 运行时库详细设计（LLD）

> 状态：**v1 设计合同基线（本轮文档审计完成后冻结，待代码实施和功能验证）**；本合同并非源码已实现或测试通过的证明。最终功能以 [SRS.md](SRS.md) 为准，架构见 [HLD.md](HLD.md)，工程实施/代码迁移见 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md)。
>
> 本文中的文件名、类型和函数签名是**拟议实现**，并不表示当前仓库已有对应 API。唯一可写目标仓库为 ncut-d1z/VisualSim。来源仓库 ncut-d1z/ros2-vio-node 只读。平台 Ubuntu 24.04，禁止 Windows 构建；不强制降级构建工具链。

## 1. 必须遵循的存储边界

**允许长时间保留的业务数据**仅为：当前运动类别/模式/参数/作用域栈、当前相机/IMU/Room 配置、调用者对此进行每次修改的**带纳秒时间戳事件历史**。这些数据足以重建任意有效时刻的物理状态。

不得持久保存历史图片、IMU 样本、相机位姿、射线命中、按帧运动状态或任何历史运动积分结果缓存。允许单次调用内部的临时局部对象；调用结束即释放。由于必须拒绝重复时间和连续编号，还需要各传感器最近成功生成时间与 next_frame_number 两项**最小协议控制变量**，不将其扩展为帧队列或结果索引。

本设计**不使用历史图像帧存在性**作为 Raycast 的前置条件；Raycast 通过事件历史按需计算查询时刻的位姿。

## 2. 已确定的公开接口合同（设计基线 v1）

此处定义**未来实现的接口合同 v1**，不是声称当前源码已实现。原型中不再采用“按代码规范微调”的非冻结口径；任何破坏签名/语义的后续修改均须同步更新 SRS、HLD、LLD、API 版本及追踪。

~~~cpp
namespace visualsim {
using TimestampNs = std::int64_t;
using SensorId = std::uint64_t;
static_assert(std::is_same_v<TimestampNs, std::int64_t>);

enum class Status : std::uint32_t {
  Ok=0, InvalidArgument=1, InvalidMode=2, InvalidTimestamp=3,
  EventOutOfOrder=4, EventHistoryFrozen=5, UnmatchedMotionScope=6,
  InvalidCalibration=7, CameraNotFound=8, DuplicateCameraName=9,
  NoCameraAtTime=10, DuplicateTimestamp=11, OutOfOrderTimestamp=12,
  TimeOutOfRange=13, NoIntersection=14, InvalidBearing=15,
  OutsideFov=16, InvalidConfigSchema=17,
  UnsupportedAngularAcceleration=18, InternalError=19
};
template<class T> struct Result {
  Status status;                 // Ok 或稳定枚举错误码
  std::optional<T> value;       // 仅 status == Ok 时有值
  std::string message;          // 可读诊断，不作为机器解析依据
};
template<> struct Result<void> {
  Status status;
  std::string message;          // 没有 value
};

struct LinearStationary {};
struct LinearConstantVelocity { Eigen::Vector3d velocity_W_mps; };
struct LinearConstantAcceleration {
  Eigen::Vector3d initial_velocity_W_mps;
  Eigen::Vector3d acceleration_W_mps2;
};
enum class CircularReactivationPolicy : std::uint32_t {
  ReanchorOnReactivation = 0,   // 出栈恢复时保持位置连续
  FixedWorldCenter = 1          // 出栈恢复时保持所声明世界圆心不变
};
struct LinearUniformCircular {
  Eigen::Vector3d center_W_m;          // 首次激活的世界圆心 (m)
  double radius_m;                     // 半径 > 0 (m)
  Eigen::Vector3d normal_W;            // 轨道平面单位法向
  Eigen::Vector3d phase_reference_W;   // 轨道平面内的单位零相位方向
  double phase_rad;                    // 生效时的初始相位 (rad)
  double rate_radps;                   // 有符号轨道角速率 (rad/s)
  CircularReactivationPolicy reactivation_policy;
};

using LinearMotionSpec = std::variant<
  LinearStationary, LinearConstantVelocity,
  LinearConstantAcceleration, LinearUniformCircular>;

struct AngularStationary {};
struct AngularFixedAxisRotation {
  Eigen::Vector3d axis_B;       // 单位固定轴
  double rate_radps;            // 有符号角速率
};
using AngularMotionSpec = std::variant<
  AngularStationary, AngularFixedAxisRotation>;

struct MotionState {
  Eigen::Vector3d position_W_m;
  Eigen::Matrix3d rotation_WB;
  Eigen::Vector3d velocity_W_mps;
  Eigen::Vector3d acceleration_W_mps2;
  Eigen::Vector3d angular_velocity_B_radps;
  Eigen::Vector3d angular_acceleration_B_radps2;
  bool derivative_discontinuous; // 切换点右侧导数标志
};
struct CameraCalibration {
  Eigen::Isometry3d T_BC = Eigen::Isometry3d::Identity(); // C -> B
  Eigen::Matrix3d K =
      (Eigen::Matrix3d() << 458.654, 0.0, 367.215,
                            0.0, 457.296, 248.375,
                            0.0,   0.0,     1.0).finished();
  int width = 752;
  int height = 480;
  std::array<double,4> radtan{0.0,0.0,0.0,0.0}; // k1,k2,p1,p2
  double nominal_rate_hz = 20.0;
};
struct ImuCalibration {
  Eigen::Isometry3d T_BS = Eigen::Isometry3d::Identity(); // S -> B
  double nominal_rate_hz = 200.0;
  double gyroscope_noise_density = 0.0;
  double gyroscope_random_walk = 0.0;
  double accelerometer_noise_density = 0.0;
  double accelerometer_random_walk = 0.0;
  Eigen::Vector3d initial_gyro_bias_S = Eigen::Vector3d::Zero();
  Eigen::Vector3d initial_accel_bias_S = Eigen::Vector3d::Zero();
  bool noise_enabled = false;
  std::uint64_t seed = 0;
};
struct RoomConfig {
  double depth_m = 10.0;
  double width_m = 10.0;
  double height_m = 3.0;
};
struct CameraHandle { SensorId id; std::string name; };
struct SensorGate { bool camera=false; bool imu=false; };
struct ImuMeasurement {
  Eigen::Vector3d angular_velocity_S_radps;
  Eigen::Vector3d specific_force_S_mps2;
};
struct SensorFrame {
  TimestampNs timestamp_ns;
  SensorId sensor_id;           // 改名后仍不变
  std::string sensor_name;
  std::uint64_t frame_number;
  std::variant<cv::Mat, ImuMeasurement> payload;
};
struct SampleBatch { std::vector<SensorFrame> frames; };
struct RayHit {
  std::size_t input_index;
  bool hit;
  Eigen::Vector3d point_W_m;   // 仅命中时有效
  double range_m;              // 仅命中时有效
  std::string surface_name;    // 仅命中时有效
  Status failure;              // 命中时 Ok
};
struct ChangeRecord {
  TimestampNs effective_time_ns;
  std::uint64_t insertion_order;
  std::string object_kind;
  std::string operation;
  std::string canonical_payload_yaml; // schema v1 的序列化新参数
};

class Simulator final {
public:
  Simulator();
  ~Simulator();                              // 在 .cpp 定义
  Simulator(Simulator&&) noexcept;
  Simulator& operator=(Simulator&&) noexcept;
  Simulator(const Simulator&) = delete;
  Simulator& operator=(const Simulator&) = delete;

  Result<void> BeginMotionMode(TimestampNs effective_ns);
  Result<void> EndMotionEnd(TimestampNs effective_ns);
  Result<void> SetLinearMotion(TimestampNs effective_ns,
                               const LinearMotionSpec&);
  Result<void> SetAngularMotion(TimestampNs effective_ns,
                                const AngularMotionSpec&);
  Result<void> SetPose(TimestampNs effective_ns,
                       const Eigen::Vector3d& position_W_m,
                       const Eigen::Matrix3d& rotation_WB);

  Result<CameraHandle> CreateCamera(
      TimestampNs effective_ns, const Eigen::Isometry3d& T_BC); // 默认 K/尺寸/畸变/采样率
  Result<void> RenameCamera(TimestampNs effective_ns,
                            SensorId camera_id, std::string_view new_name);
  Result<void> SetCameraCalibration(TimestampNs effective_ns,
                                    SensorId camera_id, const CameraCalibration&);
  Result<void> SetImuCalibration(TimestampNs effective_ns,
                                 const ImuCalibration&);
  Result<void> SetRoom(TimestampNs effective_ns, const RoomConfig&);
  Result<CameraCalibration> GetCameraCalibration(
      TimestampNs at_ns, SensorId camera_id) const;
  Result<ImuCalibration> GetImuCalibration(TimestampNs at_ns) const;
  Result<RoomConfig> GetRoom(TimestampNs at_ns) const;
  Result<std::vector<CameraHandle>> ListCameras(TimestampNs at_ns) const;
  Result<MotionState> GetMotionState(TimestampNs at_ns) const;
  Result<std::vector<ChangeRecord>> GetChangeHistory() const;
  Result<std::vector<ChangeRecord>> GetChangeHistory(
      TimestampNs from_ns, TimestampNs to_ns) const; // [from,to]
  Result<void> LoadMotionConfig(const std::filesystem::path& yaml_path);

  Result<SampleBatch> Generate(TimestampNs at_ns, SensorGate);
  Result<std::vector<RayHit>> Raycast(
      TimestampNs at_ns, const std::vector<Eigen::Vector3d>& bearings_C,
      std::string_view camera_name = "cam0") const;
  Result<void> ResetSamplingSession();  // 保留运动/标定历史

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace visualsim
~~~

### 2.1 返回值、错误与异常

- 所有公开可失败操作统一返回 Result<T> 或 Result<void>；成功时 status=Ok，非 void 必有 value；失败时非 void 无 value 且 message 非空。Result<void> 不具有 value；空门控 Generate 返回 Ok 与空 frames。
- v1 的 Status 枚举名称和显式数值保持稳定；message 是人工诊断，不是可解析协议。可预期的非法参数、时间、相机和配置使用已有对应状态码，而不是笼统 InternalError。
- **第三方异常翻译**：yaml-cpp 的解析/重复键/字段类型错误、配置读取中的文件系统/IO 错误映射为 InvalidConfigSchema（message 记录 YAML 路径/字段）；OpenCV 的 cv::Exception、未分类 yaml-cpp 异常、其他可捕获的第三方或 std::exception 统一翻译为 InternalError（保留诊断）；可捕获的未知异常同样翻译为 InternalError。上述异常均不能穿过公开接口。
- **仅 std::bad_alloc 可穿越 C++ 公开边界**，前提是同一兼容工具链；它也不得留下半提交。Setter、Begin/End、Create/Rename、LoadMotionConfig、Generate 和 ResetSamplingSession 的失败或可捕获异常均具**强状态保证**：任何错误都不得改变已提交事件、相机 ID/编号、配置、协议时间和帧号。事件导入不能只提交部分事件。
- **事务的提交点与返回安全（v1 冻结）**：先在单次调用局部空间完成所有可能抛异常的验证、历史事件/相机注册结构扩容、图像 cv::Mat 分配、IMU 计算、完整 Result/SampleBatch 及错误消息构造；之后以不抛异常的状态交换/已预分配字段赋值提交事件与最小协议计数器。对返回对象，必须使提交后的返回路径为 noexcept move 或可证明不会分配的返回构造；若具体类型无法保证，需在提交前返回对象准备阶段完成必要分配，或采用可回滚事务，**不能先递增帧号再因构造/返回 Result 失败而破坏强保证**。相关 noexcept 约束应在实现时以类型特性和故障注入验证，不以当前文档宣布编译已通过。
- Eigen 不保证以异常报告违反前置条件的断言，调用前须验证维度、有限值、SO(3)、向量合法性；不可恢复的进程中止不属于异常返回协议。只读 Get*/Raycast 不改变状态；Raycast 全局错误和逐条失败按 §7 区分。

### 2.2 线程安全与多实例

同一个 Simulator 实例**不提供内部同步**。用户必须串行调用 Set*、Begin/End、LoadMotionConfig、Generate、ResetSamplingSession，也必须避免它们与 Get*/Raycast/GetChangeHistory 并发；在无写入的期间，多个只读查询可并行，但调用者仍须确保实例活着且未发生移动/销毁。不同 Simulator 实例之间不共享可变历史、计数器或噪声状态，可以并发运行。回调/日志不允许在内部持锁期间调用用户代码（当前 v1 不提供回调式接口）。

### 2.3 数据所有权、生命周期与二进制兼容边界

- Simulator 用 Pimpl（Impl 在 .cpp 定义）隐藏内部状态；公开值类型是**值语义**。GetChangeHistory、ListCameras、Raycast 和 Generate 返回独立 std::vector；返回体在 Simulator 销毁后仍有效。
- v1 相机图像**默认且本版固定为 8 位单通道灰度 CV_8UC1**，尺寸严格等于各自有效 CameraCalibration 的 width×height；没有静默 RGB/BGR 转换。图像结果中的 cv::Mat 由每次生成单独分配，返回时转移/共享该帧独立的 OpenCV 引用计数缓冲；后续 Generate 不重用可写底层存储，调用者可安全持有或释放。若要完全独立复制可在调用方 clone；库不保存历史 cv::Mat。
- Pimpl 只减少布局暴露，**不提供跨编译器、libstdc++ 或 OpenCV/Eigen 版本的稳定二进制 ABI**。初始 API/SONAME 主版本为 1；破坏公开类型/布局/异常 ABI 时须升级 SONAME 和 CMake package 主版本。目标只保证相同兼容 Linux ABI/工具链及依赖版本的消费者可运行。
- 输出时间、名称、帧号等属于返回对象所有，不借用临时字符串指针；仅在 RayHit.hit=true 时读取 point_W_m、range_m 和 surface_name。

### 2.4 时间域及参数合法性（v1 冻结）

有效纳秒范围 0≤t≤INT64_MAX。事件按非递减纳秒追加，所有纳秒字段为 std::int64_t；整数差分先检验溢出，解析运动时只将相对纳秒差转成 double 秒，不能用浮点秒往返覆盖原时间。输入非有限按 InvalidArgument/InvalidCalibration，计算导致的非有限运动输出按 TimeOutOfRange。

| 数值检查 | v1 确定接受范围（等号属于有效边界） | 失败 |
| --- | --- | --- |
| SO(3) 旋转矩阵 R | 全部有限，Frobenius 范数 \(\|R^\mathsf{T}R-I\|_F\le 10^{-10}\)，\(|\det R-1|\le10^{-10}\) | 位姿 InvalidArgument；外参 InvalidCalibration |
| 单位轨道法向、相位参考方向及旋转轴 | 各分量有限且 \(|\|u\|-1|\le10^{-9}\) | InvalidMode |
| 圆周法向与相位零方向 | \(|n_W^\mathsf{T}e_W|\le10^{-9}\) | InvalidMode |
| 配置文件四元数 (w,x,y,z) | 有限且 \(|\|q\|-1|\le10^{-10}\)；合格后确定性归一化一次 | InvalidConfigSchema |
| 相机内参 K | fx>0、fy>0、宽高正且有限；第三行偏差 \(|K_{20}|,|K_{21}|,|K_{22}-1|\le10^{-12}\) | InvalidCalibration |
| 圆周初始/固定圆心位置相容 | \(\|p_\text{anchor}-(c+r_0)\|\le\max(10^{-9}\ {\rm m},10^{-10}r)\) | InvalidMode |
| Room 解析射线 | 方向平行阈值 \(10^{-12}\)（无量纲）；最小正向距离 \(\max(10^{-9}\ {\rm m},10^{-12}D)\)；有限面边界容差 \(10^{-12}D\)，D 为房间对角线（米） | §7 的 RayHit.failure |

单位视线另依 §7 的 \(|\|u_C\|-1|\le10^{-9}\) 判定。所有越界单位向量均拒绝，不得悄悄归一化；合格后可以作一次确定性归一化以减小数值误差。Room 尺寸、焦距/采样率须为有限正数；噪声强度须为有限非负数。

## 3. 立即式命令和双类别栈

### 3.1 生效状态

在任意时刻，运动配置为：

~~~text
ActiveMotion = {
  linear: exactly one LinearMotionSpec,
  angular: exactly one AngularMotionSpec
}
ScopeStack = stack of saved ActiveMotion
~~~

默认两类均为 Stationary。BeginMotionMode 在给定时间保存当前一对模式及全部参数至作用域栈；EndMotionEnd 从栈顶恢复这一对模式及参数。SetLinearMotion 替换整个 linear 槽，SetAngularMotion 替换整个 angular 槽。若想仅改变该模式的某个参数，可为该槽提供精确的 SetLinearParameter/SetAngularParameter 操作，但效果仍是**替换同类当前有效规范**而非新建第二个并行模式。

### 3.2 调用示例与期望效果

~~~cpp
Simulator sim;
sim.SetPose(0, p0, R0);
sim.SetLinearMotion(0, LinearConstantVelocity{v_W});

sim.BeginMotionMode(1'000'000'000LL);
sim.SetAngularMotion(1'000'000'000LL,
                     AngularFixedAxisRotation{axis_B, omega});
sim.SetLinearMotion(2'000'000'000LL,
                    LinearUniformCircular{center_W, radius_m, normal_W, phase_ref_W,
                                           phase_rad, rate,
                          CircularReactivationPolicy::ReanchorOnReactivation});
sim.EndMotionEnd(3'000'000'000LL);

auto state = sim.GetMotionState(2'500'000'000LL);
auto hits  = sim.Raycast(2'500'000'000LL, bearings_C, "cam0");
~~~

这是按 v1 接口示意的调用片段，省略了每次 Result 的错误检查，**不是当前仓库已经能够编译的样例**。Begin/End 在相同时间允许零时长作用域，操作按稳定插入顺序执行。没有对应 Begin 的 End 返回 UnmatchedMotionScope；不自动生成临时作用域。

### 3.3 时间事件与历史保护

EventTimeline 的每条事件含 timestamp_ns:int64、单调递增的 insertion_order、类型、目标不可变 ID、值及对作用域的影响；当前配置可由完整事件流重建。配置文件解析必须转换为同一事件格式，保证顺序等价。GetChangeHistory() 返回完整显式事件流（含 YAML baseline 实际提供字段转换得到的合成 t=0 事件）；GetChangeHistory(from_ns,to_ns) 为闭区间 [from_ns,to_ns] 过滤，按 (timestamp_ns,insertion_order) 排序，要求 0≤from_ns≤to_ns≤INT64_MAX，否则 InvalidTimestamp；没有事件时 Ok 且返回空列表。省略的 baseline 默认项不生成历史变更记录。

默认只允许按**生效时间非递减**的顺序追加事件，同一时间戳可按调用顺序多次覆盖。已经成功生成任意传感器数据后，不接受在**任一已生成帧的时间戳及之前**新增/修改事件，避免历史重算改变旧输出；只读 GetMotionState/Raycast 可以查询历史时间，不会冻结配置也不会生成帧。

需要重新定义已经生成过数据的历史时，必须**新建 Simulator 实例**并重新构造/导入历史。ResetSamplingSession 仅重置采样序号和上一次采样时间，**不清空、不解冻任何历史事件**。v1 不提供 ClearConfiguration/ReplaceEventTimeline；读取历史与 Raycast 不会冻结事件。

## 4. 基于事件的分段解析运动重算

### 4.1 算法概述

求时间 t 的状态，不读取任何缓存的帧/位姿：

~~~text
Evaluate(t):
  validate t and event-time domain
  initialize state at t=0: p=(0,0,0)m, R=I, modes=(linear stationary, angular stationary)
  scopeStack := empty local replay stack
  cursor_ns := start_ns
  for event_group in chronological events with time <= t:
      state := IntegrateAnalytically(state, active_modes, event.time - cursor_ns)
      apply group events in insertion_order:
          Begin -> push current mode pair
          SetLinear -> replace linear mode and parameters
          SetAngular -> replace angular mode and parameters
          End -> restore popped mode pair
          SetPose -> replace absolute p/R by caller request
          other config events -> update their effective values for query
      cursor_ns := event.time
  state := IntegrateAnalytically(state, active_modes, t - cursor_ns)
  return state and effective calibration/room if needed
~~~

循环中的 state 和 scopeStack 是**单次调用临时变量**，不是跨调用缓存。调用生成、IMU、交点、运动真值时复用相同的 Evaluate 语义。多个改变时间相同的事件先按顺序应用，再使用右侧模式做该时间之后的运动。切换边界 GetMotionState 返回右侧模式的速度/加速度，并在 derivative_discontinuous=true 时提示两侧导数不连续。

### 4.1.1 初始条件、事件作用域和时间边界

若调用者没有 SetPose(0)，仿真初始运动状态 p_WB=(0,0,0) m、R_WB=I、v=a=ω=α=0；若有时间为 0 的 SetPose 事件，按照同刻 insertion_order 生效。默认 Room 尺寸 depth=10 m、width=10 m、height=3 m；当配置文件提供显式初始值时以该初始值作为同一时刻的起点。

时间域是 [0, INT64_MAX] 纳秒；在无有效运动导数、非有限输出或无效外参条件下必须报告错误。事件组发生在时刻 t_i：先以旧模式解析 [t_{i-1},t_i) 到事件左极限，再按 insertion_order 应用所有 t_i 事件，最后令 t_i 的查询取**右侧状态**。无事件时刻无需采样即可直接查询。采用相对时间差纳秒转换为秒，避免大绝对时间导致的秒数差分抵消。

### 4.2 基本运动解析式

设每段开始为 t0，Δt=(t−t0)·10⁻⁹ s，段首世界位置 p0、载体朝向 R0。

**线静止**：

$$
p_W(t)=p_0,\qquad v_W(t)=0,\qquad a_W(t)=0.
$$

**匀速直线**（世界系恒定 v）：

$$
p_W(t)=p_0+v_W\Delta t,\qquad a_W=0.
$$

**匀加速直线**（世界系常加速度 a、段初速度 v0）：

$$
p_W(t)=p_0+v_0\Delta t+\tfrac12 a_W\Delta t^2,\qquad
v_W(t)=v_0+a_W\Delta t,\qquad a_W(t)=a_W.
$$

**匀速圆周（显式圆心 + 半径 + 轨道平面 + 相位）**：输入声明世界圆心 \(c_{\rm decl}\)、半径 \(r>0\)、单位轨道法向 \(n_W\)、平面内单位零相位方向 \(e_W\)、初始相位 \(\phi\) 和有符号轨道速率 \(\Omega\)。设 \(Q(\theta)=\operatorname{Exp}([n_W]_\times\theta)\)、初始有向半径 \(r_0=r Q(\phi)e_W\)、\(\theta=\Omega\Delta t\)。

**首次设置此模式时**，必须满足段首位置 \(p_0=c_{\rm decl}+r_0\)（固定容差见 §2.4），否则返回 InvalidMode，事件不得提交；有效世界圆心 \(c_{\rm eff}=c_{\rm decl}\)。

$$
p_W(t)=c_{\rm eff}+Q(\theta)r_0
=p_0+\bigl(Q(\theta)-I\bigr)r_0,
$$

$$
v_W(t)=\Omega n_W\times Q(\theta)r_0,\qquad
a_W(t)=\Omega^2 n_W\times\bigl(n_W\times Q(\theta)r_0\bigr).
$$

**作用域出栈恢复旧圆周规范**时，保存的声明圆心、半径、零相位方向、相位、法向、速率与重入策略全部恢复，先依据声明参数计算相同的 \(r_0\)：
- ReanchorOnReactivation：从退出时的当前位置 \(p_{\rm current}\) 继续，将本段**有效**圆心确定为 \(c_{\rm eff}=p_{\rm current}-r_0\)；保留声明圆心在事件历史中的原值、不把有效圆心缓存为持久状态。
- FixedWorldCenter：保持 \(c_{\rm eff}=c_{\rm decl}\)，要求 \(p_{\rm current}=c_{\rm decl}+r_0\) 满足 §2.4 容差；冲突时 EndMotionEnd 返回 InvalidMode，栈和事件历史保持不变，禁止隐式位置瞬移。

重新调用 SetLinearMotion 设置**新圆周规范**仍须验证声明圆心和当前位置的一致性，而非忽略 center_W_m 字段。

**角静止**：R_WB(t)=R0，ω_B=α_B=0。

**定轴匀速纯旋转**：以 B 系固定单位轴 n_B 和角速率 ω：

$$
R_{WB}(t)=R_0\operatorname{Exp}([\omega n_B]_\times\Delta t),\qquad
\omega_B=\omega n_B,\qquad \alpha_B=0.
$$

不同时间段的旋转必须**按时间先后在当前段首姿态上右乘**，不能把不同时刻不共轴的旋转向量简单相加。线/角类别可跨类叠加，世界系平移规律独立于角模式。非零角加速度**不在本次规定的角模式集合内**；对 SetAngularAcceleration(nonzero) 之类的矛盾设置明确报 UnsupportedAngularAcceleration，而不伪造解析结果。

### 4.3 参数切换、出栈与显式跳变

1. 每次 SetLinearMotion、SetAngularMotion、EndMotionEnd 都先解析旧运动到当前事件时间，再以当前 p/R 作为新段锚点；类内覆盖另一个类别不受影响。
2. BeginMotionMode 保存的是线、角两类模式参数快照，不保存待恢复的旧位姿。EndMotionEnd 只恢复模式规范，并从事件时刻**重新锚定**，不得跳回 Begin 时的位置/朝向。
3. UniformCircular 首次激活核验显式圆心、半径、法向、相位及连续性；从作用域恢复时遵循 CircularReactivationPolicy（§4.2），不得无视已声明的圆心或相位。
4. SetPose(t,p,R) 是**唯一允许主动造成 p/R 不连续的业务事件**，同时按有效模式重新锚定：恒加速模式恢复指定的 initial_velocity_W_mps；圆周模式在 ReanchorOnReactivation 策略下由新位置与当前 r0 推导有效圆心，在 FixedWorldCenter 策略下必须满足声明圆心/相位的 §2.4 相容性，否则拒绝 InvalidMode。需要保留切换前速度时，调用者须在同一时间的后续事件明确设置。
5. 查询恰好处于模式/SetPose 事件时间 t 时，返回在所有 t 事件执行**之后**的右侧位姿与相应模式解析导数；如左右导数不同或 SetPose 造成跳变，设置 derivative_discontinuous=true。段内支持的两个角模式角加速度恒为 0；不虚构切换冲量的有限角加速度。
6. 同一时刻多事件按提交顺序依次执行，若产生矛盾配置（例如零长度圆周半径、非正房间尺寸、非法 SO(3)）整个操作拒绝，原事件历史不变。

## 5. 动态相机及标定时间线

### 5.1 注册、默认相机与重命名规则

CameraRegistry 的 SensorId 稳定且不会重用，名称按时刻解析。自动名称优先使用 cam0、cam1、cam2……，**若后续后缀已被合法重命名占用则跳过并继续递增**；更名不回收已发行自动序号，也不更改此前帧的 ID/编号。

**默认与显式创建的定稿规则（避免查询引起隐式状态突变）：**

- 起点 t=0 的初始配置视作有一台**虚拟默认** cam0（T_BC=(I3,0)），无需真正修改事件历史；只读 Raycast/ListCameras 不会“创建”对象。
- 只有**首条 CreateCamera 的生效时间为 0、此前没有任何针对虚拟默认 cam0 的 SetCameraCalibration/RenameCamera 等显式修改事件、也没有采样成功记录**时，首个显式相机才能以调用者 T_BC **替代尚未使用的虚拟默认配置**，占用稳定 SensorId=0 和名称 cam0；新相机其他标定字段取本节固定默认值。
- 如果首条 CreateCamera 的生效时间大于 0，或者此前已在 t=0 通过合法事件修改过虚拟 cam0，则保留原有 cam0 的**不可变身份及所有历史标定/名称**；本次 CreateCamera 分配新 SensorId（通常为 1）与后续可用 camX 名称，不得覆盖旧 cam0。对默认 cam0 的先前只读查询不会引发实质创建事件，不影响这个判定。
- 若已在某时刻成功生成传感器帧，任何 CreateCamera 生效时间小于或等于该已生成帧时间的请求须先按 §3.3 返回 EventHistoryFrozen；不能利用 CreateCamera 替代已输出的传感器身份。
- 自动 SensorId 逐次单调递增，永不重用；**自动 camX 名称的数字后缀与 SensorId 不要求相等**。分配新名称时，从 next_auto_camera_index 起单调扫描，遇当前时刻已由其他相机占用、或历史上曾自动分配过的 camX 编号则跳过，选择下一个可用 camX；跳过的编号亦不回收。即使此前 RenameCamera 把 cam0 改成 cam1，也应让之后 CreateCamera 自动选择 cam2（而不是一直返回 DuplicateCameraName）。RenameCamera **显式指定与其他相机当前名称冲突**才返回 DuplicateCameraName，且不得更改名称历史。
- 显式创建、默认相机和之后的新相机使用相同的默认图像标定，除非调用者以带时间戳的 SetCameraCalibration 覆盖。
- RenameCamera(t,id,new_name) 禁止空名称、与该时刻其他相机同名；修改只影响 t 以后，旧历史使用旧名。后续生成使用新名，但同一 SensorId 的时间去重和 frame_number 保持连续。

事件历史包括所有显式 CreateCamera、RenameCamera 和标定变更。GetCameraCalibration(t,id)、ListCameras(t)、Raycast(t,name) 查询只采纳 t 时有效的定义，不依赖此前 Generate 是否调用。

**v1 的完整相机默认标定（与目标仓库当前单目生成器基线一致）：** 宽 752、高 480 像素；fx=458.654、fy=457.296、cx=367.215、cy=248.375、skew=0，K 第三行为 (0,0,1)；radial-tangential 畸变系数 (k1,k2,p1,p2)=(0,0,0,0)；nominal_rate_hz=20.0。默认 cam0 的 T_BC=(I3,0)；CreateCamera(t,T_BC) 创建任何新相机时只覆盖该 T_BC，**其余默认字段按上述值确定，随后可被 SetCameraCalibration 在有效时间覆盖**。相机图像固定输出 CV_8UC1 灰度（§2.3），外部 caller 可以直接按当前默认标定生成图像和读取 GetCameraCalibration，不需要额外初始化调用。

**nominal_rate_hz 是标称元数据**，不触发后台生成、不限制调用者选择纳秒采样时刻；传感器 Generate 的时间仍按 §8 验证逐流严格递增。初始化后的有效相机配置以 getter 返回为准，禁止导出的 sensor.yaml 与实际投影标定不一致。

### 5.2 外参与相机成像

采用 W（世界）、B（载体）、C（相机）、S（IMU）坐标系。T_BC 把相机系坐标变到载体系：

$$
R_{WC}=R_{WB}R_{BC},\quad c_W=p_{WB}+R_{WB}t_{BC},
\quad X_C=R_{WC}^\top(X_W-c_W).
$$

相机采用针孔和 radial-tangential 畸变（k1,k2,p1,p2）。对 z>0 的三维点，x=X_C/Z_C，y=Y_C/Z_C，r²=x²+y²：

$$
x_d=x(1+k_1r^2+k_2r^4)+2p_1xy+p_2(r^2+2x^2),
$$

$$
y_d=y(1+k_1r^2+k_2r^4)+p_1(r^2+2y^2)+2p_2xy.
$$

像素由 K·[x_d,y_d,1] 得到（适当归一化）。渲染 Room 必须以各相机当时有效 K、T_BC、畸变确定真实像素位置、视域与遮挡，不能仅更新 YAML。允许任意数量相机使用各自标定，不把所有相机投影结果先求“左右目公共路标”而丢掉其他相机可见目标。

## 6. IMU 真实模型与无结果缓存的噪声

### 6.1 刚性外参、比力和角速度

已知状态 p_WB、R_WB、a_WB、ω_B、α_B，IMU 到载体的 T_BS=(R_BS,t_BS)，世界系重力**固定为** g_W=(0,0,−9.81) m/s²。**v1 不提供 SetGravityWorld 或重力的 YAML/事件配置**：重力是这一版的常量环境约定，始终参与比力模型；如需更改，需要明确升版扩展公共设置/查询接口、事件历史与 YAML，而不是在实现里加入不留历史的隐藏 setter。

$$
a_{WS}=a_{WB}+R_{WB}\Bigl(\alpha_B\times t_{BS}
+\omega_B\times(\omega_B\times t_{BS})\Bigr),
$$

$$
\omega_S=R_{BS}^\top\omega_B,\qquad
f_S=R_{BS}^\top R_{WB}^\top(a_{WS}-g_W).
$$

输出 ω_S 为 rad/s，f_S 为 m/s²；必须包含非零杆臂在旋转下的向心项，角加速度由当前运动模式提供（本版各常速角段内为 0）。切换点角速度跳变意味着理想冲量未定义；返回右侧非冲量状态，标记边界，不虚构无限角加速度峰值。

### 6.2 噪声单位、离散化、无缓存按需重算

每个 IMU 标定字段采用明确单位：gyroscope_noise_density 为 rad/s/√Hz，gyroscope_random_walk 为 rad/s²/√Hz；accelerometer_noise_density 为 m/s²/√Hz，accelerometer_random_walk 为 m/s³/√Hz。等效连续时间标准差约定为白噪声密度 N、零偏布朗运动强度 K，不将已保存 YAML 值视为无需数值生效的注释。

关闭 noise_enabled 时，不添加随机白噪与随机游走（默认 initial_*_bias_S=0）。开启后，每轴：

$$
y_{\omega,k}=\omega_{S,k}+b_g(t_k)+n_{g,k},\qquad
y_{f,k}=f_{S,k}+b_a(t_k)+n_{a,k}.
$$

白噪声标准差为 N/√Δt，其中首个成功 IMU 帧的 Δt=1/nominal_rate_hz，后续 Δt=(本次 timestamp_ns−imu0 最后成功 timestamp_ns)×10⁻⁹ 秒。有效 Δt 必须严格正，且有限。每轴 n=N·Z/√Δt，Z 为标准正态，随机键由 seed、配置事件 insertion_order、sensor ID、该采样时间与轴决定；同种子、同一采样序列在同一工具链下可复现。

**偏置随机游走不用历史样本/偏置缓存**：定义连续时间 b(t)=b_initial+K·W(t−t_epoch)，其中 t_epoch 为最近一次 IMU 标定/噪声设置事件的生效时刻，W 是按纳秒整数时间可寻址的零均值布朗运动，Cov(W(s),W(t))=min(s,t)（秒）。每次显式 SetImuCalibration 定义新的 t_epoch、seed 和 b_initial，形成新的随机过程 epoch；不隐式继承旧 epoch 已积累的偏置。

**固定时间根域与整数边界（v1 冻结）**：以最近一次噪声设置事件 t_epoch 为相对时间原点，先验证 0≤t_epoch≤t≤INT64_MAX，再计算有符号非负差 t−t_epoch，并转换为 std::uint64_t 的 relative_ns，故 0≤relative_ns≤INT64_MAX。抽象二叉 dyadic Brownian-bridge 树的根采用**左闭右开**区间 [0,2^63) ns，长度采用 uint64_t(1)<<63 表示；右端点 2^63 **不是** std::int64_t 可表示的 TimestampNs，因此实现绝不把该右端点转换为 TimestampNs，也不计算可能溢出的有符号 t_epoch+2^63。节点起点、相对偏移、树层级和节点索引使用无符号整数；整树共有 63 层二分至 1 ns 叶，查询 t_epoch 时 W(0)=0 精确成立，t=INT64_MAX 仍落在半开根区间内。任何时间差或无符号索引运算都须检测不合法范围，不能发生隐式有符号溢出。

随机过程在该根域内按时间一致的确定性 Brownian bridge 生成：根区间长度 T_s=(2^63 ns)×10^-9 s，根增量为 sqrt(T_s)·Z(root)。对子区间长度 L_s、已知父区间增量 Δ，左半区增量为 Δ/2+sqrt(L_s/4)·Z(node)，右半区增量为 Δ−左半区增量；Z 由下述固定 SplitMix64 + Box–Muller 键规则（seed、epoch、轴、节点相对起点和层级）产生。沿根至 relative_ns 的 dyadic 分割累加已覆盖子区间增量，最多 63 层，得 W(relative_ns)；**不保存**中间节点、先前偏置或 IMU 样本。布朗增量的方差以秒为单位，任意固定纳秒节点的零均值协方差遵循 Cov(W(s),W(t))=min(s,t)（秒）。

v1 的伪随机位流固定使用 SplitMix64 置换：先对 64 位无符号键加 0x9E3779B97F4A7C15，依次令 x=(x xor (x>>30))*0xBF58476D1CE4E5B9、x=(x xor (x>>27))*0x94D049BB133111EB、x=x xor (x>>31)，全部按 64 位模 2^64 运算。键的混合顺序固定为：令 key=seed，依次对固定域标签（白噪声=1、布朗桥根=2、布朗桥内部结点=3）、配置事件 insertion_order、sensor_id、轴号(0..5)、采样时刻或结点起点的整数纳秒、结点层级、独立高斯分量编号，执行 key=SplitMix64(key xor SplitMix64(该字段的 uint64_t 位模式))。白噪声使用请求时刻；布朗桥使用结点起点和层级；不同域和轴具有不同键空间。分别以高斯分量编号 0、1 构造两个置换输出，高 53 位整数 m 映射为 u=(m+0.5)·2^(-53)，产生开区间 (0,1) 的 U1/U2，采用 Box–Muller Z=√(−2 ln U1) cos(2πU2)。这一定义固定了 v1 的随机过程家族与索引规则；受标准数学库浮点实现差异影响，**仅承诺同工具链/同构建的逐值可复现**，不承诺跨 libc/架构 bitwise 相同。

**v1 冻结的 99% 卡方统计验收合同**（V06-S；功能正确性验收，不属于延期的 NFR-02 性能验收）：

| 参数 | 冻结值 |
| --- | --- |
| 独立随机种子 | 8 个：整数 0、1、2、3、4、5、6、7 |
| 采样间隔 | \(\Delta t=5{,}000{,}000\) ns = 0.005 s；IMU nominal_rate_hz=200，初帧同样使用 0.005 s |
| 白噪声样本 | 每种子、每轴 4096 个连续输出，\(t_k=k\Delta t,\ k=0,\ldots,4095\)；设随机游走强度和初始偏置为零 |
| 随机游走样本 | 每种子、每轴采 \(t_k=k\Delta t,\ k=0,\ldots,4096\) 的 4097 个样本，取 4096 个相邻偏置增量；设白噪声密度和初始偏置为零 |
| 陀螺仪强度 | 白噪声 \(N_g=0.001\ {\rm rad/s}/\sqrt{\rm Hz}\)；游走 \(K_g=0.0001\ {\rm rad/s^2}/\sqrt{\rm Hz}\) |
| 加速度计强度 | 白噪声 \(N_a=0.01\ {\rm m/s^2}/\sqrt{\rm Hz}\)；游走 \(K_a=0.001\ {\rm m/s^3}/\sqrt{\rm Hz}\) |
| 四个独立验收组 | gyro white、accel white、gyro walk、accel walk，**每组汇总** 8 种子 × 3 轴 × 4096 样本 |
| 每组样本量/自由度 | \(n=98304,\ \nu=n-1=98303\) |
| 样本方差 | \(s^2=\sum_{i=1}^{n}(x_i-\bar x)^2/(n-1)\)，其中白噪取输出减理想值、游走取相邻输出差（在对应模式隔离另一个噪声项后） |
| 理论方差 | 白噪 \(\sigma_0^2=N^2/\Delta t\)；游走增量 \(\sigma_0^2=K^2\Delta t\) |
| 判定统计量 | \(T=\nu s^2/\sigma_0^2\)；各组独立要求 \(\chi^2_{\nu,0.005}\le T\le\chi^2_{\nu,0.995}\) |
| 固定数值参考阈值 | 对 \(\nu=98303\)，下界 **97164.6270**、上界 **99448.8862**（四舍五入展示；实现以所给卡方分位定义为准） |
| 通过条件 | 四组**全部**通过各自的双侧 99% 方差区间，同时通过 V06-D 的禁噪无噪、固定种子逐元素重放及参数真实生效检查 |

白噪和偏置游走**分别隔离测试**，不能把含布朗偏置的总输出直接作为白噪独立样本。随机游走增量由连续时间布朗运动定义，固定采样步长下协方差与上述理论方差一致。四次分别 99% 检验并不声称联合 99% 家族置信度；若今后需要联合错误率控制，须升版约定，不擅自修改上述冻结阈值。这里的独立样本指不同种子/独立轴流和布朗运动不重叠增量。另行验证相同工具链和相同调用序列的固定种子逐元素重放（V06-D），不使用统计区间代替确定性验证。

## 7. Room 真实求交与可见性

场景只有 Room，其六个有限面满足：

- x=0、x=depth；
- y=0、y=width；
- z=0、z=height。

给定输入时间 t、当时存在且可按名称找到的相机 C 和相机系单位视线 u_C，**先调用历史解析求值得到 t 的 p_WB/R_WB 及该相机当时 T_BC/K/畸变**，计算：

$$
d_W=R_{WC}u_C,\qquad P_W(\lambda)=c_W+\lambda d_W,\quad\lambda>0.
$$

对每个候选面，在射线方向该轴非零时求 λ=(plane−c_axis)/d_axis；过滤非正 λ、平行、有限面二维边界之外的点，使用最小有效 λ 作为真实首次命中。**v1 固定容差**：方向平行阈值 1e−12（无量纲），正向距离阈值 max(1e−9 m, Room 对角线长度×1e−12)，有限边界容差 Room 对角线长度×1e−12；端点是否有效按 §2.4 的包含等号原则。棱边/角点等距命中按稳定面名先后 (x0,xDepth,y0,yWidth,z0,zHeight) 决定 surface_name，世界坐标仍是同一个几何点。

须验证视线三个分量有限且 |‖u_C‖−1|≤1e−9；超过容差的非单位向量**报错而不是悄悄归一化**。u_C 的 z 分量须为正。将 u_C 投影为归一化坐标 (u_x/u_z,u_y/u_z)，使用**当时相机的完整内参和径向切向畸变**投影到 (px,py)，只在 0≤px<width、0≤py<height 时视为有效成像视域。有效视域以畸变后像素为准，而非未畸变针孔投影。Room 渲染的遮挡规则必须和此求交的最近表面选择一致。返回列表与输入 vector 索引一一对应：每个结果有命中与否、世界坐标、射线距离、面标识及失败原因；不得把离散绘制网格顶点误当成唯一真实交点。

**Raycast 失败码与优先级（v1 冻结）**：

| 检查顺序 | 条件 | Result.status 或 RayHit.failure |
| --- | --- | --- |
| 整体 1 | at_ns 不在允许时间域或运动状态不能求值 | 整体 TimeOutOfRange |
| 整体 2 | 指定 camera_name 从未注册、不是该时刻有效的相机名 | 整体 CameraNotFound；若相机身份存在但该时刻尚未创建则整体 NoCameraAtTime |
| 整体 3 | 该时刻场景/相机标定不可用、内部非预期异常 | 整体 InvalidCalibration / InternalError |
| 单条 1 | 某条 u_C 含 NaN/Inf、零向量或 \(|\|u_C\|-1|>10^{-9}\) | 对应输入索引 InvalidBearing |
| 单条 2 | 单位视线 z≤0、畸变投影非有限、或畸变后像素在有效成像区域外 | 对应输入索引 OutsideFov |
| 单条 3 | 有效视线在 Room 上没有正向有限表面交点 | 对应输入索引 NoIntersection |
| 单条 4 | 所有检查通过，存在最近有限面有效正向交点 | 对应输入索引 Ok，hit=true |

**先整体验证，再按每条视线的单条 1→4 顺序判定**，首个适用失败原因获胜。**当 bearings_C.size()==0 时，也必须先校验 time_ns 与指定 camera_name：若有效，则返回 Ok 和长度为 0 的 std::vector<RayHit>，不执行任何射线计算；若时间/相机无效，则按上表返回对应整体错误，不能因为输入为空而跳过全局校验。**整体失败返回无 value；单条失败只填该 RayHit 的 index、hit=false、failure，其他几何字段不具备有效含义。只要整体校验成功，即使所有输入视线单条失败也返回 Result.status=Ok 和与输入长度相等的 RayHit 向量；不存在“所有未命中就整体 NoIntersection”的特例。对于时间下无该名称却历史上曾存在的改名相机，按该时刻名称不可解析返回 CameraNotFound，避免把重命名错当作未创建。

**Raycast 不依赖任何跨调用位姿缓存，也不要求 t 被 Generate 用过**；可以反复读过去 t，不能影响当前运动/配置历史和任何传感器序号。

## 8. 采样门控、时间严格递增与帧编号

每次 Generate(at_ns, gate) 执行以下事务：

~~~text
1. 若 camera=false 且 imu=false -> 空成功；无副作用
2. 用事件历史求 at_ns 当时的相机集合、名称、标定、IMU 标定、Room
3. 检查相机与 IMU 所有被启用传感器各自的 last_successful_ns
   at_ns == last_successful_ns -> DuplicateTimestamp
   at_ns <  last_successful_ns -> OutOfOrderTimestamp
4. 检查有效时间及参数，在函数局部临时生成所有图像、IMU 观测
5. 全部成功后，一次性提交每传感器 last_successful_ns 与 next_frame_number
6. 返回所有当时有效相机（按不可变 ID）及可选 imu0 的帧
任一错误则不提交、不分配帧号、不变更随机历史或配置
~~~

每个 SensorIdentity 的 next_frame_number 从 0 开始；相机改名不重置编号，新增相机从 0 开始。事件时间与采样时间是两个独立概念：事件定义物理规律，采样请求只读取规律并输出帧。同一时间不同传感器可分别被请求，例如先 camera，后 imu；同一时间再次请求已成功生成的相机则拒绝。

~~~text
Generate(0ns, camera=true, imu=false)  -> cam0#0 (以及其余当时相机)
Generate(0ns, camera=false,imu=true)   -> imu0#0 (合法)
Generate(0ns, camera=true, imu=false)  -> DuplicateTimestamp
Generate(5ms, camera=true,imu=true)    -> 新帧
Generate(2ms, camera=true,imu=false)   -> OutOfOrderTimestamp
~~~

若不同相机因历史注册时间不同，调用时只对该时刻存在的相机进行输出/防重，不得给未来才创建的相机分配帧。ResetSamplingSession 只重置全部传感器的 last_successful_ns、next_frame_number；不删除已配置的运动事件历史。若还需恢复 seed 与噪声演化，按配置时间函数确定性重算，不储存旧样本。

## 9. 配置文件、导出和 Linux 安装

### 9.1 正式 YAML schema v1（设计冻结）

v1 顶层字段固定为 schema_version、baseline、events、可选 sampling。**时间 t=0 的唯一确定顺序冻结为：先初始化内建默认值 → 再应用 baseline → 最后按 YAML 数组原顺序依次应用所有 timestamp_ns=0 的 events**；后续时间按纳秒升序处理，禁止事件时间回退。baseline 中明确给出的字段按固定顺序 room → pose → imu 转成 t=0 的合成变更事件，赋予首批 insertion_order（从 0 递增）；被省略的 baseline 字段使用内建默认值，不生成合成事件。然后对 events（包含同刻 t=0）依数组顺序继续分配 insertion_order，数值严格递增。**同一字段如 baseline 和 t=0 event 均给出，则后面的 event 覆盖 baseline**，结果与先在函数 API 设置 baseline 再按事件列表顺序调用完全一致。Begin/End 不得在 baseline 内出现。

events 数组中每条**状态修改事件**都需要 timestamp_ns、op、value（Begin/End 除外，不得填 value）。配置与函数式事件的语义必须一致，GetChangeHistory 全量和区间重载都能见到显式 baseline 对应的合成事件。

以下 YAML 为**符合当前 v1 合同的完整样例**：

~~~yaml
schema_version: 1
baseline:
  room:
    depth_m: 10.0
    width_m: 10.0
    height_m: 3.0
  pose:
    position_W_m: [5.0, 5.0, 1.5]
    quaternion_WB_wxyz: [1.0, 0.0, 0.0, 0.0]
  imu:
    T_BS:
      quaternion_BS_wxyz: [1.0, 0.0, 0.0, 0.0]
      translation_B_m: [0.0, 0.0, 0.0]
    nominal_rate_hz: 200.0
    gyroscope_noise_density: 0.001
    gyroscope_random_walk: 0.0001
    accelerometer_noise_density: 0.01
    accelerometer_random_walk: 0.001
    initial_gyro_bias_S: [0.0, 0.0, 0.0]
    initial_accel_bias_S: [0.0, 0.0, 0.0]
    noise_enabled: false
    seed: 1234
events:
  - timestamp_ns: 0
    op: SetLinearMotion
    value:
      mode: ConstantVelocity
      velocity_W_mps: [0.25, 0.0, 0.0]
  - timestamp_ns: 0
    op: CreateCamera
    value:
      T_BC:
        quaternion_BC_wxyz: [1.0, 0.0, 0.0, 0.0]
        translation_B_m: [0.0, 0.0, 0.0]
  - timestamp_ns: 1000000000
    op: BeginMotionMode
  - timestamp_ns: 1000000000
    op: SetAngularMotion
    value:
      mode: FixedAxisConstantRotation
      axis_B: [0.0, 0.0, 1.0]
      rate_radps: 0.35
  - timestamp_ns: 2000000000
    op: EndMotionEnd
sampling:
  camera_times_ns: [0, 50000000, 100000000]
  imu_times_ns: [0, 5000000, 10000000]
~~~

v1 操作与 value 字段约束：

| op | value 的必需字段 | 行为 |
| --- | --- | --- |
| BeginMotionMode / EndMotionEnd | 不得出现 value | 入栈 / 出栈 |
| SetPose | position_W_m[3]、quaternion_WB_wxyz[4] | 显式重设绝对位姿 |
| SetLinearMotion | mode + 模式字段 | Stationary 无附加值；ConstantVelocity: velocity_W_mps[3]；ConstantAcceleration: initial_velocity_W_mps[3]、acceleration_W_mps2[3]；**UniformCircular: center_W_m[3]、radius_m、normal_W[3]、phase_reference_W[3]、phase_rad、rate_radps、reactivation_policy**（ReanchorOnReactivation 或 FixedWorldCenter） |
| SetAngularMotion | mode + 模式字段 | Stationary 无附加值；FixedAxisConstantRotation: axis_B[3]、rate_radps |
| CreateCamera | T_BC（quaternion_BC_wxyz[4]、translation_B_m[3]） | 自动命名为 camX |
| RenameCamera | camera_id、new_name | 改相机显示名，sensor id 不变 |
| SetCameraCalibration | camera_id、T_BC、K_row_major[9]、width、height、radtan[4]、nominal_rate_hz | 完整替换指定相机标定 |
| SetImuCalibration | 同 baseline.imu 结构与全部字段 | 完整替换 IMU 标定及噪声 epoch |
| SetRoom | depth_m、width_m、height_m | 改 Room 参数，历史时刻按对应版本重算 |

UniformCircular 模式的 YAML 值示例（该独立片段可放入某条 SetLinearMotion 事件的 value 内；首次生效位置必须落在所声明圆周上）：

~~~yaml
mode: UniformCircular
center_W_m: [5.0, 5.0, 1.5]
radius_m: 2.0
normal_W: [0.0, 0.0, 1.0]
phase_reference_W: [1.0, 0.0, 0.0]
phase_rad: 0.0
rate_radps: 0.5
reactivation_policy: ReanchorOnReactivation
~~~

除首次激活时的圆心/位姿一致性外，重入时按 §4.2 的 reactivation_policy 决定是否允许重新锚定，配置解析不得丢弃声明圆心、相位或策略。

如果未显式创建相机，默认从 t=0 存在虚拟 cam0，T_BC=(I3,0)，且 K/分辨率/零畸变/标称频率完全采用 §5.1 的冻结默认值。**只有此前未修改默认 cam0 的标定/名称、也未产生传感器帧时**，第一条 t=0 的 CreateCamera 才会取代未使用的虚拟默认 cam0；其他情形保留 cam0 并为新相机分配下一个不冲突的 camX（见 §5.1）。所有仅提供 T_BC 的 CreateCamera 也采用同一默认 K、752×480、radtan=(0,0,0,0)、20 Hz，允许随后以完整 SetCameraCalibration 覆盖；因此 YAML 样例中的相机可在未另设内参的情况下立即生成图像。baseline 允许省略 pose/room/imu 的整项，此时分别采用零位姿、Room (10m,10m,3m)、IMU 单位外参与无噪默认参数（标称频率 200Hz，其余噪声和初始偏置为 0，seed=0）。baseline 内一旦出现某项对象，则该对象的所有规定字段必填，避免半更新含义不明确。

**验证与事务规则：**

1. schema_version 必须是整数 1，其他版本报 InvalidConfigSchema；未知顶层键、未知 op、重复 YAML map key、非法数量、非有限数、非单位旋转/法向、同一时间的名称冲突均拒绝。
2. timestamp_ns、sampling 中的 *_times_ns 必须是 YAML 十进制**整数**，在 std::int64_t 非负范围，禁止浮点数、科学计数法、溢出或字符串冒充纳秒。baseline 在 t=0 **始终先于 events**，按 room→pose→imu 固定顺序合成历史事件；events 时间须非递减，同刻以数组顺序确定其后续 insertion_order（§9.1 开头已冻结覆盖规则）。
3. quaternion 顺序固定 **w,x,y,z**；位置/平移 [x,y,z]，K_row_major 是 3×3 按行展开的 9 个数，radtan 为 [k1,k2,p1,p2]。imuread 读取的 noise density 单位由 §6.2 固定。
4. LoadMotionConfig(path) 仅允许用于**尚无显式业务事件且尚未输出帧**的新实例；整个文档先解析、验证、重建局部状态，成功才整体提交，任一错误则原对象完全不变。错误 message 包含可读 YAML 路径（如 events[3].value.axis_B），调用者以 Status 判断错误类别。
5. sampling 是**CLI 导出请求计划**，不是运动/标定修改事件；库的 LoadMotionConfig 只验证其结构并不隐式生成数据或写文件。新建模拟器若在未显式设置相机参数时直接按 sampling 生成图像，则输出默认尺寸 752×480、像素格式 CV_8UC1，必须使用 §5.1 的默认标定；不允许因为省略 K/width/height 而使用未初始化值。CLI 将两个 *_times_ns 合并、升序调度，每时刻为对应门控合并一次 Generate。不同传感器可同时间采样，单一传感器时间序列必须严格递增。
6. 读取和写出的配置应保留事件顺序、标定数值及版本标记；库内事件历史 ChangeRecord 的 canonical_payload_yaml 可使用这里定义的 value 同构序列化，不得把计算出的图像或位姿追加进历史。

### 9.2 CMake / 安装行为

最终在目标仓库创建 SHARED 库并安装到 CMAKE_INSTALL_PREFIX 下，导出公开头文件、库、VisualSimConfig.cmake、VisualSimTargets.cmake，提供消费端 find_package(VisualSim CONFIG REQUIRED) 和 target_link_libraries(app PRIVATE VisualSim::VisualSim) 的例子。依赖只含实际使用的非 ROS 库；CMake 在 Windows 上明示配置失败，移除 Windows 运行/构建入口。

**不需要也不允许以“Ubuntu 24.04 默认工具链太老”为理由强制降低 CMake、GCC 或 C++ 标准要求**。维持用户现有可用的构建链，并在文档中列明实际选用版本。纳秒时间字段进行 std::is_same_v<TimestampNs,std::int64_t> 等编译期检查。

## 10. 验证责任与追踪

本 LLD 只固定实现合同和算法边界，**不在此维护完整测试用例表**。V01–V13 的测试定义、FR/NFR/AC→LLD/V/P 对应矩阵、验收命令、阶段状态与风险见 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md)。其中 **V12 性能验收已延期至首次成功运行程序之后再决定**，不作为当前功能交付的完成条件。

设计验收不等于代码已实现或运行成功。当前所有接口、噪声算法和 YAML 版本均为**设计合同**；本轮未修改生产代码、未构建或运行测试。
