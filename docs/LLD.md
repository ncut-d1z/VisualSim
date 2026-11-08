# VisualSim 运行时库详细设计（LLD）

> 本文为待实施的 **v1 详细设计合同**，不是已实现或已通过测试的证明。最终功能以 [SRS.md](SRS.md) 为准，架构见 [HLD.md](HLD.md)，工程实施/代码迁移见 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md)。
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
struct LinearUniformCircular {
  Eigen::Vector3d radius_W_m;   // 相对生效位置的初始有向半径
  Eigen::Vector3d normal_W;     // 单位法向，与半径正交
  double rate_radps;            // 有符号轨道角速率
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
  Eigen::Isometry3d T_BC;       // 相机 C -> 载体 B
  Eigen::Matrix3d K;
  int width;
  int height;
  std::array<double,4> radtan;  // k1,k2,p1,p2
  double nominal_rate_hz;
};
struct ImuCalibration {
  Eigen::Isometry3d T_BS;       // IMU S -> 载体 B
  double nominal_rate_hz;
  double gyroscope_noise_density;
  double gyroscope_random_walk;
  double accelerometer_noise_density;
  double accelerometer_random_walk;
  Eigen::Vector3d initial_gyro_bias_S;
  Eigen::Vector3d initial_accel_bias_S;
  bool noise_enabled;
  std::uint64_t seed;
};
struct RoomConfig { double depth_m, width_m, height_m; };
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
      TimestampNs effective_ns, const Eigen::Isometry3d& T_BC);
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

- **所有公开可失败操作统一返回 Result<T> 或 Result<void>**，其中成功时 status=Ok；非 void 成功必有 value，失败时没有 value 且必须提供非空 message。Result<void> 不具有 value。
- Status 枚举名称及上述显式数值在 v1 ABI 主版本内稳定；新增错误码不可更改已有数值，破坏性变更必须升级主版本；内部异常被翻译为 InternalError（附诊断），已定义的非法输入用对应类型码。状态码用于程序分支，message 只供人阅读，不保证可解析文案稳定。
- 仅不可恢复的内存分配异常 std::bad_alloc 允许越过库边界向同工具链 C++ 调用者传播；所有事件提交、生成和配置导入均满足**强异常保证**：成功前状态不变，异常也不能留下部分结果。
- 全部只读接口无状态改变；未知的时间、相机、非法单位视线统一用 Result 错误或 RayHit.failure（整个调用失败与逐条无命中区分）。最外层的空门控 Generate 返回 status=Ok、空 frames。
- Status 的对外语义详见 §8（重复/回退）、§7（无交点）、§3（事件时序）和 §9（配置）。

### 2.2 线程安全与多实例

同一个 Simulator 实例**不提供内部同步**。用户必须串行调用 Set*、Begin/End、LoadMotionConfig、Generate、ResetSamplingSession，也必须避免它们与 Get*/Raycast/GetChangeHistory 并发；在无写入的期间，多个只读查询可并行，但调用者仍须确保实例活着且未发生移动/销毁。不同 Simulator 实例之间不共享可变历史、计数器或噪声状态，可以并发运行。回调/日志不允许在内部持锁期间调用用户代码（当前 v1 不提供回调式接口）。

### 2.3 数据所有权、生命周期与二进制兼容边界

- Simulator 用 Pimpl（Impl 在 .cpp 定义）隐藏内部状态；公开值类型是**值语义**。GetChangeHistory、ListCameras、Raycast 和 Generate 返回独立 std::vector；返回体在 Simulator 销毁后仍有效。
- 图像结果中的 cv::Mat 由每次生成单独分配，返回时转移/共享该帧独立的 OpenCV 引用计数缓冲；后续 Generate 不重用可写底层存储，调用者可安全持有或释放。若要完全独立复制可在调用方 clone；库不保存历史 cv::Mat。
- Pimpl 只减少布局暴露，**不提供跨编译器、libstdc++ 或 OpenCV/Eigen 版本的稳定二进制 ABI**。初始 API/SONAME 主版本为 1；破坏公开类型/布局/异常 ABI 时须升级 SONAME 和 CMake package 主版本。目标只保证相同兼容 Linux ABI/工具链及依赖版本的消费者可运行。
- 输出时间、名称、帧号等属于返回对象所有，不借用临时字符串指针；仅在 RayHit.hit=true 时读取 point_W_m、range_m 和 surface_name。

### 2.4 时间域及参数合法性

有效查询纳秒时间范围为 0 ≤ t ≤ INT64_MAX；事件时间必须非负且按非递减顺序追加，查询可读取任意有效历史时间，运动计算如出现非有限结果返回 TimeOutOfRange/InvalidArgument。时间戳与纳秒间隔内部保持 std::int64_t，浮点秒仅在解析求值计算边界使用，不能回算覆盖整数时间。坐标值必须有限，旋转矩阵满足 SO(3)（容差按测试计划确定），相机焦距和分辨率为正，噪声参数非负；非法配置整个操作失败。传感器 ID 稳定，不因 RenameCamera 更改。

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
                    LinearUniformCircular{radius_W, normal_W, rate});
sim.EndMotionEnd(3'000'000'000LL);

auto state = sim.GetMotionState(2'500'000'000LL);
auto hits  = sim.Raycast(2'500'000'000LL, bearings_C, "cam0");
~~~

这是按 v1 接口示意的调用片段，省略了每次 Result 的错误检查，**不是当前仓库已经能够编译的样例**。Begin/End 在相同时间允许零时长作用域，操作按稳定插入顺序执行。没有对应 Begin 的 End 返回 UnmatchedMotionScope；不自动生成临时作用域。

### 3.3 时间事件与历史保护

EventTimeline 的每条事件含 timestamp_ns:int64、单调递增的 insertion_order、类型、目标不可变 ID、值及对作用域的影响；当前配置可由完整事件流重建。配置文件解析必须转换为同一事件格式，保证顺序等价。

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

**匀速圆周**：指定世界系有向半径 r0（r=||r0||>0）、法向 n_W（单位，n·r0=0）、有符号速率 Ω。记 Q(θ)=Exp([n_W]×θ)，θ=ΩΔt：

$$
p_W(t)=p_0+(Q(\theta)-I)r_0,
$$

$$
v_W(t)=\Omega\,n_W\times Q(\theta)r_0,\qquad
a_W(t)=\Omega^2 n_W\times(n_W\times Q(\theta)r_0).
$$

段内圆心是 c=p0−r0。调用者指定世界圆心时，首次激活从 p0−c 得到 r0，并检查圆半径与平面约束。**旧圆周模式出栈恢复时使用保留的 r0/n/Ω，在出栈位置重新锚定**，新的有效世界圆心为 p_current−r0；如果调用者要求固定世界圆心且无法保持连续性，应拒绝有冲突的请求，不能无提示跳位姿。

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
3. UniformCircular 使用相对初始有向半径 radius_W_m、轨道法向和角速率；退出作用域重新启用旧圆周规范时，使用退出时当前 p 和保存的 radius_W_m 确定新的圆心 c=p−radius_W_m。如果用户请求固定旧世界圆心且违反连续性，则按 InvalidMode 拒绝而非隐式重定位。
4. SetPose(t,p,R) 是**唯一允许主动造成 p/R 不连续的业务事件**：它显式替换事件时刻的绝对姿态及位置，同时按当前有效模式重新锚定局部起点；恒加速模式的 initial_velocity_W_mps 重新作为新段初速度，圆周模式沿当前 radius_W_m 重建圆心。若调用者希望保持原速度，应在同一 t 的后续事件显式写入对应参数。
5. 查询恰好处于模式/SetPose 事件时间 t 时，返回在所有 t 事件执行**之后**的右侧位姿与相应模式解析导数；如左右导数不同或 SetPose 造成跳变，设置 derivative_discontinuous=true。段内支持的两个角模式角加速度恒为 0；不虚构切换冲量的有限角加速度。
6. 同一时刻多事件按提交顺序依次执行，若产生矛盾配置（例如零长度圆周半径、非正房间尺寸、非法 SO(3)）整个操作拒绝，原事件历史不变。

## 5. 动态相机及标定时间线

### 5.1 注册、默认相机与重命名规则

CameraRegistry 的 SensorId 稳定且不会重用，名称按时刻解析。自动名称固定依次 cam0、cam1、cam2……，更名不回收自动序号，也不更改此前帧的 id/编号。

**默认与显式创建的定稿规则（避免查询引起隐式状态突变）：**

- 起点 t=0 的初始配置视作有一台**虚拟默认** cam0（T_BC=(I3,0)），无需真正修改事件历史；只读 Raycast/ListCameras 不会“创建”对象。
- 如果第一条 CreateCamera 事件的生效时间恰为 0，且此前没有任何成功生成帧，则第一台显式相机**替代虚拟默认值、占用稳定 ID=0 和名称 cam0**，而非增加 cam1；这是仿真初始化阶段的注册。
- 如果第一条 CreateCamera 的生效时间大于 0，则此前的虚拟默认 cam0 从 t=0 起存在，新相机自该事件时刻成为 cam1。这样过去历史与无副作用查询有确定结果。
- 若已在 t=0 成功生成默认 cam0 的帧，则 t=0 的后续 CreateCamera 被 EventHistoryFrozen 拒绝，不能将已输出传感器身份替换。
- 后续每次注册用下一个自动 ID/名称；若自动名称与调用者先前 RenameCamera 形成占用冲突，必须明确返回 DuplicateCameraName，不能覆盖或重用其他相机身份。
- RenameCamera(t,id,new_name) 禁止空名称、与该时刻其他相机同名；修改只影响 t 以后，旧历史使用旧名。后续生成使用新名，但同一 SensorId 的时间去重和 frame_number 保持连续。

事件历史包括所有显式 CreateCamera、RenameCamera 和标定变更。GetCameraCalibration(t,id)、ListCameras(t)、Raycast(t,name) 查询只采纳 t 时有效的定义，不依赖此前 Generate 是否调用。

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

已知状态 p_WB、R_WB、a_WB、ω_B、α_B，IMU 到载体的 T_BS=(R_BS,t_BS)，世界重力 g_W=(0,0,−9.81) m/s²（可配置）。

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

给出确定性且**不保存跨查询中间状态**的实现方案：对 0..2^63 ns 的时间域使用固定二叉 dyadic Brownian-bridge 树。根区间总增量为 sqrt(T_s)·Z(root)；每个区间长度 L_s 已知父区间总增量 Δ，则左半区增量由 Δ/2+sqrt(L_s/4)·Z(node) 给出，右半区增量为 Δ−左半区增量；Z 由固定 counter-based 64-bit hash（SplitMix64 与 Box–Muller，按以下固定规则定义）按 seed、epoch、轴和节点索引确定。沿根至目标纳秒时刻的路径求和，最多 63 层，得到可重现 W(t)；无需缓存任何 IMU 样本或偏置状态。布朗桥保证时间一致的增量协方差。生成时仅保留局部运算中间量。

v1 的伪随机位流固定使用 SplitMix64 置换：先对 64 位无符号键加 0x9E3779B97F4A7C15，依次令 x=(x xor (x>>30))*0xBF58476D1CE4E5B9、x=(x xor (x>>27))*0x94D049BB133111EB、x=x xor (x>>31)，全部按 64 位模 2^64 运算。键的混合顺序固定为：令 key=seed，依次对固定域标签（白噪声=1、布朗桥根=2、布朗桥内部结点=3）、配置事件 insertion_order、sensor_id、轴号(0..5)、采样时刻或结点起点的整数纳秒、结点层级、独立高斯分量编号，执行 key=SplitMix64(key xor SplitMix64(该字段的 uint64_t 位模式))。白噪声使用请求时刻；布朗桥使用结点起点和层级；不同域和轴具有不同键空间。分别以高斯分量编号 0、1 构造两个置换输出，高 53 位整数 m 映射为 u=(m+0.5)·2^(-53)，产生开区间 (0,1) 的 U1/U2，采用 Box–Muller Z=√(−2 ln U1) cos(2πU2)。这一定义固定了 v1 的随机过程家族与索引规则；受标准数学库浮点实现差异影响，**仅承诺同工具链/同构建的逐值可复现**，不承诺跨 libc/架构 bitwise 相同。

噪声统计验收：对固定间隔、独立种子的样本，白噪样本方差与 N²/Δt、随机游走差分方差与 K²Δt 对照；使用预设 99% 卡方方差置信区间判定，并另测固定 seed 的逐元素完全重放。此检验必须在独立实验样本上进行，不允许以单个随机点的输出大小推断方差达标。性能预算与跨标准库 bitwise 一致性不在本版承诺内。

## 7. Room 真实求交与可见性

场景只有 Room，其六个有限面满足：

- x=0、x=depth；
- y=0、y=width；
- z=0、z=height。

给定输入时间 t、当时存在且可按名称找到的相机 C 和相机系单位视线 u_C，**先调用历史解析求值得到 t 的 p_WB/R_WB 及该相机当时 T_BC/K/畸变**，计算：

$$
d_W=R_{WC}u_C,\qquad P_W(\lambda)=c_W+\lambda d_W,\quad\lambda>0.
$$

对每个候选面，在射线方向该轴非零时求 λ=(plane−c_axis)/d_axis；过滤非正 λ、平行、有限面二维边界之外的点，使用最小有效 λ 作为真实首次命中。建议固定容差：方向平行阈值 1e−12，正向距离阈值 max(1e−9 m, Room 对角线长度×1e−12)，有限边界容差 Room 对角线长度×1e−12。棱边/角点等距命中按稳定面名先后 (x0,xDepth,y0,yWidth,z0,zHeight) 决定 surface_name，世界坐标仍是同一个几何点。

须验证视线三个分量有限且 |‖u_C‖−1|≤1e−9；超过容差的非单位向量**报错而不是悄悄归一化**。u_C 的 z 分量须为正。将 u_C 投影为归一化坐标 (u_x/u_z,u_y/u_z)，使用**当时相机的完整内参和径向切向畸变**投影到 (px,py)，只在 0≤px<width、0≤py<height 时视为有效成像视域。有效视域以畸变后像素为准，而非未畸变针孔投影。Room 渲染的遮挡规则必须和此求交的最近表面选择一致。返回列表与输入 vector 索引一一对应：每个结果有命中与否、世界坐标、射线距离、面标识及失败原因；不得把离散绘制网格顶点误当成唯一真实交点。

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

v1 顶层字段固定为 schema_version、baseline、events、可选 sampling。基线在仿真时间 0 生效；events 数组中的每条**状态修改事件**都需要 timestamp_ns、op、value（Begin/End 除外，不得填 value）。配置与函数式事件的语义必须一致。

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
| SetLinearMotion | mode + 模式字段 | Stationary 无附加值；ConstantVelocity: velocity_W_mps[3]；ConstantAcceleration: initial_velocity_W_mps[3]、acceleration_W_mps2[3]；UniformCircular: radius_W_m[3]、normal_W[3]、rate_radps |
| SetAngularMotion | mode + 模式字段 | Stationary 无附加值；FixedAxisConstantRotation: axis_B[3]、rate_radps |
| CreateCamera | T_BC（quaternion_BC_wxyz[4]、translation_B_m[3]） | 自动命名为 camX |
| RenameCamera | camera_id、new_name | 改相机显示名，sensor id 不变 |
| SetCameraCalibration | camera_id、T_BC、K_row_major[9]、width、height、radtan[4]、nominal_rate_hz | 完整替换指定相机标定 |
| SetImuCalibration | 同 baseline.imu 结构与全部字段 | 完整替换 IMU 标定及噪声 epoch |
| SetRoom | depth_m、width_m、height_m | 改 Room 参数，历史时刻按对应版本重算 |

若不含显式 CreateCamera，自动使用默认 cam0，T_BC=(I3,0)；若在 t=0 使用 CreateCamera，首台显式相机占 cam0 身份，详见 §5.1。baseline 允许省略 pose/room/imu 的整项，此时分别采用零位姿、Room (10m,10m,3m)、IMU 单位外参与无噪默认参数（标称频率 200Hz，其余噪声和初始偏置为 0，seed=0）。baseline 内一旦出现某项对象，则该对象的所有规定字段必填，避免半更新含义不明确。

**验证与事务规则：**

1. schema_version 必须是整数 1，其他版本报 InvalidConfigSchema；未知顶层键、未知 op、重复 YAML map key、非法数量、非有限数、非单位旋转/法向、同一时间的名称冲突均拒绝。
2. timestamp_ns、sampling 中的 *_times_ns 必须是 YAML 十进制**整数**，在 std::int64_t 非负范围，禁止浮点数、科学计数法、溢出或字符串冒充纳秒。events 按文档顺序，时间戳须非递减；相同时刻以数组顺序确定 insertion_order。
3. quaternion 顺序固定 **w,x,y,z**；位置/平移 [x,y,z]，K_row_major 是 3×3 按行展开的 9 个数，radtan 为 [k1,k2,p1,p2]。imuread 读取的 noise density 单位由 §6.2 固定。
4. LoadMotionConfig(path) 仅允许用于**尚无显式业务事件且尚未输出帧**的新实例；整个文档先解析、验证、重建局部状态，成功才整体提交，任一错误则原对象完全不变。错误 message 包含可读 YAML 路径（如 events[3].value.axis_B），调用者以 Status 判断错误类别。
5. sampling 是**CLI 导出请求计划**，不是运动/标定修改事件；库的 LoadMotionConfig 只验证其结构并不隐式生成数据或写文件。CLI 将两个 *_times_ns 合并、升序调度，每时刻为对应门控合并一次 Generate。不同传感器可同时间采样，单一传感器时间序列必须严格递增。
6. 读取和写出的配置应保留事件顺序、标定数值及版本标记；库内事件历史 ChangeRecord 的 canonical_payload_yaml 可使用这里定义的 value 同构序列化，不得把计算出的图像或位姿追加进历史。

### 9.2 CMake / 安装行为

最终在目标仓库创建 SHARED 库并安装到 CMAKE_INSTALL_PREFIX 下，导出公开头文件、库、VisualSimConfig.cmake、VisualSimTargets.cmake，提供消费端 find_package(VisualSim CONFIG REQUIRED) 和 target_link_libraries(app PRIVATE VisualSim::VisualSim) 的例子。依赖只含实际使用的非 ROS 库；CMake 在 Windows 上明示配置失败，移除 Windows 运行/构建入口。

**不需要也不允许以“Ubuntu 24.04 默认工具链太老”为理由强制降低 CMake、GCC 或 C++ 标准要求**。维持用户现有可用的构建链，并在文档中列明实际选用版本。纳秒时间字段进行 std::is_same_v<TimestampNs,std::int64_t> 等编译期检查。

## 10. 验证责任与追踪

本 LLD 只固定实现合同和算法边界，**不在此维护完整测试用例表**。V01–V13 的测试定义、FR/NFR/AC→LLD/V/P 对应矩阵、验收命令、阶段状态与风险见 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md)。其中 **V12 性能验收已延期至首次成功运行程序之后再决定**，不作为当前功能交付的完成条件。

设计验收不等于代码已实现或运行成功。当前所有接口、噪声算法和 YAML 版本均为**设计合同**；本轮未修改生产代码、未构建或运行测试。
