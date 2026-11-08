# VisualSim 运行时库详细设计（LLD）

> 本文是设计草案，尚未实施。最终功能以 [SRS.md](SRS.md) 为准，架构见 [HLD.md](HLD.md)，工程实施/代码迁移见 [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md)。
>
> 本文中的文件名、类型和函数签名是**拟议实现**，并不表示当前仓库已有对应 API。唯一可写目标仓库为 ncut-d1z/VisualSim。来源仓库 ncut-d1z/ros2-vio-node 只读。平台 Ubuntu 24.04，禁止 Windows 构建；不强制降级构建工具链。

## 1. 必须遵循的存储边界

**允许长时间保留的业务数据**仅为：当前运动类别/模式/参数/作用域栈、当前相机/IMU/Room 配置、调用者对此进行每次修改的**带纳秒时间戳事件历史**。这些数据足以重建任意有效时刻的物理状态。

不得保留上一设计中的 PoseCache、StereoPoseSnapshot、最近 64 组双目位姿、历史图片、历史 IMU 样本、历史射线命中、按帧运动状态或任何积分结果缓存。允许单次调用内部的临时局部对象；调用结束即释放。由于必须拒绝重复时间和连续编号，还需要各传感器最近成功生成时间与 next_frame_number 两项**最小协议控制变量**，不将其扩展为帧队列或结果索引。

本设计**不使用历史图像帧存在性**作为 Raycast 的前置条件；Raycast 通过事件历史按需计算查询时刻的位姿。

## 2. 公开接口草案

全部纳秒字段采用**完全相同的 std::int64_t** 类型。使用 Eigen 做坐标与矩阵，OpenCV cv::Mat 作为图像输出。以下仅表达公共合同，可按代码规范微调函数/类型布局。

~~~cpp
namespace visualsim {
using TimestampNs = std::int64_t;
static_assert(std::is_same_v<TimestampNs, std::int64_t>);

enum class LinearMode {
  Stationary, ConstantVelocity, ConstantAcceleration, UniformCircular
};
enum class AngularMode {
  Stationary, FixedAxisConstantRotation
};

struct LinearMotionSpec {
  LinearMode mode;
  Eigen::Vector3d velocity_W_mps;         // 匀速 / 匀加速初速度
  Eigen::Vector3d acceleration_W_mps2;    // 匀加速
  Eigen::Vector3d orbit_radius_W_m;       // 圆周生效瞬间的有向半径
  Eigen::Vector3d orbit_normal_W;         // 单位轨道法向
  double orbit_rate_radps;                // 有符号圆周角速率
};
struct AngularMotionSpec {
  AngularMode mode;
  Eigen::Vector3d axis_B;                // B 系单位轴
  double angular_rate_radps;             // 有符号角速率
};

struct MotionState {
  Eigen::Vector3d position_W_m;
  Eigen::Matrix3d rotation_WB;
  Eigen::Vector3d velocity_W_mps;
  Eigen::Vector3d acceleration_W_mps2;
  Eigen::Vector3d angular_velocity_B_radps;
  Eigen::Vector3d angular_acceleration_B_radps2;
  bool derivative_discontinuous = false; // 配置切换边界
};

struct CameraCalibration {
  Eigen::Isometry3d T_BC;   // 相机 C -> 载体 B
  Eigen::Matrix3d K;
  int width;
  int height;
  // k1,k2,p1,p2
  std::array<double, 4> radtan;
  double nominal_rate_hz;
};
struct ImuCalibration {
  Eigen::Isometry3d T_BS;    // IMU S -> 载体 B
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
struct RoomConfig {
  double depth_m;
  double width_m;
  double height_m;
};

struct SensorGate {
  bool camera = false;      // 当前已注册的所有相机
  bool imu = false;
};
struct ImuMeasurement {
  Eigen::Vector3d angular_velocity_S_radps;
  Eigen::Vector3d specific_force_S_mps2;
};
struct SensorFrame {
  TimestampNs timestamp_ns;
  std::string sensor_name;
  std::uint64_t frame_number;
  std::variant<cv::Mat, ImuMeasurement> payload;
};
struct SampleBatch { std::vector<SensorFrame> frames; };

struct RayHit {
  std::size_t input_index;
  bool hit;
  Eigen::Vector3d point_W_m;
  double range_m;
  std::string surface_name;
  std::string reason;
};

enum class Status {
  Ok, InvalidArgument, InvalidMode, InvalidTimestamp, EventOutOfOrder,
  EventHistoryFrozen, UnmatchedMotionScope, InvalidCalibration,
  CameraNotFound, DuplicateCameraName, NoCameraAtTime,
  DuplicateTimestamp, OutOfOrderTimestamp, TimeOutOfRange,
  NoIntersection, UnsupportedAngularAcceleration, InternalError
};
template<class T> struct Result {
  Status status;
  std::optional<T> value;
  std::string message;
};

struct ChangeRecord {
  TimestampNs effective_time_ns;
  std::uint64_t insertion_order;
  std::string object_kind; // motion/scope/camera/imu/room/pose
  std::string operation;
  // typed or serialized new value, no computed frame/pose results
};

class Simulator {
public:
  Status BeginMotionMode(TimestampNs effective_ns);
  Status EndMotionEnd(TimestampNs effective_ns);
  Status SetLinearMotion(TimestampNs effective_ns,
                         const LinearMotionSpec&);
  Status SetAngularMotion(TimestampNs effective_ns,
                          const AngularMotionSpec&);
  Status SetPose(TimestampNs effective_ns,
                 const Eigen::Vector3d& position_W_m,
                 const Eigen::Matrix3d& rotation_WB);

  Result<std::string> CreateCamera(TimestampNs effective_ns,
                                   const Eigen::Isometry3d& T_BC);
  Status RenameCamera(TimestampNs effective_ns,
                      std::string_view camera_id_or_name,
                      std::string_view new_name);
  Status SetCameraCalibration(TimestampNs effective_ns,
                              std::string_view camera_id_or_name,
                              const CameraCalibration&);
  Status SetImuCalibration(TimestampNs effective_ns,
                           const ImuCalibration&);
  Status SetRoom(TimestampNs effective_ns, const RoomConfig&);
  Result<CameraCalibration> GetCameraCalibration(
      TimestampNs at_ns, std::string_view camera_name) const;
  Result<ImuCalibration> GetImuCalibration(TimestampNs at_ns) const;

  Result<MotionState> GetMotionState(TimestampNs at_ns) const;
  std::vector<ChangeRecord> GetChangeHistory() const;
  Status LoadMotionConfig(std::string_view path); // 等效事件顺序

  Result<SampleBatch> Generate(TimestampNs at_ns, SensorGate);
  Result<std::vector<RayHit>> Raycast(
      TimestampNs at_ns,
      const std::vector<Eigen::Vector3d>& bearings_C,
      std::string_view camera_name = "cam0") const;
  void ResetSamplingSession();  // 不丢弃运动/标定修改历史
};
} // namespace visualsim
~~~

说明：CreateCamera(T_BC) 必须创建新相机；SetCameraCalibration/单独 SetCameraExtrinsics 用于修改**现有**相机而不是隐式替换为另一台。若需要完整的配置文件兼容，可加 CreateCamera(name, T_BC) 的可选名称重载，确保唯一性及自动编号规则。

数据修改接口的生效时间必须是 TimestampNs，不能只附一个没有生效时间的全局 setter。若提供便捷“立即修改”重载，也必须明确使用哪个纳秒时刻、写入同一事件历史。公开头文件可以用 Pimpl 隐藏不必要的第三方实现细节；库不暴露 ROS 类型。

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
sim.SetLinearMotion(0, LinearMotionSpec::ConstantVelocity(v_W));

sim.BeginMotionMode(1'000'000'000LL); // 1 秒入栈当前线/角模式
sim.SetAngularMotion(1'000'000'000LL,
                     AngularMotionSpec::ConstantRotation(axis_B, omega));
sim.SetLinearMotion(2'000'000'000LL,
                    LinearMotionSpec::UniformCircular(r_W, n_W, rate));
// 2~3 秒：圆周线运动 + 匀速转动角运动，线运动已覆盖原匀速直线
sim.EndMotionEnd(3'000'000'000LL);
// 3 秒起恢复入栈前的匀速直线 + 角静止；
// 当前位置/朝向连续，不跳回 1 秒时的位置/朝向。

auto state = sim.GetMotionState(2'500'000'000LL);
auto hits  = sim.Raycast(2'500'000'000LL, bearings_C, "cam0");
~~~

这是设计示意，并非已有可编译代码。Begin/End 在相同时间允许零时长作用域，操作按稳定插入顺序执行。没有对应 Begin 的 End 返回 UnmatchedMotionScope；不自动生成临时作用域。

### 3.3 时间事件与历史保护

EventTimeline 的每条事件含 timestamp_ns:int64、单调递增的 insertion_order、类型、目标不可变 ID、值及对作用域的影响；当前配置可由完整事件流重建。配置文件解析必须转换为同一事件格式，保证顺序等价。

默认只允许按**生效时间非递减**的顺序追加事件，同一时间戳可按调用顺序多次覆盖。已经成功生成任意传感器数据后，不接受在**任一已生成帧的时间戳及之前**新增/修改事件，避免历史重算改变旧输出；只读 GetMotionState/Raycast 可以查询历史时间，不会冻结配置也不会生成帧。

需要更改过去的历史时：显式 ResetSamplingSession 后重新创建/导入期望事件序列（或新建仿真实例）。重置采样序号不自动删除事件历史；另提供明确的 ClearConfiguration/ReplaceEventTimeline 操作时必须把副作用写入 API 文档。

## 4. 基于事件的分段解析运动重算

### 4.1 算法概述

求时间 t 的状态，不读取任何缓存的帧/位姿：

~~~text
Evaluate(t):
  validate t and event-time domain
  initialize state from baseline, modes=(linear stationary, angular stationary)
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

## 5. 动态相机及标定时间线

### 5.1 注册与名称

CameraRegistry 使用稳定不可变 sensor_id（例如 uint64）作为真正身份，name 是可在时间线上变化的别名。自动编号 next_auto_camera_index 初始为 0：

- 还没有任何显式注册及已使用默认相机时，第一次 CreateCamera(T_BC,t) 得到 cam0。
- 若无 CreateCamera，第一次枚举相机/生成图像/使用默认 cam0 时自动保证有一台 T_BC=(I3,0) 的 cam0；其默认创建语义可视为初始配置，不产生额外 phantom cam1。
- 如果已使用默认 cam0，再 CreateCamera 得到 cam1；之后依次递增，改名不回收旧自动编号。
- RenameCamera(t,id,new_name) 必须确认该时间下名称唯一；历史查询使用当时的名称；帧号及时间约束始终绑定不可变 sensor_id。

每次创建、重命名、修改标定均记入时间线；查询 t 仅使用 t 以前生效的相机定义。新相机在创建前的时间不能被用来生成图像或求交。

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

### 6.2 可复现噪声，不保存历史传感器数据

默认 noise_enabled=false，返回无噪值。开启后白噪声、零偏随机游走参数必须数值生效。为遵守禁止缓存 IMU 样本/历史偏置的要求，采用**以随机种子、时间参数和噪声事件版本确定的可按需重算随机过程**：

- 对每个 IMU 时间请求，白噪声由 seed、sensor_id、该时刻的纳秒时间/采样序号等确定性输入产生，方差按连续噪声密度和采样间隔离散化。
- 随机游走偏置定义为基于 seed、噪声配置生效时刻及固定时间网格的时间函数；使用确定性随机增量构造偏置，历史/未来时刻都能按需重算相同值，**不保留前一帧 IMU 样本或偏置轨迹缓存**。
- 初始采样间隔用标称 1/rate_hz；其后白噪声离散间隔可用 imu0 最近成功输出时间求得（该时间是防重协议变量，而非帧缓存）；固定时间网格随机游走模型须与该抽样规则在数学上注明。
- 固定种子、相同修改历史、时间请求顺序 → 可复现。不同参数生效版本不得悄悄沿用旧随机过程。

对按步长 Δt 积分的离散白噪和偏置增量，可采用常见密度关系 σ_white=N/√Δt、σ_walk=K√Δt；具体谱密度单位/约定在实际 API 文档和数值验证中固定。随机过程时间函数可由前缀增量求值或无缓存的分层确定性生成实现，**但不得以长期保存各次偏置/测量结果充当缓存**。当长序列无缓存重算的性能不满足目标时，只能优化算法，不得在没有用户授权情况下重新引入帧或位姿缓存。

## 7. Room 真实求交与可见性

场景只有 Room，其六个有限面满足：

- x=0、x=depth；
- y=0、y=width；
- z=0、z=height。

给定输入时间 t、当时存在且可按名称找到的相机 C 和相机系单位视线 u_C，**先调用历史解析求值得到 t 的 p_WB/R_WB 及该相机当时 T_BC/K/畸变**，计算：

$$
d_W=R_{WC}u_C,\qquad P_W(\lambda)=c_W+\lambda d_W,\quad\lambda>0.
$$

对每个候选面，在射线方向该轴非零时求 λ=(plane−c_axis)/d_axis；过滤非正 λ、平行、有限面二维边界之外的点，使用最小有效 λ 作为真实首次命中；棱边/角点相等交点采用固定面优先级，以保证可复现。

须验证视线有限、长度在单位容差内、位于相机正向有效成像视域、实际畸变后像素在图像范围。Room 渲染的遮挡规则必须和此求交的最近表面选择一致。返回列表与输入 vector 索引一一对应：每个结果有命中与否、世界坐标、射线距离、面标识及失败原因；不得把离散绘制网格顶点误当成唯一真实交点。

**Raycast 不查任何 PoseCache，也不要求 t 被 Generate 用过**；可以反复读过去 t，不能影响当前运动/配置历史和任何传感器序号。

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

### 9.1 配置文件格式提案

推荐 YAML（yaml-cpp 可选），其含义与逐条 C++ 调用一致。例如：

~~~yaml
simulation:
  room: {depth_m: 10.0, width_m: 10.0, height_m: 3.0}
  initial_pose:
    timestamp_ns: 0
    position_W_m: [5.0, 5.0, 1.5]
    rotation_WB_quaternion_wxyz: [1, 0, 0, 0]
  events:
    - {timestamp_ns: 0, op: SetLinearMotion, mode: ConstantVelocity,
       velocity_W_mps: [0.25, 0, 0]}
    - {timestamp_ns: 1000000000, op: BeginMotionMode}
    - {timestamp_ns: 1000000000, op: SetAngularMotion,
       mode: FixedAxisConstantRotation, axis_B: [0, 0, 1],
       angular_rate_radps: 0.35}
    - {timestamp_ns: 2000000000, op: EndMotionEnd}
  cameras:
    - {timestamp_ns: 0, op: CreateCamera, name: cam0, T_BC: identity}
  imu:
    noise_enabled: false
  sampling:
    camera_times_ns: [0, 50000000, 100000000]
    imu_times_ns: [0, 5000000, 10000000]
~~~

此示例只是结构草案；正式 schema 应在实现时固定字段和错误反馈。配置文件读取、事件解析及排序必须遵守 SRS 的时间/作用域/相机创建规则，并对事件合法性采取“全部验证后提交”策略，不能装载一半失败。

CLI 的运动示例（前向平移、纯旋转及组合）应该作为配置文件或对外函数调用示例，而不是写死在 main.cpp 里。**唯一场景永远是 Room**，不接受 ForwardDepthScene 名称。导出需用户显式指定路径及是否覆盖，禁止自动 remove_all 任意目录。

### 9.2 CMake / 安装行为

最终在目标仓库创建 SHARED 库并安装到 CMAKE_INSTALL_PREFIX 下，导出公开头文件、库、VisualSimConfig.cmake、VisualSimTargets.cmake，提供消费端 find_package(VisualSim CONFIG REQUIRED) 和 target_link_libraries(app PRIVATE VisualSim::VisualSim) 的例子。依赖只含实际使用的非 ROS 库；CMake 在 Windows 上明示配置失败，移除 Windows 运行/构建入口。

**不需要也不允许以“Ubuntu 24.04 默认工具链太老”为理由强制降低 CMake、GCC 或 C++ 标准要求**。维持用户现有可用的构建链，并在文档中列明实际选用版本。纳秒时间字段进行 std::is_same_v<TimestampNs,std::int64_t> 等编译期检查。

## 10. 建议验证用例（实施阶段再执行）

| ID | 验证 | 对应需求 |
| --- | --- | --- |
| V01 | Ubuntu 安装 .so、无 ROS 独立消费端；Windows 配置失败；无工具链降级 | AC-01 |
| V02 | C++ 调用与 YAML 事件序列等价；顺序相同且同刻覆盖稳定 | AC-02 |
| V03 | 同类模式覆盖、跨类组合、嵌套 Begin/End 恢复，退出没有 p/R 瞬移 | AC-03 |
| V04 | 静止、恒速、恒加速、圆周与多段组合的 p/R/v/a/ω/α 和解析导数一致 | AC-04 |
| V05 | 默认 cam0 外参为 (I3,0)；先显式创建得到 cam0，后续 cam1/cam2；重命名后不重复、不重置帧号 | AC-05 |
| V06 | 相机 K/T_BC/非零畸变影响图像；IMU T_BS 杆臂影响比力；启用/禁用噪声和随机种子有效 | AC-06 |
| V07 | 所有相机的门控、同刻 imu 的独立调用、每传感器重复/回退拒绝，整批失败无副作用 | AC-07 |
| V08 | 在**没有生成图像**的过去/未来时间，Raycast 仍能返回 Room 六面最近真实交点；无效视线状态正确 | AC-08 |
| V09 | 事件改动被记录/可重算历史；扫描内存状态和类型确保没有 64 帧位姿、图像、IMU 结果缓存 | AC-09 |
| V10 | 所有 ns 字段严格 std::int64_t；配置驱动数据集包含原有平移/旋转能力，场景只有 Room | AC-10 |
| V11 | 发生错误后可用新合法时间生成数据，编号、噪声过程和事件历史未被污染 | AC-07/09 |

**本轮仅写设计文档，不编译、不改生产代码、不运行测试。** 代码分批实施和来源仓库只读保护见 IMPLEMENTATION_PLAN.md。
