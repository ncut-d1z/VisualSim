#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace visualsim {

struct Pose {
  Eigen::Vector3d position_w = Eigen::Vector3d::Zero();
  // Rotation from camera/body coordinates to world coordinates.
  Eigen::Matrix3d R_wc = Eigen::Matrix3d::Identity();
};

struct Kinematics {
  Eigen::Vector3d velocity_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration_w = Eigen::Vector3d::Zero();
  // Angular velocity expressed in the camera/body frame.
  Eigen::Vector3d angular_velocity_c = Eigen::Vector3d::Zero();
};

class Trajectory {
 public:
  explicit Trajectory(double duration_s) : duration_s_(duration_s) {
    if (!(duration_s > 0.0)) {
      throw std::invalid_argument("duration must be positive");
    }
  }
  virtual ~Trajectory() = default;

  virtual Pose pose(double time_s) const = 0;
  virtual Kinematics kinematics(double time_s) const = 0;
  virtual std::string name() const = 0;

  double duration() const noexcept { return duration_s_; }

 protected:
  double clampTime(double time_s) const noexcept {
    return std::clamp(time_s, 0.0, duration_s_);
  }

 private:
  double duration_s_;
};

// Camera orientation is constant. Translation is at constant speed along the
// camera's initial forward (optical) axis.
class ForwardConstantVelocity final : public Trajectory {
 public:
  ForwardConstantVelocity(double duration_s, double speed_mps,
                          const Eigen::Vector3d& position0_w =
                              Eigen::Vector3d::Zero(),
                          const Eigen::Matrix3d& R0_wc =
                              Eigen::Matrix3d::Identity())
      : Trajectory(duration_s),
        speed_mps_(speed_mps),
        position0_w_(position0_w),
        R0_wc_(R0_wc),
        forward_w_(R0_wc.col(2).normalized()) {
    if (!(speed_mps > 0.0)) {
      throw std::invalid_argument("forward speed must be positive");
    }
  }

  Pose pose(double time_s) const override {
    const double t = clampTime(time_s);
    return {position0_w_ + speed_mps_ * t * forward_w_, R0_wc_};
  }

  Kinematics kinematics(double /*time_s*/) const override {
    Kinematics k;
    k.velocity_w = speed_mps_ * forward_w_;
    return k;
  }

  std::string name() const override { return "forward_constant_velocity"; }

  double speed() const noexcept { return speed_mps_; }

 private:
  double speed_mps_;
  Eigen::Vector3d position0_w_;
  Eigen::Matrix3d R0_wc_;
  Eigen::Vector3d forward_w_;
};

// Camera center is fixed. The camera rolls at a constant angular velocity
// around its optical axis. With R0_wc = I, the optical axis is +Z.
class OpticalAxisConstantRotation final : public Trajectory {
 public:
  OpticalAxisConstantRotation(double duration_s, double omega_rad_s,
                              const Eigen::Vector3d& position_w =
                                  Eigen::Vector3d::Zero(),
                              const Eigen::Matrix3d& R0_wc =
                                  Eigen::Matrix3d::Identity())
      : Trajectory(duration_s),
        omega_rad_s_(omega_rad_s),
        position_w_(position_w),
        R0_wc_(R0_wc) {
    if (std::abs(omega_rad_s) < 1e-12) {
      throw std::invalid_argument("angular velocity must be non-zero");
    }
  }

  Pose pose(double time_s) const override {
    const double t = clampTime(time_s);
    // Right multiplication rotates the camera about its own +Z optical axis.
    const Eigen::Matrix3d R_roll =
        Eigen::AngleAxisd(omega_rad_s_ * t, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    return {position_w_, R0_wc_ * R_roll};
  }

  Kinematics kinematics(double /*time_s*/) const override {
    Kinematics k;
    k.angular_velocity_c = omega_rad_s_ * Eigen::Vector3d::UnitZ();
    return k;
  }

  std::string name() const override { return "optical_axis_constant_rotation"; }

  double omega() const noexcept { return omega_rad_s_; }

 private:
  double omega_rad_s_;
  Eigen::Vector3d position_w_;
  Eigen::Matrix3d R0_wc_;
};

}  // namespace visualsim
