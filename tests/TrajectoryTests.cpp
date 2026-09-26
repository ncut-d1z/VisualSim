#include "Trajectory.hpp"
#include "Room.hpp"

#include <Eigen/Core>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void Require(bool ok, const char* message) {
  if (!ok)
    throw std::runtime_error(message);
}

}  // namespace

int main() {
  using namespace visualsim;
  try {
    {
      ForwardConstantVelocity path(
          10.0, 0.5, Eigen::Vector3d(1.0, 2.0, 3.0),
          Eigen::Matrix3d::Identity());

      const auto p0 = path.pose(0.0);
      const auto p2 = path.pose(2.0);
      const auto k = path.kinematics(7.0);

      Require((p0.R_wc - Eigen::Matrix3d::Identity()).norm() < 1e-14,
              "forward motion orientation changed at t=0");
      Require((p2.R_wc - p0.R_wc).norm() < 1e-14,
              "forward motion orientation is not constant");
      Require((p2.position_w - Eigen::Vector3d(1.0, 2.0, 4.0)).norm() < 1e-14,
              "forward motion position is incorrect");
      Require((k.velocity_w - Eigen::Vector3d(0.0, 0.0, 0.5)).norm() < 1e-14,
              "forward motion velocity is incorrect");
      Require(k.acceleration_w.norm() < 1e-14,
              "forward motion acceleration is not zero");
      Require(k.angular_velocity_c.norm() < 1e-14,
              "forward motion angular velocity is not zero");
    }

    {
      constexpr double omega = 0.4;
      OpticalAxisConstantRotation path(
          10.0, omega, Eigen::Vector3d(1.0, 2.0, 3.0),
          Eigen::Matrix3d::Identity());

      const auto p0 = path.pose(0.0);
      const auto p2 = path.pose(2.0);
      const auto k = path.kinematics(5.0);
      const Eigen::Matrix3d expected =
          Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitZ()).toRotationMatrix();

      Require((p0.position_w - p2.position_w).norm() < 1e-14,
              "optical-axis rotation changed camera position");
      Require((p2.R_wc - expected).norm() < 1e-14,
              "optical-axis rotation attitude is incorrect");
      Require(k.velocity_w.norm() < 1e-14,
              "optical-axis rotation linear velocity is not zero");
      Require(k.acceleration_w.norm() < 1e-14,
              "optical-axis rotation linear acceleration is not zero");
      Require((k.angular_velocity_c -
               Eigen::Vector3d(0.0, 0.0, omega)).norm() < 1e-14,
               "optical-axis angular velocity is incorrect");
    }

    {
      const Eigen::Vector3d requested_axis(1.0, -2.0, 4.0);
      OpticalAxisConstantRotation path(
          1.0, 0.5, requested_axis, Eigen::Vector3d::Zero(),
          Eigen::Matrix3d::Identity());
      const Eigen::Vector3d axis = requested_axis.normalized();
      Require((path.rotationAxis() - axis).norm() < 1e-14,
              "custom rotation axis was not normalized");
      Require((path.kinematics(0.0).angular_velocity_c - 0.5 * axis).norm()
                  < 1e-14,
              "custom angular velocity axis is incorrect");
    }

    {
      const Room<double> room;
      Require(room.object_points_.size() > 1000,
              "reference rectangular room is not richly sampled");
      Require((room.center_ - Eigen::Vector3d(5.0, 5.0, 1.5)).norm() < 1e-14,
              "reference rectangular room dimensions are incorrect");
    }

    std::cout << "PASS: both trajectories satisfy their defining kinematics\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << "\n";
    return 1;
  }
}

