#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <tuple>
#include <vector>

#include <Eigen/Dense>

namespace visualsim {

// Rectangular, richly textured room adapted from the reference VisualSim
// implementation in KaiserKatze/ros2-vio-node/src/VisualSim/Room.hpp.
// Coordinates are (depth, width, height), with the camera placed at the room
// center by the generator.
template <typename value_type = double> struct Room {
  using Point3 = Eigen::Vector<value_type, 3>;
  using Point3i = std::array<int, 3>;

  const value_type width_{10.0};
  const value_type depth_{10.0};
  const value_type height_{3.0};
  const Point3 center_{depth_ * static_cast<value_type>(0.5),
                       width_ * static_cast<value_type>(0.5),
                       height_ * static_cast<value_type>(0.5)};

  const int cnt_sep_depth_;
  const int cnt_sep_width_;
  const int cnt_sep_height_;
  const value_type step_d_;
  const value_type step_w_;
  const value_type step_h_;
  std::vector<Point3i> object_points_;
  Eigen::Matrix<value_type, 3, Eigen::Dynamic> object_matrix_;

  Room(int cnt_sep_depth = 20, int cnt_sep_width = 20,
       int cnt_sep_height = 6)
      : cnt_sep_depth_{cnt_sep_depth}, cnt_sep_width_{cnt_sep_width},
        cnt_sep_height_{cnt_sep_height},
        step_d_{depth_ / static_cast<value_type>(cnt_sep_depth_)},
        step_w_{width_ / static_cast<value_type>(cnt_sep_width_)},
        step_h_{height_ / static_cast<value_type>(cnt_sep_height_)} {
    if (cnt_sep_depth <= 0 || cnt_sep_width <= 0 || cnt_sep_height <= 0) {
      throw std::invalid_argument("room subdivision counts must be positive");
    }
    if (cnt_sep_depth % 2 != 0 || cnt_sep_width % 2 != 0 ||
        cnt_sep_height % 2 != 0) {
      throw std::invalid_argument("room subdivision counts must be even");
    }

    // Floor and ceiling.
    for (int ix = 0; ix <= cnt_sep_depth_; ++ix) {
      for (int iy = 0; iy <= cnt_sep_width_; ++iy) {
        object_points_.push_back({ix, iy, 0});
        object_points_.push_back({ix, iy, cnt_sep_height_});
      }
    }

    // Walls x = 0 and x = depth.
    for (int iy = 0; iy <= cnt_sep_width_; ++iy) {
      for (int iz = 0; iz <= cnt_sep_height_; ++iz) {
        object_points_.push_back({0, iy, iz});
        object_points_.push_back({cnt_sep_depth_, iy, iz});
      }
    }

    // Walls y = 0 and y = width.
    for (int ix = 0; ix <= cnt_sep_depth_; ++ix) {
      for (int iz = 0; iz <= cnt_sep_height_; ++iz) {
        object_points_.push_back({ix, 0, iz});
        object_points_.push_back({ix, cnt_sep_width_, iz});
      }
    }

    std::sort(object_points_.begin(), object_points_.end());
    object_points_.erase(
        std::unique(object_points_.begin(), object_points_.end()),
        object_points_.end());

    object_matrix_.resize(3, static_cast<Eigen::Index>(object_points_.size()));
    for (std::size_t i = 0; i < object_points_.size(); ++i) {
      const auto& point = object_points_[i];
      object_matrix_.col(static_cast<Eigen::Index>(i)) =
          Point3{point[0] * step_d_, point[1] * step_w_, point[2] * step_h_};
    }
  }

  std::vector<Point3> Points() const {
    std::vector<Point3> points;
    points.reserve(object_points_.size());
    for (const auto& point : object_points_) {
      points.emplace_back(point[0] * step_d_, point[1] * step_w_,
                          point[2] * step_h_);
    }
    return points;
  }
};

}  // namespace visualsim

