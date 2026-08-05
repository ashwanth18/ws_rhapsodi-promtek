#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace scooping_controller
{

struct TouchOffFit
{
  std::array<double, 3> centroid{};
  double yaw_deg{0.0};
  std::array<double, 3> plane_normal{};
  double residual_m{0.0};
};

/// Fit a container pose from three TCP touch points.
/// Matches scripts/calibrate_container_pose.py (centroid, yaw from p1->p2, residual).
inline TouchOffFit fit_touch_points(const std::vector<std::array<double, 3>>& points)
{
  if (points.size() != 3U) {
    throw std::runtime_error("touch-off requires exactly 3 points");
  }
  TouchOffFit fit;
  for (std::size_t i = 0; i < 3; ++i) {
    fit.centroid[i] = (points[0][i] + points[1][i] + points[2][i]) / 3.0;
  }
  const std::array<double, 3> ux = {
    points[1][0] - points[0][0],
    points[1][1] - points[0][1],
    points[1][2] - points[0][2]};
  const std::array<double, 3> uy = {
    points[2][0] - points[0][0],
    points[2][1] - points[0][1],
    points[2][2] - points[0][2]};
  fit.plane_normal = {
    ux[1] * uy[2] - ux[2] * uy[1],
    ux[2] * uy[0] - ux[0] * uy[2],
    ux[0] * uy[1] - ux[1] * uy[0]};
  const double magnitude = std::sqrt(
    fit.plane_normal[0] * fit.plane_normal[0] +
    fit.plane_normal[1] * fit.plane_normal[1] +
    fit.plane_normal[2] * fit.plane_normal[2]);
  if (magnitude < 1e-9) {
    throw std::runtime_error("touch points are collinear");
  }
  for (auto& value : fit.plane_normal) {
    value /= magnitude;
  }
  fit.yaw_deg = std::atan2(ux[1], ux[0]) * 180.0 / 3.14159265358979323846;
  fit.residual_m = 0.0;
  for (const auto& point : points) {
    double residual = 0.0;
    for (std::size_t i = 0; i < 3; ++i) {
      residual += (point[i] - fit.centroid[i]) * fit.plane_normal[i];
    }
    fit.residual_m = std::max(fit.residual_m, std::abs(residual));
  }
  return fit;
}

}  // namespace scooping_controller
