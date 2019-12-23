#pragma once

#include <geometry_msgs/Vector3.h>
#include <algorithm>
#include "types.hpp"

template <typename T>
inline T DegreeToRadian(T angle) {
  return angle / 180.f * M_PI;
}

template <typename T>
inline T RadianToDegree(T angle) {
  return angle / M_PI * 180.f;
}

template <typename T>
inline void CorrectAngleDegree180(T& deg) {
  T x_mod = std::fmod(deg + 180, 360);
  if (x_mod < 0) {
    x_mod += 360;
  }
  deg = x_mod - 180;
}

template <typename T>
inline T GetSlope(T x1, T y1, T x2, T y2) {
  auto res = std::atan2(y2 - y1, x2 - x1);
  return RadianToDegree(res);
}

template <typename T>
inline void rotateVector(T& vec, float degree) {
  auto x = vec.x();
  auto y = vec.y();
  auto r = DegreeToRadian(degree);

  auto sin_ = std::sin(r);
  auto cos_ = std::cos(r);

  vec.setX(x * cos_ - y * sin_);
  vec.setY(x * sin_ + y * cos_);
}

template <typename T>
double length(T v) {
  return sqrt(v.x() * v.x() + v.y() * v.y());
}

template <typename T>
double GetDistance(const T& p) {
  return std::sqrt((p.x * p.x) + (p.y * p.y));
}

template <typename T>
double GetDistance(const T& p,
                   const T& p2) {
  double x = std::abs(p.x - p2.x);
  double y = std::abs(p.y - p2.y);
  return std::sqrt(x * x + y * y);
}

template <typename T>
T RotateCoordinateAxis(const double& alpha,
                                            const T& p) {
  double alpha_rad = DegreeToRadian(alpha);
  T rotated;
  rotated.x = p.x * std::cos(alpha_rad) - p.y * std::sin(alpha_rad);
  rotated.y = p.x * std::sin(alpha_rad) + p.y * std::cos(alpha_rad);
  return rotated;
}
