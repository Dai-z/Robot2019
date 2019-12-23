#pragma once

#include <math.h>
#include <random>
#include <cassert>
#include <vector>

struct Pose {
 public:
  double x;
  double y;
  double heading;

  Pose() : x(0), y(0), heading(0) {}
  Pose(double x, double y, double heading) : x(x), y(y), heading(heading) {}
  Pose(const Pose& other) : x(other.x), y(other.y), heading(other.heading) {}

  bool operator==(const Pose& oth) const {
    if (x != oth.x) return false;

    if (y != oth.y) return false;

    return heading == oth.heading;
  }
  double& operator[](int index) {
    assert(index < 2);
    if (index == 0) return x;
    if (index == 1) return y;
    if (index == 2) return heading;
    return x;
  }
  double headingR() const { return heading / 180.0 * M_PI; }
  double length() const { return sqrt(x * x + y * y); };
  /**
   * @brief set heading angle in radian
   *
   * @param h - given heading angle in radian
   */
  void setHeadingR(double h) { heading = h / M_PI * 180.0; };
  /**
   * @brief rotate coordinate system  with given degree
   *
   * @param degree - given degree
   */
  void rotate(double degree) {
    double s = sin(degree * 180.f / M_PI);
    double c = cos(degree * 180.f / M_PI);

    double tmpx = c * x - s * y;
    double tmpy = s * x + c * y;

    x = tmpx;
    y = tmpy;
  }
};

struct Matrix {
  Matrix() {
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) m[i][j] = 0.0;
  }
  double m[3][3];
};

struct Particle {
  Particle() {}
  Pose pose;
  double weight;
};

//! Information for a cluster of sample particles
struct Cluster {
  //! Number of samples
  int count;

  //! Total weight of samples in this cluster
  double weight;
  //! Average weight of samples in this cluster
  double meanWeight;

  //! Mean value for cluster statistics
  Pose mean;
  //! Covariance value for cluster statistics
  Matrix cov;

  //! Workspace for mean value
  double m[4];
  //! Workspace for covariance value
  double c[2][2];
};

template <typename T>
class Distribution {
 public:
  Distribution(float a, float b) {
    dist_ = T(a, b);

    std::random_device rd;
    generator_ = std::default_random_engine(rd());
  }
  float sample() { return dist_(generator_); }

 private:
  std::default_random_engine generator_;
  T dist_;
};

using Gaussian = Distribution<std::normal_distribution<float>>;
using Uniform = Distribution<std::uniform_real_distribution<float>>;
