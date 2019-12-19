#include <algorithm>

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

inline QVector3D getGlobalPosition(QVector3D robotPos, QVector3D pos) {
  auto rx = robotPos.x();
  auto ry = robotPos.y();
  auto rz = robotPos.z();

  rotateVector(pos, rz);

  auto x = pos.x();
  auto y = pos.y();
  auto z = pos.z();

  QVector3D res;
  res.setX(x + rx);
  res.setY(y + ry);
  res.setZ(z + rz);

  return res;
}

inline QPointF getGlobalPosition(QVector3D robotPos, QPointF pos) {
  auto v = getGlobalPosition(robotPos, QVector3D(pos.x(), pos.y(), 0));
  return QPointF(v.x(), v.y());
}