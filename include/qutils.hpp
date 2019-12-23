#pragma once

#include "utils.hpp"
#include <QMatrix>
#include <QVector3D>

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

inline QVector3D getGlobalPosition(QVector3D robotPos,
                                   geometry_msgs::Vector3 pos) {
  auto v = getGlobalPosition(robotPos, QVector3D(pos.x, pos.y, pos.z));
  return QVector3D(v.x(), v.y(), v.z());
}

inline QVector3D getFieldPosition(QVector3D robotPos, QVector3D pos) {
  auto rx = robotPos.x();
  auto ry = robotPos.y();
  auto rz = robotPos.z();

  auto x = pos.x();
  auto y = pos.y();
  auto z = pos.z();

  QVector3D res = pos;
  res.setX(x - rx);
  res.setY(y - ry);
  res.setZ(z - rz);

  rotateVector(res, -rz);
  return res;
}
inline QPointF getFieldPosition(QVector3D robotPos, QPointF pos) {
  auto v = getFieldPosition(robotPos, QVector3D(pos.x(), pos.y(), 0));
  return QPointF(v.x(), v.y());
}

inline QPointF getFieldPosition(QVector3D robotPos, qreal x, qreal y) {
  return getFieldPosition(robotPos, QPointF(x, y));
}

inline QVector3D Vector3ToQVector3D(geometry_msgs::Vector3 in) {
  QVector3D res;
  res.setX(in.x);
  res.setY(in.y);
  res.setZ(in.z);
  return res;
}

inline QPointF Vector3ToQPointF(geometry_msgs::Vector3 in) {
  QPointF res;
  res.setX(in.x);
  res.setY(in.y);
  return res;
}

inline geometry_msgs::Vector3 QVector3DToVector3(QVector3D in) {
  geometry_msgs::Vector3 res;
  res.x = in.x();
  res.y = in.y();
  res.z = in.z();
  return res;
}

inline geometry_msgs::Vector3 QPointFToVector3(QPointF in) {
  geometry_msgs::Vector3 res;
  res.x = in.x();
  res.y = in.y();
  return res;
}
