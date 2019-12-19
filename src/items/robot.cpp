#include "robot.hpp"

Robot::Robot(bool loc, QGraphicsItem* parent)
    : Item(parent), loc_(loc) {
  model_ = Model::getInstance();
  if (!loc) {
    this->setFlag(QGraphicsItem::ItemIsMovable, true);
  }
}

QRectF Robot::boundingRect() const {
  int r = 15;
  return QRectF(-r, -r, r * 2, r * 2);
}

static const std::vector<QPointF> points{QPointF(15, 0), QPointF(-10, -10),
                                         QPointF(-10, 10)};

void Robot::myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                    QWidget* widget) {
  if (loc_)
    this->locPaint(painter);
  else
    this->realPaint(painter);
  //    qDebug() << "robot paint";
}

void Robot::locPaint(QPainter* painter) {
  QVector3D locPos = model_->getLocRobotPos();
  setPos(locPos.x(), locPos.y());
  setRotation(locPos.z());

  auto c = QColor(255, 167, 0, 100);

  QPen pen(c, 0);

  painter->setPen(pen);
  painter->setBrush(c);
  painter->drawPolygon(points.data(), points.size());

  painter->setPen(Qt::black);
  drawText(
      0, 0,
      QString("(%1, %2, %3)").arg(x()).arg(y()).arg(this->angle_));
}

void Robot::realPaint(QPainter* painter) {
  if (!selected_) {
    auto pos = model_->getSimRobotPos();
    setPos(pos);
  }

  QColor c(255, 167, 0);
  QPen pen(c, 0);

  painter->setPen(pen);
  painter->setBrush(c);
  painter->drawPolygon(points.data(), points.size());

  painter->setPen(Qt::black);
}

void Robot::mousePressEvent(QGraphicsSceneMouseEvent* event) {
  selected_ = true;
  Item::mousePressEvent(event);
}

void Robot::mouseReleaseEvent(QGraphicsSceneMouseEvent* event) {
  selected_ = false;
  model_->setSimRobotPos(pos());
  Item::mouseReleaseEvent(event);
}

void Robot::mouseMoveEvent(QGraphicsSceneMouseEvent* event) {
  model_->setSimRobotPos(pos());
  Item::mouseMoveEvent(event);
}
