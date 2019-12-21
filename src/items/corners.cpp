#include "corners.hpp"

const float R = 150;
const auto rect = QRectF(-R / 2, -R / 2, R, R);

Corners::Corners(QGraphicsItem* parent) : Item(parent) {
  model_ = Model::getInstance();
}

void Corners::myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                      QWidget* widget) {
  auto robotPos = model_->getLocRobotPos();

  auto Lcorners = model_->getSimLCorners();
  auto Tcorners = model_->getSimTCorners();

  for (auto p : Lcorners) {
    auto g = getGlobalPosition(robotPos, p);
    this->drawCorner(painter, g.x(), g.y(), QString("L"));
  }

  for (auto p : Tcorners) {
    auto g = getGlobalPosition(robotPos, p);
    this->drawCorner(painter, g.x(), g.y(), QString("T"));
  }
}

void Corners::drawCorner(QPainter* painter, qreal x, qreal y, QString type) {
  painter->setPen(QPen(Qt::magenta, 5));
  painter->setBrush(Qt::magenta);
  QRectF rect(x - 5, y - 5, 10, 10);
  painter->drawEllipse(rect);
  drawText(x + 5, y, type);
}
