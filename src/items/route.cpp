#include "route.hpp"

const float R = 150;
const auto rect = QRectF(-R / 2, -R / 2, R, R);

Route::Route(QGraphicsItem* parent) : Item(parent) {
  model_ = Model::getInstance();
}

void Route::myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                    QWidget* widget) {
  auto route = model_->getRoute();
  for (auto r : route) {
      drawRoute(painter, r.x, r.y);
  }
}

void Route::drawRoute(QPainter* painter, qreal x, qreal y) {
  painter->setPen(QPen(Qt::red, 5));
  painter->setBrush(Qt::red);
  painter->drawPoint(x, y);
}
