
#include "circle.hpp"

const float R = 150;
const auto rect = QRectF(-R / 2, -R / 2, R, R);

Circle::Circle(QGraphicsItem* parent) : Item(parent) {
  model_ = Model::getInstance();
}

void Circle::myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                     QWidget* widget) {
  // draw if circle is seen in localization information
  if (model_->getSimSeeCircle()) {
    auto circlePos = model_->getSimCircle();
    auto g = getGlobalPosition(model_->getLocRobotPos(), circlePos);
    setPos(g);
    auto c = QColor(255, 255, 0, 80);
    painter->setPen(QPen(c, 5));
    painter->drawEllipse(rect);

    painter->setPen(Qt::black);
    drawText(0, 0, QString("Circle"));
  }
}
