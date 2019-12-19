#include "ball.hpp"
#include <QTimer>
#include "utils.hpp"

const float ballR = 15;
const QRectF ballRect = QRectF(-ballR / 2, -ballR / 2, ballR, ballR);

Ball::Ball(bool loc, QGraphicsItem* parent) : loc_(loc), Item(parent) {
  this->setFlag(QGraphicsItem::ItemIsMovable, true);
}

QRectF Ball::boundingRect() const { return ballRect; }

void Ball::myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                   QWidget* widget) {
  // In simulation mode, just set sim ball position
  if (!loc_ && !selected_) {
    setPos(Model::getSimBallPos());
  }

  // Paint ball and label
  auto c = QColor(255, 167, 0);

  painter->setPen(c);
  painter->setBrush(c);
  painter->drawEllipse(ballRect);

  painter->setPen(Qt::black);
}

void Ball::mousePressEvent(QGraphicsSceneMouseEvent* event) {
  selected_ = true;
  Item::mousePressEvent(event);
}

void Ball::mouseReleaseEvent(QGraphicsSceneMouseEvent* event) {
  selected_ = false;
  model_->setSimBallPos(pos().x(), pos().y());
  Item::mouseReleaseEvent(event);
}

void Ball::mouseMoveEvent(QGraphicsSceneMouseEvent* event) {
  model_->setSimBallPos(pos().x(), pos().y());
  Item::mouseMoveEvent(event);
}
