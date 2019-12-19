#include "item.hpp"

#include <QGraphicsSceneMouseEvent>
#include <QKeyEvent>

Item::Item(QGraphicsItem* parent) : QGraphicsItem(parent) {
  this->setFlag(QGraphicsItem::ItemIsFocusable, true);
  setVisible(false);
}

Item::~Item() {}

QRectF Item::boundingRect() const { return QRectF(0, 0, 0, 0); }

qreal Item::y() const { return -QGraphicsItem::y(); }

void Item::setY(qreal y) { QGraphicsItem::setY(-y); }

void Item::setPos(qreal x, qreal y) {
  setX(x);
  setY(y);
}

void Item::setPos(QPointF p) { setPos(p.x(), p.y()); }

void Item::setPos(QVector3D pos) {
  setX(pos.x());
  setY(pos.y());
  setRotation(pos.z());
}

QVector3D Item::pos() const { return QVector3D(x(), y(), angle_); }

qreal Item::angle() const { return angle_; }

void Item::paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                 QWidget* widget) {
  setPainter(painter);
  flipPainterYAxis();
  painter->rotate(angle_);

  if (rotate_enabled_) {
    auto c = QColor(240, 230, 140, 150);
    painter->setPen(QPen(c, 0));
    auto rect = this->boundingRect();
    painter->setBrush(c);
    painter->drawEllipse(rect);
  }

  this->myPaint(painter, option, widget);

  int w_2 = 1040 / 2 - 20;
  int h_2 = 740 / 2 - 20;

  int x_ = x();
  int y_ = y();

  if (x_ < -w_2) {
    setX(-w_2);
  } else if (x_ > w_2) {
    setX(w_2);
  }

  if (y_ < -h_2) {
    setY(-h_2);
  } else if (y_ > h_2) {
    setY(h_2);
  }
}

void Item::flipPainterYAxis() {
  if (!painter_) return;

  qreal scale_ = scale();
  // QMatrix m;
  // m.scale(1, -1);
  // painter->setMatrix(m);

  auto m = painter_->matrix();
  m.scale(scale_, -scale_);
  painter_->setMatrix(m);
}

void Item::myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                   QWidget* widget) {}

void Item::drawText(QPointF p, QString str) { drawText(p.x(), p.y(), str); }

void Item::drawText(qreal x, qreal y, QString str) {
  painter_->rotate(-angle_);
  flipPainterYAxis();

  auto sin_ = std::sin(DegreeToRadian(angle_));
  auto cos_ = std::cos(DegreeToRadian(angle_));
  auto x_ = x * cos_ - y * sin_;
  auto y_ = x * sin_ + y * cos_;

  painter_->drawText(x_, -y_, str);
  flipPainterYAxis();
  painter_->rotate(angle_);
}

void Item::setRotation(qreal r) {
  angle_ = r;
  CorrectAngleDegree180(angle_);
}

void Item::rotateDegree(qreal r) {
  angle_ += r;
  CorrectAngleDegree180(angle_);
}

void Item::rotateRadian(qreal r) {
  angle_ += RadianToDegree(r);
  CorrectAngleDegree180(angle_);
}

void Item::mousePressEvent(QGraphicsSceneMouseEvent* event) {
  if (rotate_enabled_) {
    pressed_ = true;
    angle_start_press_ = angle_;
    angle_start_press_2_ =
        GetSlope(x(), y(), event->pos().x(), -event->pos().y());
  }
  QGraphicsItem::mousePressEvent(event);
}

void Item::mouseReleaseEvent(QGraphicsSceneMouseEvent* event) {
  pressed_ = false;
  QGraphicsItem::mouseReleaseEvent(event);
}

void Item::mouseMoveEvent(QGraphicsSceneMouseEvent* event) {
  if (rotate_enabled_) {
    auto t2 = GetSlope(x(), y(), event->pos().x(), -event->pos().y());
    angle_ = angle_start_press_ + t2 - angle_start_press_2_;
  } else {
    QGraphicsItem::mouseMoveEvent(event);
  }
}

void Item::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Shift) {
    rotate_enabled_ = true;
  }
}

void Item::keyReleaseEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Shift) {
    rotate_enabled_ = false;
  }
}

void Item::setPainter(QPainter* p) { painter_ = p; }