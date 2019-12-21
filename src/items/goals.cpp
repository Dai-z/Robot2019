
#include "goals.hpp"

const float R = 150;
const auto rect = QRectF(-R / 2, -R / 2, R, R);

Goals::Goals(QGraphicsItem* parent) : Item(parent) {
  model_ = Model::getInstance();
}

void Goals::myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option,
                    QWidget* widget) {
  auto goals = model_->getSimGoalPosts();
  auto robotPos = model_->getLocRobotPos();
  for_each(goals.begin(), goals.end(), [&](geometry_msgs::Vector3& p) {
    auto g = getGlobalPosition(robotPos, p);
    this->drawGoal(painter, g.x(), g.y());
  });
}

void Goals::drawGoal(QPainter* painter, qreal x, qreal y) {
  painter->setPen(QPen(Qt::red, 5));
  painter->setBrush(Qt::red);
  QRectF rect(x - 5, y - 5, 10, 10);
  painter->drawEllipse(rect);
  drawText(x + 5, y, QString("G"));
}
