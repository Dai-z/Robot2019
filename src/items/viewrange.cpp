#include "viewrange.hpp"

ViewRange::ViewRange(QGraphicsItem* parent) : Item(parent) {
  model_ = Model::getInstance();
  view_range_field_.resize(4);
  view_range_field_[0].setX(350);
  view_range_field_[0].setY(300);
  view_range_field_[1].setX(350);
  view_range_field_[1].setY(-300);
  view_range_field_[2].setX(30);
  view_range_field_[2].setY(-60);
  view_range_field_[3].setX(30);
  view_range_field_[3].setY(60);
}

QRectF ViewRange::boundingRect() const { return QRectF(0, 0, 0, 0); }

void ViewRange::myPaint(QPainter* painter,
                        const QStyleOptionGraphicsItem* option,
                        QWidget* widget) {
  QVector3D robotPos;
  robotPos = model_->getSimRobotPos();
  checkSimBallInView();
  checkCirclePointInview();
  checkGoalInview();
  checkWhitePointsInView();
  // checkSimObstacleInView();

  auto aa = getGlobalPosition(robotPos, view_range_field_[0]);
  auto bb = getGlobalPosition(robotPos, view_range_field_[1]);
  auto cc = getGlobalPosition(robotPos, view_range_field_[2]);
  auto dd = getGlobalPosition(robotPos, view_range_field_[3]);

  std::vector<QPointF> points{aa, bb, cc, dd};

  drawText(aa, QString("a"));
  drawText(bb, QString("b"));
  drawText(cc, QString("c"));
  drawText(dd, QString("d"));

  QPen pen(QColor(Qt::yellow), 0);
  painter->setPen(pen);
  painter->setBrush(QBrush());
  painter->drawPolygon(points.data(), points.size());
}

// Copied from https://stackoverflow.com/a/2922778
int pnpoly(int nvert, float* vertx, float* verty, float testx, float testy) {
  int i, j, c = 0;
  for (i = 0, j = nvert - 1; i < nvert; j = i++) {
    if (((verty[i] > testy) != (verty[j] > testy)) &&
        (testx <
         (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) +
             vertx[i]))
      c = !c;
  }
  return c;
}

bool ViewRange::inView(float x, float y) {
  float vx[4], vy[4];
  auto robotPos = model_->getSimRobotPos();

  auto dx = robotPos.x() - x;
  auto dy = robotPos.y() - y;

  auto dis = sqrt(dx * dx + dy * dy);
  if (dis > MAX_SEEN_DIST) return false;

  auto aa = getGlobalPosition(robotPos, view_range_field_[0]);
  auto bb = getGlobalPosition(robotPos, view_range_field_[1]);
  auto cc = getGlobalPosition(robotPos, view_range_field_[2]);
  auto dd = getGlobalPosition(robotPos, view_range_field_[3]);

  vx[0] = aa.x();
  vy[0] = aa.y();
  vx[1] = bb.x();
  vy[1] = bb.y();
  vx[2] = cc.x();
  vy[2] = cc.y();
  vx[3] = dd.x();
  vy[3] = dd.y();

  return pnpoly(4, vx, vy, x, y);
}

void ViewRange::checkSimBallInView() {
  auto pos = Model::getSimBallPos();
  bool seen = inView(pos.x(), pos.y());
}

void ViewRange::checkWhitePointsInView() {
  std::vector<std::pair<int, int>> Lcorner = {
      {350, 250}, {350, -250}, {-350, 250}, {-350, -250},
      {450, 300}, {450, -300}, {-450, 300}, {-450, -300}};
  std::vector<std::pair<int, int>> Tcorner = {
      {0, 300}, {0, -300}, {450, 250}, {450, -250}, {-450, 250}, {-450, -250}};

  auto& lres = model_->getSimLCorners();
  lres.clear();
  auto robotPos = model_->getSimRobotPos();
  geometry_msgs::Vector3 tmp;

  // Update L corner
  for (auto p : Lcorner) {
    auto x = p.first;
    auto y = p.second;
    if (inView(x, y)) {
      auto g = getFieldPosition(robotPos, x, y);
      tmp.x = g.x();
      tmp.y = g.y();
      lres.push_back(tmp);
    }
  };

  // Update T corner
  auto& tres = model_->getSimTCorners();
  tres.clear();
  for (auto p : Tcorner) {
    auto x = p.first;
    auto y = p.second;
    if (inView(x, y)) {
      auto g = getFieldPosition(robotPos, x, y);
      tmp.x = g.x();
      tmp.y = g.y();
      tres.push_back(tmp);
    }
  };
}

void ViewRange::checkCirclePointInview() {
  bool circle_in_view = inView(0, 0);
  model_->setSeeSimCircle(circle_in_view);
  if (circle_in_view) {
    auto& circle_field = model_->getSimCircle();
    auto robotPos = model_->getSimRobotPos();
    auto g = getFieldPosition(robotPos, 0, 0);
    circle_field.setX(g.x());
    circle_field.setY(g.y());
  }
}

void ViewRange::checkGoalInview() {
  auto& res = model_->getSimGoalPosts();
  res.clear();

  // (450, 130) (450, -130)
  // x, y
  // x, -y

  // -x, -y
  // -x, y
  auto goalX = 450;
  auto goalY = 130;

  auto robotPos = model_->getSimRobotPos();
  std::vector<std::pair<int, int>> foo = {{1, 1}, {1, -1}, {-1, -1}, {-1, 1}};

  geometry_msgs::Vector3 tmp;
  for_each(foo.begin(), foo.end(), [&](std::pair<int, int> p) {
    auto x = p.first * goalX;
    auto y = p.second * goalY;
    if (inView(x, y)) {
      auto g = getFieldPosition(robotPos, x, y);
      tmp.x = g.x();
      tmp.y = g.y();
      res.push_back(tmp);
    }
  });
  assert(res.size() < 3);
}

// void ViewRange::checkSimObstacleInView() {
//   auto& res = model_->getSimObstacles();
//   res.clear();
//   // process sim obstacle (only one)
//   auto simObstaclePos = Model::getSimObstaclePos();
//   bool seen = inView(simObstaclePos.x(), simObstaclePos.y());
//   model_->setSeeSimObstacle(seen);
//   if (seen) {
//     auto robotPos = model_->getSimRobotPos();
//     res.push_back(QPointFToVector3(getFieldPosition(robotPos,
//     simObstaclePos)));
//   }
//   // TODO(corenel) treat other robots as obsatcles
// }
