#include "ball.hpp"
#include "utils.hpp"
// #include "circle.hpp"
#include "control_widget.hpp"
// #include "dest.hpp"
#include "field.hpp"
// #include "goals.hpp"
#include "item_manager.hpp"
// #include "obstacles.hpp"
#include "particles.hpp"
#include "robot.hpp"
// #include "viewrange.hpp"
// #include "whitelines.hpp"
// #include "whitepoints.hpp"
// #include "world.hpp"
#include <algorithm>

ItemManager::ItemManager(QObject* parent, QGraphicsScene* scene,
                         ControlWidget* control)
    : QObject(parent), scene_(scene), control_(control) {}

ItemManager::~ItemManager() {}

void ItemManager::Init() {
  // Add field, ball and obstacle
  scene_->addItem(new FieldItem());

  // Simulation
  //   auto simObstacle = new Obstacles(false, 0);
  //   simObstacle->setVisible(false);
  //   scene_->addItem(simObstacle);

  auto simRobot = new Robot(false);
  auto simBall = new Ball(false);

  auto locRobot = new Robot(true);
  auto p = new Particles();
  //   auto locCircle = new Circle(i);
  //   auto locLines = new WhiteLines(i);
  //   auto locWhitePoints = new WhitePoints(i);
  // auto viewRange = new ViewRange(i);
  //   auto goals = new Goals(i);
  //   auto obstacles = new Obstacles(true, i);
  //   auto dest = new Dest(i);

  scene_->addItem(locRobot);
  scene_->addItem(simRobot);
  scene_->addItem(simBall);
  scene_->addItem(p);
  //   scene_->addItem(locCircle);
  //   scene_->addItem(viewRange);
  //   scene_->addItem(locWhitePoints);
  //   scene_->addItem(goals);
  //   scene_->addItem(obstacles);
  //   scene_->addItem(dest);

  simRobot->setVisible(true);
  simBall->setVisible(true);

  locRobot->setVisible(true);

  connect(control_->show_particle_, &QCheckBox::toggled,
          [=](bool checked) { p->setVisible(checked); });

  // 30 fps rendering
  auto t = new QTimer(this);
  connect(t, &QTimer::timeout, [=]() {
    // handle collide
    if (simBall->isVisible()) {
      auto ballx = simBall->x();
      auto bally = simBall->y();
      if (simRobot->isVisible()) {
        auto model = Model::getInstance();
        auto simRobotPos = model->getSimRobotPos();
        auto ballField = getFieldPosition(simRobotPos, ballx, bally);

        if (fabs(ballField.x()) > 10 || fabs(ballField.y()) > 10) {
          return;
        }

        ballField.setX(ballField.x() + ballField.x() > 0 ? 10 : -10);
        ballField.setY(ballField.y() + ballField.y() > 0 ? 15 : -15);

        auto nb = getGlobalPosition(simRobotPos,
                                    QPointF(ballField.x(), ballField.y()));

        simBall->setX(nb.x());
        simBall->setY(nb.y());
      }
    }
  });
  t->start(1000 / 30.f);
}