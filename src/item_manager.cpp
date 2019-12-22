#include "item_manager.hpp"
#include <algorithm>
#include "ball.hpp"
#include "circle.hpp"
#include "control_widget.hpp"
#include "corners.hpp"
#include "field.hpp"
#include "goals.hpp"
#include "particles.hpp"
#include "robot.hpp"
#include "route.hpp"
#include "utils.hpp"
#include "viewrange.hpp"

ItemManager::ItemManager(QObject* parent, QGraphicsScene* scene,
                         ControlWidget* control)
    : QObject(parent), scene_(scene), control_(control) {}

ItemManager::~ItemManager() {}

void ItemManager::Init() {
  // Add field, ball and obstacle
  scene_->addItem(new FieldItem());

  // Simulation
  auto simRobot = new Robot(false);
  auto simBall = new Ball(false);
  scene_->addItem(simRobot);
  scene_->addItem(simBall);

  auto locRobot = new Robot(true);
  auto viewRange = new ViewRange();
  auto p = new Particles();
  auto circle = new Circle();
  auto goals = new Goals();
  auto corners = new Corners();
  auto route = new Route();

  scene_->addItem(locRobot);
  scene_->addItem(p);
  scene_->addItem(viewRange);
  scene_->addItem(circle);
  scene_->addItem(goals);
  scene_->addItem(corners);
  scene_->addItem(route);

  simRobot->setVisible(true);
  simBall->setVisible(true);

  locRobot->setVisible(true);
  circle->setVisible(true);
  goals->setVisible(true);
  corners->setVisible(true);
  route->setVisible(true);

  connect(control_->show_view_range_, &QCheckBox::toggled,
          [=](bool checked) { viewRange->setVisible(checked); });

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