// #include "ball.hpp"
// #include "circle.hpp"
// #include "control_widget.hpp"
// #include "dest.hpp"
#include "field.hpp"
// #include "goals.hpp"
#include "item_manager.hpp"
// #include "obstacles.hpp"
// #include "particles.hpp"
#include "robot.hpp"
// #include "viewrange.hpp"
// #include "whitelines.hpp"
// #include "whitepoints.hpp"
// #include "world.hpp"
#include <algorithm>

ItemManager::ItemManager(QObject* parent, QGraphicsScene* scene)
    : QObject(parent), scene_(scene) {}

ItemManager::~ItemManager() {}

void ItemManager::Init() {
  // Add field, ball and obstacle
  scene_->addItem(new FieldItem());
  //   // scene_->addItem(new World());
  //   auto simBall = new Ball(false, 0);
  //   simBall->setVisible(false);
  //   scene_->addItem(simBall);
  //   auto simObstacle = new Obstacles(false, 0);
  //   simObstacle->setVisible(false);
  //   scene_->addItem(simObstacle);

  // Set up simulated robot, which should not appear in MONITOR mode
  //   auto vecSimRobot = std::vector<Robot*>();

  // Monitoring & Simulating
  auto simRobot = new Robot(false);
  //   vecSimRobot.push_back(simRobot);

  auto locRobot = new Robot(true);
  //   auto locBall = new Ball(true, i);
  //   auto p = new Particles(i);
  //   auto locCircle = new Circle(i);
  //   auto locLines = new WhiteLines(i);
  //   auto locWhitePoints = new WhitePoints(i);
  //   auto viewRange = new ViewRange(i);
  //   auto goals = new Goals(i);
  //   auto obstacles = new Obstacles(true, i);
  //   auto dest = new Dest(i);

  scene_->addItem(locRobot);
  scene_->addItem(simRobot);
  //   scene_->addItem(locBall);
  //   scene_->addItem(p);
  //   scene_->addItem(locCircle);
  //   scene_->addItem(viewRange);
  //   scene_->addItem(locWhitePoints);
  //   scene_->addItem(goals);
  //   scene_->addItem(obstacles);
  //   scene_->addItem(dest);

  // simRobot->setVisible(true);
  locRobot->setVisible(true);

  // Sim Robot, ball and obstacle is only visiable for simulation mode
  // connect(control_->mode_, &QComboBox::currentTextChanged, [=](QString s) {
  //     MODE mode = s == "Monitor" ? MONITOR : SIMULATOR;
  //     if (mode == MONITOR) {
  //         simRobot->setVisible(false);
  //         simBall->setVisible(false);
  //         simObstacle->setVisible(false);
  //     } else {
  //         simBall->setVisible(true);
  //         simObstacle->setVisible(true);

  //         locRobot->setVisible(false);
  //         locBall->setVisible(false);
  //         obstacles->setVisible(false);
  //     }
  // });

  // Loc items is both for monitor and simulation mode
  //     connect(control_->show_robot_[i], &QCheckBox::toggled, [=](bool
  //     checked) {
  //         // locRobot->setVisible(checked);
  //         // locBall->setVisible(checked);
  //         locCircle->setVisible(checked);
  //         locLines->setVisible(checked);
  //         locWhitePoints->setVisible(checked);
  //         viewRange->setVisible(checked);
  //         goals->setVisible(checked);
  //         // obstacles->setVisible(checked);
  //         dest->setVisible(checked);

  //         if (!checked) {
  //             p->setVisible(false);
  //         } else {
  //             if (control_->show_particle_->checkState() == Qt::Checked)
  //                 p->setVisible(true);
  //         }

  //         if (Model::getMode() == SIMULATOR) {
  //             simRobot->setVisible(checked);
  //         }
  //     });

  //     // Particles and view range is both for monitor and simulation mode
  //     connect(control_->show_particle_, &QCheckBox::toggled, [=](bool
  //     checked) {
  //         if (control_->show_robot_[i]->checkState() == Qt::Checked) {
  //             p->setVisible(checked);
  //         }
  //     });
  // }

  // 30 fps rendering
  auto t = new QTimer(this);
  connect(t, &QTimer::timeout, [=]() {
    //     // handle collide
    //     if (simBall->isVisible()) {
    //         auto ballx = simBall->x();
    //         auto bally = simBall->y();
    //         for (size_t i = 0; i < vecSimRobot.size(); ++i) {
    //             auto r = vecSimRobot[i];
    //             if (!r->isVisible())
    //                 continue;

    //             auto model = Model::getInstance(i + 1);
    //             auto simRobotPos = model->getSimRobotPos();
    //             auto ballField = getFieldPosition(simRobotPos, ballx, bally);

    //             if (fabs(ballField.x()) > 10 || fabs(ballField.y()) > 10) {
    //                 return;
    //             }

    //             ballField.setX(ballField.x() + ballField.x() > 0 ? 10 : -10);
    //             // ballField.setY(ballField.y() + ballField.y() > 0 ? 15 :
    //             -15);

    //             auto nb = getGlobalPosition(simRobotPos,
    //             QPointF(ballField.x(), ballField.y()));

    //             simBall->setX(nb.x());
    //             simBall->setY(nb.y());
    //         }
    //     }
    // if (simObstacle->isVisible()) {
    //     for (size_t i = 0; i < vecSimRobot.size(); ++i) {
    //         auto model = Model::getInstance(i + 1);
    //         auto r = vecSimRobot[i];
    //         if (!r->isVisible())
    //             continue;
    //
    //         for (auto& obstacle : model->getSimObstacles()) {
    //
    //             auto simRobotPos = model->getSimRobotPos();
    //             auto obstacleField = getFieldPosition(simRobotPos,
    //             obstacle.x, obstacle.y);
    //
    //             if (fabs(obstacleField.x()) > 10 || fabs(obstacleField.y()) >
    //             10) {
    //                 return;
    //             }
    //
    //             simRobotPos.setX(simRobotPos.x() + obstacleField.x() > 0 ? 10
    //             : -10);
    //             // simRobotPos.setY(simRobotPos.y() + obstacleField.y() > 0 ?
    //             15 : -15);
    //
    //             model->setSimRobotPos(simRobotPos);
    //         }
    //     }
    // }
  });
  t->start(1000 / 30.f);
}