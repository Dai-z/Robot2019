#include "model.hpp"
#include <QDebug>
#include "utils.hpp"

Model* Model::instance_ = NULL;
bool Model::showParticles = false;
bool Model::showViewRange = false;
QPointF Model::ball_sim_ = QPointF(100, 0);

Model::Model(QObject* parent)
    : QObject(parent), robot_pos_sim_(-200, -300, 90) {
  nh_ = new ros::NodeHandle("~");

  sub_amcl_info_ =
      nh_->subscribe<imb::AMCLInfo>("/AMCL", 1, &Model::AMCLCallback, this);
  sub_astar_info_ =
      nh_->subscribe<imb::AstarInfo>("/AStar", 1, &Model::AstarCallback, this);
  pub_mark_info_ = nh_->advertise<imb::MarkInfo>("/Landmark", 1);

  // send sim info at 30fps
  sim_period_ = 10;
  {
    QTimer* t = new QTimer(this);
    connect(t, &QTimer::timeout, [this]() {
      // Update marks' info
      imb::MarkInfo marks;
      marks.target.x = ball_sim_.x();
      marks.target.y = ball_sim_.y();
      marks.see_circle = see_circle_sim_;
      if (see_circle_sim_) {
        marks.circle.x = circle_sim_.x();
        marks.circle.y = circle_sim_.y();
      } else {
        marks.circle.x = NAN;
        marks.circle.y = NAN;
      }
      for (auto g : goal_posts_sim_) marks.goal_posts.push_back(g);
      for (auto l : L_corners_sim_) marks.cornerL.push_back(l);
      for (auto t : T_corners_sim_) marks.cornerT.push_back(t);

      pub_mark_info_.publish(marks);
      ros::spinOnce();
    });
    t->start(sim_period_);
  }
}

Model::~Model() {}

bool Model::isEnabled() { return enabled_; }

// Setter & Getter

QVector3D Model::getLocRobotPos() {
  Lock l(lock_);
  return robot_pos_loc_;
}

QVector3D Model::getSimRobotPos() {
  Lock l(lock_);
  return robot_pos_sim_;
}

bool Model::getSimSeeCircle() {
  Lock l(lock_);
  return see_circle_sim_;
}

QPointF& Model::getSimCircle() {
  Lock l(lock_);
  return circle_sim_;
}

std::vector<geometry_msgs::Vector3>& Model::getSimGoalPosts() {
  Lock l(lock_);
  return goal_posts_sim_;
}

std::vector<geometry_msgs::Vector3>& Model::getSimLCorners() {
  Lock l(lock_);
  return L_corners_sim_;
}

std::vector<geometry_msgs::Vector3>& Model::getSimTCorners() {
  Lock l(lock_);
  return T_corners_sim_;
}

std::vector<imb::ParticleInfo>& Model::getParticles() {
  Lock l(lock_);
  return particles_loc_;
}

std::vector<geometry_msgs::Vector3>& Model::getRoute() {
  Lock l(lock_);
  return route_;
}

void Model::setSimRobotPos(QVector3D pos) {
  Lock l(lock_);
  robot_pos_sim_ = pos;
}

// Setter & Getter end

QPointF Model::getSimBallPos() { return ball_sim_; }

void Model::setSimBallPos(qreal x, qreal y) {
  ball_sim_.setX(x);
  ball_sim_.setY(y);
}

void Model::setSeeSimCircle(bool see) {
  Lock l(lock_);
  see_circle_sim_ = see;
}

Model* Model::getInstance() {
  if (!instance_) instance_ = new Model();
  return instance_;
}

void Model::setEnable(bool enabled) {
  enabled_ = enabled;
  // sendSimMotionInfo();
}

void Model::onSimRobotPosChanged(qreal x, qreal y, qreal angle) {
  Lock l(lock_);
  robot_pos_sim_.setX(x);
  robot_pos_sim_.setY(y);
  robot_pos_sim_.setZ(angle);
}

void Model::AMCLCallback(const imb::AMCLInfo::ConstPtr& msg) {
  Lock l(lock_);
  particles_loc_ = msg->particles;

  robot_pos_loc_ = Vector3ToQVector3D(msg->robot_pos);
}

void Model::AstarCallback(const imb::AstarInfo::ConstPtr& msg) {
  Lock l(lock_);
  route_ = msg->route;
}
