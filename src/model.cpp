#include "model.hpp"
#include <QDebug>
#include "qutils.hpp"

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
  pub_mark_info_ = nh_->advertise<imb::MarkInfo>("/LandMark", 1);
  pub_motion_info_ = nh_->advertise<geometry_msgs::Vector3>("/MotionDelta", 1);

  route_count_ = 0;
  walk_ = false;

  // send sim info at 30fps
  sim_period_ = 10;
  {
    QTimer* t = new QTimer(this);
    connect(t, &QTimer::timeout, [this]() {
      // Update marks' info
      imb::MarkInfo marks;
      // Update target info
      marks.target.x = ball_sim_.x();
      marks.target.y = ball_sim_.y();
      double t = std::atan(-marks.target.y / (450 - marks.target.x));
      marks.target.z = RadianToDegree(t);

      // Update circle info
      marks.see_circle = see_circle_sim_;
      if (see_circle_sim_) {
        marks.circle.x = circle_sim_.x();
        marks.circle.y = circle_sim_.y();
      } else {
        marks.circle.x = NAN;
        marks.circle.y = NAN;
      }

      // Update goals and corners
      for (auto g : goal_posts_sim_) marks.goal_posts.push_back(g);
      for (auto l : L_corners_sim_) marks.cornerL.push_back(l);
      for (auto t : T_corners_sim_) marks.cornerT.push_back(t);

      pub_mark_info_.publish(marks);
      // Simulate walking
      if (walk_ && (++route_count_ % 30 == 0) && route_.size() > 1) {
        route_count_ %= 30;
        geometry_msgs::Vector3 delta, next;
        next.x = route_[1].x;
        next.y = route_[1].y;
        next.z = route_[1].z;
        delta.x = next.x - route_[0].x;
        delta.y = next.y - route_[0].y;
        delta.z = next.z;
        robot_pos_sim_.setX(next.x - route_[0].x + robot_pos_sim_.x());
        robot_pos_sim_.setY(next.y - route_[0].y + robot_pos_sim_.y());
        robot_pos_sim_.setZ(next.z);
        pub_motion_info_.publish(delta);
      }
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

void Model::setSimRobotYaw(int yaw) {
  getInstance()->robot_pos_sim_.setZ(yaw);
  getInstance()->robot_pos_loc_.setZ(yaw);
  geometry_msgs::Vector3 msg;
  msg.z = yaw;
  getInstance()->pub_motion_info_.publish(msg);
}

void Model::setWalking(bool flag) {
  getInstance()->walk_ = flag;
  getInstance()->route_count_ = 0;
}