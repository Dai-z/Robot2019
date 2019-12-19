#include "model.hpp"

Model* Model::instance_ = NULL;
bool Model::showParticles = false;
bool Model::showViewRange = false;
QPointF Model::ball_sim_ = QPointF(100, 0);
QPointF Model::obstacle_sim_ = QPointF(300, 300);
double Model::move_spd_ = 1;

Model::Model(QObject* parent)
    : QObject(parent),
      robot_pos_sim_(-200, -300, 90),
      final_dest_(-200, -300, 90),
      last_final_dest_(final_dest_) {
  //   map_.Init();

  nh_ = new ros::NodeHandle("~");

  //   pub_vision_info_ =
  //       nh_->advertise<VisionInfo>("/vision/VisionInfo", 1);
  //   pub_motion_info_ =
  //       nh_->advertise<MotionInfo>("/motion/MotionInfo", 1);

  //   srv_set_yaw_ = nh_->advertiseService("/motion/set_motion_yaw",
  //                                        &Model::srvSetYaw, this);

  //   srv_reset_particle_point_ =
  //       nh_->advertiseService("/dvision_" + std::to_string(id_) +
  //       "/reset_particles_point",
  //                             &Model::srvResetParticlesPoint, this);
  //   srv_reset_particle_touch_left_ =
  //       nh_->advertiseService("/dvision_" + std::to_string(id_) +
  //       "/reset_particles_left_touch",
  //                             &Model::srvResetParticlesLeftTouch, this);
  //   srv_reset_particle_touch_right_ =
  //       nh_->advertiseService("/dvision_" + std::to_string(id_) +
  //       "/reset_particles_right_touch",
  //                             &Model::srvResetParticlesRightTouch, this);
  //   srv_reset_particle_touch_line_ =
  //       nh_->advertiseService("/dvision_" + std::to_string(id_) +
  //       "/reset_particles_touch_line",
  //                             &Model::srvResetParticlesTouchLine, this);

  // send sim info at 30fps
  sim_period_ = 10;
  {
    QTimer* t = new QTimer(this);
    connect(t, &QTimer::timeout, [this]() {
      //   sendSimVisionInfo();
      //   sendSimMotionInfo();
    });
    t->start(sim_period_);
  }
}

Model::~Model() {}

bool Model::isEnabled() { return enabled_; }

bool Model::isConnected() { return connected_; }

// Setter & Getter

QVector3D Model::getLocRobotPos() {
  Lock l(lock_);
  return robot_pos_loc_;
}

QVector3D Model::getSimRobotPos() {
  Lock l(lock_);
  return robot_pos_sim_;
}

QVector3D Model::getMotionDelta() {
  Lock l(lock_);
  return motion_delta_;
}

bool Model::getLocSeeCircle() {
  Lock l(lock_);
  return see_circle_loc_;
}

QPointF Model::getLocCircle() {
  Lock l(lock_);
  return circle_field_loc_;
}

std::vector<QPointF>& Model::getLocViewRangeField() {
  Lock l(lock_);
  return view_range_field_loc_;
}

// std::vector<QPointF>& Model::getLocWhitePoints() {
//   Lock l(lock_);
//   return white_points_loc_;
// }

// std::vector<dmsgs::ParticleMsg>& Model::getParticles() {
//   Lock l(lock_);
//   return particles_loc_;
// }

// std::vector<dmsgs::Line>& Model::getLocWhiteLines() {
//   Lock l(lock_);
//   return lines_field_loc_;
// }

// std::vector<geometry_msgs::Vector3>& Model::getLocGoalPostsField() {
//   Lock l(lock_);
//   return goal_posts_field_loc_;
// }

// std::vector<geometry_msgs::Vector3>& Model::getLocObstaclesField() {
//   Lock l(lock_);
//   return obstacles_field_loc_;
// }

// dvision::Map& Model::getMap() {
//   Lock l(lock_);
//   return map_;
// }

// std::vector<geometry_msgs::Vector3>& Model::getSimWhitepoints() {
//   Lock l(lock_);
//   return white_points_sim_;
// }

// std::vector<geometry_msgs::Vector3>& Model::getSimGoalPosts() {
//   Lock l(lock_);
//   return goal_posts_sim_;
// }

// std::vector<geometry_msgs::Vector3>& Model::getSimObstacles() {
//   Lock l(lock_);
//   return obstacles_sim_;
// }

void Model::setSimRobotPos(QVector3D pos) {
  Lock l(lock_);
  robot_pos_sim_ = pos;
}

void Model::setSeeSimBall(bool see) {
  Lock l(lock_);
  see_ball_sim_ = see;
}

void Model::setSeeSimObstacle(bool see) {
  Lock l(lock_);
  see_obstacle_sim_ = see;
}

// Setter & Getter end

QPointF Model::getSimBallPos() { return ball_sim_; }

void Model::setSimBallPos(qreal x, qreal y) {
  ball_sim_.setX(x);
  ball_sim_.setY(y);
}

QPointF Model::getSimObstaclePos() { return obstacle_sim_; }

void Model::setSimObstaclePos(qreal x, qreal y) {
  obstacle_sim_.setX(x);
  obstacle_sim_.setY(y);
}

void Model::setSeeSimCircle(bool see) {
  Lock l(lock_);
  see_circle_sim_ = see;
}

// void getVec(vector<QPointF>& res, const std::vector<geometry_msgs::Vector3>&
// points) {
//   res.resize(points.size());
//   for (uint32_t i = 0; i < res.size(); ++i) {
//     res[i].setX(points[i].x);
//     res[i].setY(points[i].y);
//   }
// }

// void Model::onRecvBehaviorInfo(const dmsgs::BehaviorInfo::ConstPtr& msg) {
// dest_ = Vector3ToQVector3D(msg->dest);
// final_dest_ = Vector3ToQVector3D(msg->final_dest);
// }

// void Model::sendSimVisionInfo() {
//   Lock l(lock_);
//   if (!connected_) return;
//   dmsgs::VisionInfo info;
//   updateBallTrack();
//   info.see_ball = see_ball_sim_;
//   info.ball_field = QPointFToVector3(getFieldPosition(robot_pos_sim_,
//   ball_sim_)); info.ball_global = QPointFToVector3(getSimBallPos());
//   info.ballTrack.pitch = RadianToDegree(ball_tracker_.out_pitch());
//   info.ballTrack.yaw = RadianToDegree(ball_tracker_.out_yaw());

//   info.see_circle = see_circle_sim_;
//   info.circle_field = QPointFToVector3(getFieldPosition(robot_pos_sim_,
//   circle_sim_));

//   // info.simFieldWhitePoints = white_points_sim_;
//   info.simYaw = robot_pos_sim_.z();

//   info.goals_field = goal_posts_sim_;

//   info.see_obstacle = see_obstacle_sim_;
//   info.obstacles_field = obstacles_sim_;

//   info.robot_pos = QVector3DToVector3(robot_pos_sim_);

// //   pub_vision_info_.publish(info);
// }

Model* Model::getInstance() {
  if (!instance_) instance_ = new Model();
  return instance_;
}

void Model::setEnable(bool enabled) {
  enabled_ = enabled;
  connected_ = enabled;
  // sendSimMotionInfo();
}

void Model::onSimRobotPosChanged(qreal x, qreal y, qreal angle) {
  Lock l(lock_);
  robot_pos_sim_.setX(x);
  robot_pos_sim_.setY(y);
  robot_pos_sim_.setZ(angle);
}

// void Model::onRecvActionCommand(const dmsgs::ActionCommand::ConstPtr& msg) {
//   // Lock l(lock_);

//   switch (msg->bodyCmd.gait_type) {
//     case dmsgs::BodyCommand::WALK_POS: {
//       kicking_ = false;
//       last_final_dest_ = final_dest_;
//       final_dest_.setX(msg->bodyCmd.x);
//       final_dest_.setY(msg->bodyCmd.y);
//       final_dest_.setZ(msg->bodyCmd.t);
//     } break;
//     case dmsgs::BodyCommand::KICK_BALL: {
//       kicking_ = true;
//       last_final_dest_ = final_dest_;
//       QVector3D ball_pose;
//       ball_pose.setX(msg->bodyCmd.x);
//       ball_pose.setY(msg->bodyCmd.y);
//       ball_pose.setZ(msg->bodyCmd.t);
//       final_dest_ = calKickPose(ball_pose);
//       kick_direction_ = msg->bodyCmd.t;
//     } break;
//     case dmsgs::BodyCommand::CROUCH: {
//       // clearRoute();
//       //! TODO: Add below when real path planning is ready
//       // final_dest_ = getSimRobotPos();
//     } break;
//     case dmsgs::BodyCommand::TURN: {
//       auto new_pose = getSimRobotPos();
//       new_pose[2] = msg->bodyCmd.t;
//       final_dest_ = new_pose;
//       // kicking_ = false;
//       // last_final_dest_ = final_dest_;
//       // final_dest_ = getSimRobotPos();
//       // final_dest_.setZ(msg->bodyCmd.t);
//       // updateRoute();
//     } break;
//     default:
//       break;
//   }

//   tar_plat_[0] = msg->headCmd.pitch;
//   tar_plat_[1] = msg->headCmd.yaw;
//   cur_plat_spd_[0] = msg->headCmd.pitchSpeed;
//   cur_plat_spd_[1] = msg->headCmd.yawSpeed;

//   // auto dx = msg->bodyCmd.x;
//   // auto dy = msg->bodyCmd.y;
//   // auto dt = msg->bodyCmd.t;

//   // motion_delta_.setX(dx);
//   // motion_delta_.setY(dy);
//   // motion_delta_.setZ(dt);
//   // auto updatedPos = getGlobalPosition(robot_pos_sim_, motion_delta_);
//   // robot_pos_sim_ = updatedPos;
// }

// bool Model::srvResetParticlesPoint(dmsgs::ResetParticlePoint::Request& req,
//                                    dmsgs::ResetParticlePoint::Response& res)
//                                    {
//   // Lock l(lock_);
//   setSimRobotPos(Vector3ToQVector3D(req.point));
//   final_dest_ = getSimRobotPos();
//   status_ = STANDBY;
//   return true;
// }

// bool
// Model::srvResetParticlesLeftTouch(dmsgs::ResetParticleLeftTouch::Request&
// req,
//                                        dmsgs::ResetParticleLeftTouch::Response&
//                                        res) {
//   // Lock l(lock_);
//   setSimRobotPos(QVector3D(-200, -300, 90));
//   final_dest_ = getSimRobotPos();
//   status_ = STANDBY;
//   return true;
// }

// bool
// Model::srvResetParticlesRightTouch(dmsgs::ResetParticleRightTouch::Request&
// req,
//                                         dmsgs::ResetParticleRightTouch::Response&
//                                         res) {
//   // Lock l(lock_);
//   setSimRobotPos(QVector3D(200, -300, 90));
//   final_dest_ = getSimRobotPos();
//   status_ = STANDBY;
//   return true;
// }

// bool
// Model::srvResetParticlesTouchLine(dmsgs::ResetParticleTouchLine::Request&
// req,
//                                        dmsgs::ResetParticleTouchLine::Response&
//                                        res) {
//   // Lock l(lock_);
//   if (req.side == dmsgs::ResetParticleTouchLineRequest::TOUCH_LINE_LEFT_BOTH)
//   {
//     setSimRobotPos(QVector3D(-200, -300, 90));
//   } else if (req.side ==
//   dmsgs::ResetParticleTouchLineRequest::TOUCH_LINE_RIGHT_BOTH) {
//     setSimRobotPos(QVector3D(200, -300, 90));
//   } else if (req.side ==
//   dmsgs::ResetParticleTouchLineRequest::TOUCH_LINE_RIGHT_TOP) {
//     setSimRobotPos(QVector3D(200, 300, -90));
//   } else if (req.side ==
//   dmsgs::ResetParticleTouchLineRequest::TOUCH_LINE_RIGHT_BOTTOM) {
//     setSimRobotPos(QVector3D(200, -300, 90));
//   } else if (req.side ==
//   dmsgs::ResetParticleTouchLineRequest::TOUCH_LINE_LEFT_TOP) {
//     setSimRobotPos(QVector3D(-200, 300, -90));
//   } else if (req.side ==
//   dmsgs::ResetParticleTouchLineRequest::TOUCH_LINE_LEFT_BOTTOM) {
//     setSimRobotPos(QVector3D(-200, -300, 90));
//   }
//   final_dest_ = getSimRobotPos();
//   status_ = STANDBY;
//   return true;
// }

void Model::setMoveSpeed(int val) { move_spd_ = val / 10.f; }

double Model::getMoveSpeed() { return move_spd_; }