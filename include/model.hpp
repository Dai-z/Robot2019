#pragma once

#include <ros/service_server.h>
#include <QDebug>
#include <QObject>
#include <QPointF>
#include <QTime>
#include <QVector2D>
#include <QVector3D>
#include <QtCore>
#include <array>
#include <mutex>
#include <queue>
#include <vector>
#include "ros/ros.h"

typedef std::lock_guard<std::mutex> Lock;
/**
 * @brief Field model in DViz
 */
class Model : public QObject {
  Q_OBJECT
 public:
  /**
   * @brief Model destructor
   */
  ~Model();
  /**
   * @brief Check whether field model is enabled
   *
   * @return
   */
  bool isEnabled();
  /**
   * @brief Check whether robot is connected
   *
   * @return
   */
  bool isConnected();

  // API for data manipulating

  // Getter
  /**
   * @brief Get self-localized robot position
   *
   * @return robot position from localization
   */
  QVector3D getLocRobotPos();
  /**
   * @brief Get real robot position in simulation mode
   *
   * @return real robot position in simulation mode
   */
  QVector3D getSimRobotPos();
  /**
   * @brief Get motion delta
   *
   * @return motion delta
   */
  QVector3D getMotionDelta();

  /**
   * @brief Check whether center circle is seen from localization information.
   *
   * @return whether center circle is seen from localization information
   */
  bool getLocSeeCircle();
  /**
   * @brief Get circle position in robot coordinate from localization
   * information.
   *
   * @return circle position in robot coordinate from localization information
   */
  QPointF getLocCircle();

  /**
   * @brief Get robot view range on field from localization information.
   *
   * @return robot view range on field in robot coordinate from localization
   * information
   */
  std::vector<QPointF>& getLocViewRangeField();
  /**
   * @brief Get white points from localization information
   *
   * @return white points in robot coordinate from localization information
   */
  std::vector<QPointF>& getLocWhitePoints();
  /**
   * @brief Get AMCL particles
   *
   * @return particles
   */
  // std::vector<dmsgs::ParticleMsg>& getParticles();
  /**
   * @brief Get white line from localization information
   *
   * @return white lines in robot coordinate from localization information
   */
  // std::vector<dmsgs::Line>& getLocWhiteLines();
  /**
   * @brief Get goal points from localization information
   *
   * @return goal points in robot coordinate from localization information
   */
  // std::vector<geometry_msgs::Vector3>& getLocGoalPostsField();
  /**
   * @brief Get obstacle positions from localization information
   *
   * @return obstacle positions in robot coordinate from localization
   * information
   */
  // std::vector<geometry_msgs::Vector3>& getLocObstaclesField();

  /**
   * @brief Get rasterized map of AMCL
   *
   * @return rasterized map of AMCL
   */
  // dvision::Map& getMap();

  /**
   * @brief Get white points in simulation mode
   *
   * @return white points on robot coordinate from simualtion
   */
  // std::vector<geometry_msgs::Vector3>& getSimWhitepoints();
  /**
   * @brief Get goal points from simulation
   *
   * @return goal points in robot coordinate from simulation
   */
  // std::vector<geometry_msgs::Vector3>& getSimGoalPosts();
  /**
   * @brief Get obstacle positions from simualtion
   *
   * @return obstacle positions in robot coordinate from simualtion
   */
  // std::vector<geometry_msgs::Vector3>& getSimObstacles();
  /**
   * @brief Get white points in view range of robot
   *
   * @return white points in view range of robot
   */
  // std::vector<geometry_msgs::Vector3> getWhitePointsInViewRange();

  // Setter
  /**
   * @brief Set robot posiiton in simualtion mode
   *
   * @param pos - desired robot position
   */
  void setSimRobotPos(QVector3D pos);
  /**
   * @brief Set whether ball is seen in simualtion mode
   *
   * @param see - whether ball is seen
   */
  void setSeeSimBall(bool see);
  /**
   * @brief Set whether obstacle is seen in simualtion mode
   *
   * @param see - whether obstacle is seen
   */
  void setSeeSimObstacle(bool see);
  /**
   * @brief Set whether circle is seen in simualtion mode
   *
   * @param see - whether circle is seen
   */
  void setSeeSimCircle(bool see);

 private:
  //! robot position from localization information
  QVector3D robot_pos_loc_ = {0, 0, 0};
  //! motion delta in previous
  QVector3D prev_delta_ = {0, 0, 0};
  //! motion delta at present
  QVector3D motion_delta_ = {0, 0, 0};
  //! flag for whether circle is seen from localization information
  bool see_circle_loc_ = false;
  //! circle position in robot coordinate from localization information
  QPointF circle_field_loc_;

  //! robot view range on field in robot coordinate from localization
  //! information
  std::vector<QPointF> view_range_field_loc_;
  //! AMCL particles from localization information
  // std::vector<dmsgs::ParticleMsg> particles_loc_;
  // //! goal points in robot coordinate from localization information
  // std::vector<geometry_msgs::Vector3> goal_posts_field_loc_;
  // //! obstacle positions in robot coordinate from localization information
  // std::vector<geometry_msgs::Vector3> obstacles_field_loc_;
  // //! white points in robot coordinate from localization information
  // std::vector<QPointF> white_points_loc_;
  // //! white lines in robot coordinate from localization information
  // std::vector<dmsgs::Line> lines_field_loc_;

  //! left goal post point from localization information
  QPointF goal_left_loc_;
  //! right goal post point from localization information
  QPointF goal_right_loc_;
  //! unknown goal post point from localization information
  QPointF goal_unknown_loc_;
  //! direct destination position of robot
  QVector3D dest_;
  //! final destination position of robot
  QVector3D final_dest_;

  //! white points on robot coordinate from simualtion
  // std::vector<geometry_msgs::Vector3> white_points_sim_;
  // //! goal points in robot coordinate from simulation
  // std::vector<geometry_msgs::Vector3> goal_posts_sim_;
  // //! obstacles in robot coordinate from simulation
  // std::vector<geometry_msgs::Vector3> obstacles_sim_;

  //! ground truth robot posiiton in simualtion mode
  QVector3D robot_pos_sim_;
  //! ground truth ball posiiton in simualtion mode
  static QPointF ball_sim_;
  //! ground truth obstacle posiiton in simualtion mode
  static QPointF obstacle_sim_;
  //! ground truth center circle posiiton in simualtion mode
  QPointF circle_sim_ = {0, 0};

 signals:
  /**
   * @brief signal when robot connection status has changed
   *
   * @param connected - current connection status
   */
  void robotConnectionStatusChanged(int id, bool connected);

 public slots:
  /**
   * @brief Switch the status of field model
   *
   * @param enabled - desired status of model
   */
  void setEnable(bool enabled);
  /**
   * @brief Update robot position inj simulation mode when it has been changed
   *
   * @param x - position x
   * @param y - posiiton y
   * @param angle - field angle
   */
  void onSimRobotPosChanged(qreal x, qreal y, qreal angle);

  // static methods
 public:
  /**
   * @brief Get model instance
   *
   * @return model instance
   */
  static Model* getInstance();
  /**
   * @brief Flag for whether or not to show view range of robot
   */
  static bool showViewRange;
  /**
   * @brief Flag for whether or not to show particles from AMCL
   */
  static bool showParticles;

  /**
   * @brief Get ground truth of ball position in simulation mode
   *
   * @return ground truth of ball position in simulation mode
   */
  static QPointF getSimBallPos();
  /**
   * @brief Set ball position in simulation mode
   *
   * @param x - desired ball posiiton x
   * @param y - desired ball posiiton y
   */
  static void setSimBallPos(qreal x, qreal y);
  /**
   * @brief Get ground truth of obstacle position in simulation mode
   *
   * @return ground truth of obstacle position in simulation mode
   */
  static QPointF getSimObstaclePos();
  /**
   * @brief Set obstacle position in simulation mode
   *
   * @param x - desired obstacle posiiton x
   * @param y - desired obstacle posiiton y
   */
  static void setSimObstaclePos(qreal x, qreal y);

 private:
  /**
   * @brief Model constructor
   *
   * @param parent - parent of QObject
   */
  Model(QObject* parent = 0);
  /**
   * @brief Callback function on receiving VisionInfo
   *
   * @param msg - VisionInfo
   */
  // void onRecvVisionInfo(dmsgs::VisionInfo& msg);
  /**
   * @brief Callback function on receiving MotionInfo
   *
   * @param msg - MotionInfo
   */
  // void onRecvMotionInfo(dmsgs::MotionInfo& msg);
  /**
   * @brief Send fake VisionInfo on simualtion mode
   */

 private:
  //! Flag for whether or not robot is connected
  bool connected_ = false;
  //! Flag for whether or not model is enabled
  bool enabled_ = false;
  //! Flag for whether or not ball is seen in simulation mode
  bool see_ball_sim_ = false;
  //! Flag for whether or not obstacle is seen in simulation mode
  bool see_obstacle_sim_ = false;
  //! Flag for whether or not circle is seen in simulation mode
  bool see_circle_sim_ = false;

  //! Counter for looping cycle
  int cycle_ = 0;
  //! Rasterized map of AMCL
  // dvision::Map map_;
  //! Timestamp of last time message is received
  QTime time_last_recv_;
  //! Lock
  std::mutex lock_;

  //! Set of instance of Model
  static Model* instance_;

  /// Temporarily add for test
 public:
  // void onRecvActionCommand(const dmsgs::ActionCommand::ConstPtr& msg);

  // void onRecvBehaviorInfo(const dmsgs::BehaviorInfo::ConstPtr& msg);

  void sendSimMotionInfo();

  void sendSimVisionInfo();

  void updateSimViewRange(double pitch, double yaw);

  // bool srvSetYaw(dmsgs::SetInitOrientation::Request &req,
  // dmsgs::SetInitOrientation::Response &res);

  // bool srvResetParticlesPoint(dmsgs::ResetParticlePoint::Request &req,
  //                             dmsgs::ResetParticlePoint::Response &res);

  // bool srvResetParticlesLeftTouch(dmsgs::ResetParticleLeftTouch::Request
  // &req,
  //                                 dmsgs::ResetParticleLeftTouch::Response
  //                                 &res);

  // bool srvResetParticlesRightTouch(dmsgs::ResetParticleRightTouch::Request
  // &req,
  //                                  dmsgs::ResetParticleRightTouch::Response
  //                                  &res);

  // bool srvResetParticlesTouchLine(dmsgs::ResetParticleTouchLine::Request
  // &req,
  //                                 dmsgs::ResetParticleTouchLine::Response
  //                                 &res);

 signals:
  void ballPoseChange(QPointF newPose);

 private:
  //! Ros node handle
  ros::NodeHandle* nh_;
  //!
  ros::Subscriber sub_behavior_info_;
  //!
  ros::Subscriber sub_action_command_;
  //!
  ros::Publisher pub_vision_info_;
  //!
  ros::Publisher pub_motion_info_;

  // dvision::Projection projection_;
  //! service provide by motion
  ros::ServiceServer srv_set_yaw_;
  //! service provide by vision
  ros::ServiceServer srv_reset_particle_point_;
  //!
  ros::ServiceServer srv_reset_particle_touch_left_;
  //!
  ros::ServiceServer srv_reset_particle_touch_right_;
  //!
  ros::ServiceServer srv_reset_particle_touch_line_;

  std::queue<QVector3D> route_points_;
  //! aux final_dest_
  QVector3D last_final_dest_;
  //! set if going to kick
  bool kicking_;
  //! set when kick is executing
  bool kick_finished_;
  //! direction of kicking
  double kick_direction_;
  //! simulation period in ms
  double sim_period_;

  //! static stuffs
 public:
  static void setMoveSpeed(int val);
  static double getMoveSpeed();

 private:
  static double move_spd_;
};