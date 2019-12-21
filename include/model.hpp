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
#include "imb/AMCLInfo.h"
#include "imb/AstarInfo.h"
#include "imb/MarkInfo.h"
#include "imb/ParticleInfo.h"
#include "ros/ros.h"

typedef std::lock_guard<std::mutex> Lock;
/**
 * @brief Field model in DViz
 */
class Model : public QObject {
  Q_OBJECT
 public:
  ~Model();
  bool isEnabled();

  // API for data manipulating

  // Getter
  QVector3D getLocRobotPos();
  QVector3D getSimRobotPos();

  QPointF& getSimCircle();
  bool getSimSeeCircle();

  std::vector<QPointF>& getLocWhitePoints();
  // std::vector<geometry_msgs::Vector3>& getSimWhitepoints();
  // std::vector<geometry_msgs::Vector3> getWhitePointsInViewRange();

  std::vector<imb::ParticleInfo>& getParticles();

  // std::vector<dmsgs::Line>& getLocWhiteLines();

  std::vector<geometry_msgs::Vector3>& getLocGoalPostsField();
  std::vector<geometry_msgs::Vector3>& getSimGoalPosts();

  // std::vector<geometry_msgs::Vector3>& getSimObstacles();

  // Setter
  void setSimRobotPos(QVector3D pos);
  void setSeeSimObstacle(bool see);
  void setSeeSimCircle(bool see);

  // Static methods
  static Model* getInstance();

  static bool showViewRange;
  static bool showParticles;

  static QPointF getSimBallPos();
  static void setSimBallPos(qreal x, qreal y);

  static QPointF getSimObstaclePos();
  static void setSimObstaclePos(qreal x, qreal y);

 private:
  Model(QObject* parent = 0);

  //! Set of instance of Model
  static Model* instance_;

  //! robot position from localization information
  QVector3D robot_pos_loc_ = {0, 0, 0};
  //! flag for whether circle is seen from localization information
  bool see_circle_loc_ = false;
  //! circle position in robot coordinate from localization information
  QPointF circle_field_loc_;

  //! AMCL particles from localization information
  std::vector<imb::ParticleInfo> particles_loc_;
  // //! goal points in robot coordinate from localization information
  // std::vector<geometry_msgs::Vector3> goal_posts_loc_;

  //! white points on robot coordinate from simualtion
  // std::vector<geometry_msgs::Vector3> white_points_sim_;
  // //! goal points in robot coordinate from simulation
  std::vector<geometry_msgs::Vector3> goal_posts_sim_;
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

  void AMCLCallback(const imb::AMCLInfo::ConstPtr& msg);
  void AstarCallback(const imb::AstarInfo::ConstPtr& msg);

  //! Flag for whether or not model is enabled
  bool enabled_ = false;
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

  //! Ros node handle
  ros::NodeHandle* nh_;
  ros::Subscriber sub_amcl_info_;
  ros::Subscriber sub_astar_info_;
  ros::Publisher pub_mark_info_;

  std::queue<QVector3D> route_points_;
  //! aux final_dest_
  QVector3D last_final_dest_;
  //! simulation period in ms
  double sim_period_;

 public slots:
  void setEnable(bool enabled);
  /**
   * @brief Update robot position inj simulation mode when it has been changed
   *
   * @param x - position x
   * @param y - posiiton y
   * @param angle - field angle
   */
  void onSimRobotPosChanged(qreal x, qreal y, qreal angle);

 signals:
  void ballPoseChange(QPointF newPose);
};