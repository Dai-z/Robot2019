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
#include "imb/MCLInfo.h"
#include "imb/AStarInfo.h"
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

  std::vector<geometry_msgs::Vector3>& getSimGoalPosts();

  std::vector<geometry_msgs::Vector3>& getSimLCorners();
  std::vector<geometry_msgs::Vector3>& getSimTCorners();

  std::vector<imb::ParticleInfo>& getParticles();
  std::vector<geometry_msgs::Vector3>& getRoute();

  // Setter
  void setSimRobotPos(QVector3D pos);
  void setSeeSimCircle(bool see);

  // Static methods
  static Model* getInstance();

  static bool showViewRange;
  static bool showParticles;

  static QPointF getSimBallPos();
  static void setSimBallPos(qreal x, qreal y);
  static void setSimRobotYaw(int yaw); 
  static void setWalking(bool flag); 

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

  //! goal points in robot coordinate from simulation
  std::vector<geometry_msgs::Vector3> goal_posts_sim_;
  //! T corners in robot coordinate from simulation
  std::vector<geometry_msgs::Vector3> T_corners_sim_;
  //! L corners in robot coordinate from simulation
  std::vector<geometry_msgs::Vector3> L_corners_sim_;

  //! ground truth robot posiiton in simualtion mode
  QVector3D robot_pos_sim_;
  //! ground truth ball posiiton in simualtion mode
  static QPointF ball_sim_;
  //! ground truth center circle posiiton in simualtion mode
  QPointF circle_sim_ = {0, 0};

  void AMCLCallback(const imb::MCLInfo::ConstPtr& msg);
  void AStarCallback(const imb::AStarInfo::ConstPtr& msg);

  // Route from AStar
  std::vector<geometry_msgs::Vector3> route_;

  //! Flag for whether or not model is enabled
  bool enabled_ = false;
  //! Flag for whether or not circle is seen in simulation mode
  bool see_circle_sim_ = false;

  //! Counter for looping cycle
  int cycle_ = 0;
  //! Timestamp of last time message is received
  QTime time_last_recv_;
  //! Lock
  std::mutex lock_;

  //! Ros node handle
  ros::NodeHandle* nh_;
  ros::Subscriber sub_amcl_info_;
  ros::Subscriber sub_astar_info_;
  ros::Publisher pub_mark_info_;
  ros::Publisher pub_motion_info_;

  int route_count_;
  bool walk_;
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