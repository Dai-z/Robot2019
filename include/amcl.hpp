#pragma once
#include <ros/ros.h>
#include <array>
#include "geometry_msgs/Vector3.h"
#include "imb/MarkInfo.h"
#include "kdtree.hpp"
#include "types.hpp"

//! Information for a set of sample particles
struct SampleSet {
  //! SampleSet destructor
  ~SampleSet() { delete kdtree; }

  //! Samples of particles
  std::vector<Particle> samples;
  //! A kdtree encoding the histogtam
  KdTree* kdtree = nullptr;

  // Clusters
  //! Count of clusters
  int cluster_count = 0;
  //! Maximum count of clusters
  int cluster_max_count;
  //! Set of clusters
  std::vector<Cluster> clusters;

  // Filter statistics
  //! Mean value for cluster statistics
  Pose mean;
  //! Covariance value for cluster statistics
  Matrix cov;
  //! Flag for whether or not sample set is converged
  bool converged = false;
};

/**
 * @brief class for Adaptive (or KLD-sampling) Monte Carlo localization
 */
class AMCL {
 public:
  //! AMCL constructor
  AMCL(ros::NodeHandle* nh);

  void step();

  std::vector<Particle>& particles() { return sets_[current_set_].samples; }
  inline Pose GetEstimatedPose() { return pose_; }
  bool IsConverged() { return converged_; }

  void falldownGauss();
  /**
   * @brief Get field quality
   *
   * @return field quality
   */
  inline double GetQuality() { return quality_; }

 private:
  /**
   * @brief Sample with motion model
   */
  void SampleMotionModel();
  /**
   * @brief Sample with Measurement Model
   */
  void MeasurementModel();
  /**
   * @brief Resample with Measurement Model
   */
  void Resample();
  /**
   * @brief Estimate location
   */
  void Estimate();
  /**
   * @brief Check whether or not sample set is convergent
   *
   * @return whether or not sample set is convergent
   */
  bool CheckConverged();
  /**
   * @brief Generate random pose
   *
   * @return a random pose
   */
  Pose RandomPose(const bool& use_yaw = false);
  /**
   * @brief Calculate statistics of cluster
   *
   * @param [in] set - sample set
   */
  void ClusterStats(SampleSet& set);
  /**
   * @brief Update consistency
   *
   * @param [in] z - detected measurement of land marks
   */
  void UpdateConsistency();
  /**
   * @brief calculate score of detected mark position(goal posts or center)
   *
   * @param [in] detected_p - detected mark position(goal posts or center)
   *
   * @return the score of the detected mark position(goal posts or center)
   */
  double GetScore(geometry_msgs::Vector3 detected_p);

  //! Number of particles
  int num_particles_;
  //! Count of cycles
  int cycle_ = 0;

  //! The sample sets.
  //* We keep two sets and use [current_set] to identify the active set.
  SampleSet sets_[2];
  //! Index for current active set
  int current_set_ = 0;

  //! Pose
  Pose pose_;

  // Parameters for AMCL
  //! Running averages (slow) of likelihood
  double w_slow_ = 0.0;
  //! Running averages (fast) of likelihood
  double w_fast_ = 0.0;

  //! Decay rates (slow) for running averages
  double alpha_slow_ = 0.001;
  //! Decay rates (fast) for running averages
  double alpha_fast_ = 0.1;

  // Parameters for laser probability model?
  double z_hit_ = 0.95;
  double z_rand_ = 0.95;
  double sigma_hit_ = 0.2;

  //! Cycle interval for resampling
  int resample_interval_ = 5;

  //! Distance threshold in each axis over which the pf is considered to not be
  //! converged
  double dist_threshold_ = 50;
  //! Flag for whether or not pf is converged
  bool converged_ = false;

  //! Field angle of robot
  double yaw_ = 0;

  // Gaissian noise
  //! Gaussian distribution for x axis
  Gaussian y_gauss_;
  //! Gaussian distribution for y axis
  Gaussian x_gauss_;
  //! Gaussian distribution for resampling
  Gaussian resample_gauss_;
  //! Uniform distribution
  Uniform uniform_;

  // Field quality
  double quality_;
  // Threshold to determin a particle is good or bad
  double good_dist_th_;
  double good_angle_th_;

  // Distance increase speed of force resample(cm per tick).
  int resample_dist_increase_;

  // Max distance between resample particle and current position
  int max_resample_dist_;

  // Cycle since last force resample
  int since_last_resample_;

  int estimate_interval_;

  // ROS
  ros::NodeHandle* nh_;
  ros::Subscriber sub_mark_info_;
  ros::Publisher pub_amcl_info_;

  void onLandMarkCallback(const imb::MarkInfo::ConstPtr& msg);

  geometry_msgs::Vector3 control_;
  imb::MarkInfo measurement_;
};
