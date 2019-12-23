
#include "amcl.hpp"
#include "imb/AMCLInfo.h"
#include "utils.hpp"

using namespace std;

AMCL::AMCL(ros::NodeHandle *nh)
    : current_set_(0),
      converged_(false),
      // don't change this!!!
      y_gauss_(0, .3),
      x_gauss_(0, .5),
      resample_gauss_(0, 5),
      uniform_(-1, 1),
      quality_(0),
      since_last_resample_(0),
      nh_(nh) {
  //! Initialize parameters
  num_particles_ = 300;

  dist_threshold_ = 50;
  z_hit_ = 0.95;
  z_rand_ = 0.05;
  sigma_hit_ = 5;
  good_dist_th_ = 20;
  good_angle_th_ = 5;
  resample_interval_ = 3 * 10;
  estimate_interval_ = 2;
  resample_dist_increase_ = 10;
  max_resample_dist_ = 900;

  //! Initialize sample sets
  for (int i = 0; i < 2; ++i) {
    auto &set = sets_[i];

    set.samples.resize(num_particles_);
    set.kdtree = new KdTree(3 * num_particles_);
    set.cluster_max_count = num_particles_;
    set.clusters.resize((size_t)set.cluster_max_count);

    set.mean = Pose(0, 0, 0);
    set.cov = Matrix();
  }

  //! Initialize current set
  auto &set = sets_[current_set_];
  set.kdtree->Clear();
  for (auto &sample : set.samples) {
    sample.weight = 1.0 / set.samples.size();
    sample.pose = RandomPose();
    sample.pose.heading = 90;
    set.kdtree->InsertPose(sample.pose, sample.weight);
  }
  ClusterStats(set);

  sub_mark_info_ = nh_->subscribe<imb::MarkInfo>(
      "/LandMark", 1, &AMCL::onLandMarkCallback, this);
  sub_motion_info_ = nh_->subscribe<geometry_msgs::Vector3>(
      "/MotionDelta", 1, &AMCL::onMotionCallback, this);
  pub_amcl_info_ = nh_->advertise<imb::AMCLInfo>("/AMCL", 1);
}

void AMCL::step() {
  //! Resample

  if ((++cycle_ > resample_interval_) || measurement_.see_circle) {
    Resample();
    cycle_ = 0;
  }
  SampleMotionModel();
  MeasurementModel();

  //! Estimate
  if (!(cycle_ % estimate_interval_)) {
    Estimate();
    CheckConverged();
  }

  imb::AMCLInfo msg;

  //! Put result into visionInfo
  auto &particles = this->particles();
  msg.particles.resize(particles.size());
  int nGoodParticles = 0;

  for (uint32_t i = 0; i < particles.size(); ++i) {
    msg.particles[i].pose.x = particles[i].pose.x;
    msg.particles[i].pose.y = particles[i].pose.y;
    msg.particles[i].pose.z = particles[i].pose.heading;
    msg.particles[i].weight = particles[i].weight;
    double dist = GetDistance(Pose(particles[i].pose.x, particles[i].pose.y, 0),
                              Pose(pose_.x, pose_.y, 0));
    double d_angle = std::fabs(particles[i].pose.heading - pose_.heading);
    // count good particles
    if (dist < good_dist_th_ && d_angle < good_angle_th_) nGoodParticles++;
  }

  msg.robot_pos.x = pose_.x;
  msg.robot_pos.y = pose_.y;
  msg.robot_pos.z = pose_.heading;
  quality_ = nGoodParticles / (double)particles.size();

  pub_amcl_info_.publish(msg);
}

void AMCL::SampleMotionModel() {
  double dx = control_.x;
  double dy = control_.y;
  control_.x = 0;
  control_.y = 0;

  double w = 450 + 70;
  double h = 300 + 70;

  for (auto &particle : this->particles()) {
    Pose &p = particle.pose;
    double ddx = dx + x_gauss_.sample() * 10;
    double ddy = dy + y_gauss_.sample() * 10;

    double heading = p.heading / 180.0 * M_PI;
    p.x = p.x + ddx;
    p.y = p.y + ddy;

    p.x = std::min(p.x, w);
    p.x = std::max(p.x, -w);
    p.y = std::min(p.y, h);
    p.y = std::max(p.y, -h);
  }
}

void AMCL::MeasurementModel() {
  std::vector<std::pair<int, int>> Lcorner = {
      {350, 250}, {350, -250}, {-350, 250}, {-350, -250},
      {450, 300}, {450, -300}, {-450, 300}, {-450, -300}};
  std::vector<std::pair<int, int>> Tcorner = {
      {0, 300}, {0, -300}, {450, 250}, {450, -250}, {-450, 250}, {-450, -250}};
  std::vector<std::pair<int, int>> goal_posts = {
      {450, 130}, {450, -130}, {-450, 130}, {-450, -130}};

  double total_weight = 0;
  auto &set = sets_[current_set_];
  for (auto &sample : set.samples) {
    // double prob = sample.weight;
    double prob = 0;
    Pose gParticle(sample.pose.x, sample.pose.y, sample.pose.heading);
    if (measurement_.see_circle) {
      Pose gCenter = getOnGlobalPosition(
          gParticle, Pose(measurement_.circle.x, measurement_.circle.y, 0));
      auto dist = sqrt(gCenter.x * gCenter.x + gCenter.y * gCenter.y) / 5;
      // dist = std::min(dist, 50.0);
      double pz = normal_pdf(dist, 0.0, sigma_hit_);
      prob += pz * 100;
    }

    for (auto &corner : measurement_.cornerL) {
      Pose gCenter =
          getOnGlobalPosition(gParticle, Pose(corner.x, corner.y, 0));
      // Get dist to closest corner
      double dist = 100.0;
      for (auto c : Lcorner) {
        auto dx = c.first - gCenter.x;
        auto dy = c.second - gCenter.y;
        dist = std::min(sqrt(dx * dx + dy * dy) / 5, dist);
      }
      // double pz = normal_pdf(dist, 0.0, sigma_hit_);
      double pz = 1 / dist;
      prob += pz * 30;
    }

    for (auto &corner : measurement_.cornerT) {
      Pose gCenter =
          getOnGlobalPosition(gParticle, Pose(corner.x, corner.y, 0));
      // Get dist to closest corner
      double dist = 100.0;
      for (auto c : Tcorner) {
        auto dx = c.first - gCenter.x;
        auto dy = c.second - gCenter.y;
        dist = std::min(sqrt(dx * dx + dy * dy) / 5, dist);
      }
      // double pz = normal_pdf(dist, 0.0, sigma_hit_);
      double pz = 1 / dist;
      prob += pz * 50;
    }

    for (auto &goal : measurement_.goal_posts) {
      Pose gCenter = getOnGlobalPosition(gParticle, Pose(goal.x, goal.y, 0));
      // Get dist to closest goal
      double dist = 100.0;
      for (auto g : goal_posts) {
        auto dx = g.first - gCenter.x;
        auto dy = g.second - gCenter.y;
        dist = std::min(sqrt(dx * dx + dy * dy) / 5, dist);
      }
      // double pz = normal_pdf(dist, 0.0, sigma_hit_);
      double pz = 1 / dist;
      prob += pz * 70;
    }

    sample.weight = prob;
    total_weight += sample.weight;
  }

  if (total_weight > 0) {
    for (auto &sample : set.samples) {
      sample.weight /= total_weight;
    }

    double w_avg = total_weight / set.samples.size();
    if (w_slow_ == 0.0) {
      w_slow_ = w_avg;
    } else {
      w_slow_ += alpha_slow_ * (w_avg - w_slow_);
    }

    if (w_fast_ == 0.0) {
      w_fast_ = w_avg;
    } else {
      w_fast_ += alpha_fast_ * (w_avg - w_fast_);
    }
  } else {
    for (auto &sample : set.samples) sample.weight = 1.0 / set.samples.size();
  }
}

void AMCL::Resample() {
  auto &set_a = sets_[current_set_];
  auto &set_b = sets_[(current_set_ + 1) % 2];
  std::random_shuffle(set_b.samples.begin(), set_b.samples.end());
  double b_nums = set_b.samples.size();
  double max_weight = 0;

  double c[set_a.samples.size() + 1];
  c[0] = 0.0;
  for (uint32_t i = 0; i < set_a.samples.size(); ++i) {
    c[i + 1] = c[i] + set_a.samples[i].weight;
    if (set_a.samples[i].weight > max_weight)
      max_weight = set_a.samples[i].weight;
  }

  since_last_resample_++;
  size_t cnt = 0;
  if (measurement_.see_circle) {
    // calc pose with max likelihood observing the center
    // and put some samples thered
    size_t reset_for_center_size = set_b.samples.size() / 3;
    auto &center = measurement_.circle;
    double x = center.x;
    double y = center.y;
    double r = pose_.headingR();

    auto dx = x * cos(r) - y * sin(r);
    auto dy = x * sin(r) + y * cos(r);

    auto xx = 0.0 - dx;
    auto yy = 0.0 - dy;
    int resample_dist = std::min(since_last_resample_ * resample_dist_increase_,
                                 max_resample_dist_);
    since_last_resample_ = 0;
    for (; cnt < reset_for_center_size; ++cnt) {
      set_b.samples[cnt].pose =
          Pose(xx + resample_gauss_.sample() * 2,
               yy + resample_gauss_.sample() * 2, pose_.heading);
      set_b.samples[cnt].weight = 1.0 / b_nums;
    }
    ROS_DEBUG("Resample when see circle!");
  }

  Pose high_p = pose_;
  double x = high_p.x;
  double y = high_p.y;

  if (!measurement_.see_circle) {
    // P number of the new particles are drawn around the old particles with
    // the highest particle weight.
    size_t reset_for_heavy_small = max_weight * set_b.samples.size();

    for (; cnt < reset_for_heavy_small; ++cnt) {
      set_b.samples[cnt].pose =
          Pose(x + resample_gauss_.sample(), y + resample_gauss_.sample(),
               high_p.heading);
      set_b.samples[cnt].weight = 1.0 / b_nums;
    }
    for (; cnt < set_b.samples.size(); ++cnt) {
      auto &sample_b = set_b.samples[cnt];

      sample_b.pose = Pose(x + 5 * resample_gauss_.sample(),
                           y + 5 * resample_gauss_.sample(), high_p.heading);
      sample_b.weight = 1.0 / b_nums;
    }
    ROS_DEBUG("Custom Resample");
  } else {
    for (; cnt < set_b.samples.size(); ++cnt) {
      auto &sample_b = set_b.samples[cnt];

      set_b.samples[cnt].pose =
          Pose(x + 3 * resample_gauss_.sample(),
               y + 3 * resample_gauss_.sample(), high_p.heading);
      sample_b.weight = 1.0 / b_nums;
    }
  }

  double w_diff = 1.0 - w_fast_ / w_slow_;
  w_diff = max(0.0, w_diff);

  for (unsigned int cnt = 0; cnt < set_b.samples.size(); ++cnt) {
    auto &sample_b = set_b.samples[cnt];

    if (drand48() < w_diff) {
      for (uint32_t i = 0; i < set_a.samples.size(); ++i) {
        sample_b = set_a.samples[i];
        sample_b.pose.x = sample_b.pose.x + resample_gauss_.sample();
        sample_b.pose.y = sample_b.pose.y + resample_gauss_.sample();
      }
    }
    sample_b.weight = 1.0 / b_nums;
  }

  current_set_ = (current_set_ + 1) % 2;

  CheckConverged();
}

void AMCL::Estimate() {
  // Create the kd tree for adaptive sampling
  auto &set_b = sets_[current_set_];
  set_b.kdtree->Clear();

  for (auto &sample : set_b.samples) {
    set_b.kdtree->InsertPose(sample.pose, sample.weight);
  }

  // Re-compute cluster statistics
  ClusterStats(set_b);

  auto &set = sets_[current_set_];
  double max_weight = 0.0;
  for (auto &cluster : set.clusters) {
    if (cluster.weight > max_weight) {
      max_weight = cluster.weight;
      pose_ = cluster.mean;
    }
  }
}

bool AMCL::CheckConverged() {
  auto &set = sets_[current_set_];

  double mean_x = 0, mean_y = 0;
  for (auto &sample : set.samples) {
    mean_x += sample.pose.x;
    mean_y += sample.pose.y;
  }
  mean_x /= set.samples.size();
  mean_y /= set.samples.size();

  for (auto &sample : set.samples) {
    if (fabs(sample.pose.x - mean_x) > this->dist_threshold_ ||
        fabs(sample.pose.y - mean_y) > this->dist_threshold_) {
      set.converged = false;
      converged_ = false;
      return false;
    }
  }
  converged_ = true;
  set.converged = true;
  return true;
}

void AMCL::ClusterStats(SampleSet &set) {
  //! Initialize workspace
  double m[4], c[2][2];
  set.kdtree->Cluster();

  //! Initialize cluster stats
  set.cluster_count = 0;

  for (int i = 0; i < set.cluster_max_count; ++i) {
    Cluster &cluster = set.clusters[i];
    cluster.count = 0;
    cluster.weight = 0;
    cluster.mean = Pose();
    cluster.cov = Matrix();

    for (int j = 0; j < 4; ++j) cluster.m[j] = 0.0;

    for (int j = 0; j < 2; ++j)
      for (int k = 0; k < 2; ++k) cluster.c[j][k] = 0.0;
  }

  //! Initialize overall filter stats
  size_t count = 0;
  double weight = 0.0;
  set.mean = Pose();
  set.cov = Matrix();

  for (int j = 0; j < 4; ++j) m[j] = 0.0;

  for (int j = 0; j < 2; ++j)
    for (int k = 0; k < 2; ++k) c[j][k] = 0.0;

  //! Compute cluster stats
  for (auto &sample : set.samples) {
    // Get the cluster label for this sample
    int cidx = set.kdtree->GetCluster(sample.pose);
    // assert(cidx >= 0);

    if (cidx >= set.cluster_max_count) continue;
    if (cidx + 1 > set.cluster_count) set.cluster_count = cidx + 1;

    auto &cluster = set.clusters[cidx];

    cluster.count += 1;
    cluster.weight += sample.weight;

    count += 1;
    weight += sample.weight;

    // Compute mean
    cluster.m[0] += sample.weight * sample.pose.x;
    cluster.m[1] += sample.weight * sample.pose.y;
    cluster.m[2] += sample.weight * std::cos(sample.pose.headingR());
    cluster.m[3] += sample.weight * std::sin(sample.pose.headingR());

    m[0] += sample.weight * sample.pose.x;
    m[1] += sample.weight * sample.pose.y;
    m[2] += sample.weight * std::cos(sample.pose.headingR());
    m[3] += sample.weight * std::sin(sample.pose.headingR());

    // Compute covariance in linear components
    for (int j = 0; j < 2; ++j)
      for (int k = 0; k < 2; ++k) {
        cluster.c[j][k] += sample.weight * sample.pose[j] * sample.pose[k];
        c[j][k] += sample.weight * sample.pose[j] * sample.pose[k];
      }
  }

  //! Normalize
  for (int i = 0; i < set.cluster_count; ++i) {
    auto &cluster = set.clusters[i];
    cluster.mean.x = cluster.m[0] / cluster.weight;
    cluster.mean.y = cluster.m[1] / cluster.weight;
    cluster.mean.setHeadingR(atan2(cluster.m[3], cluster.m[2]));

    cluster.cov = Matrix();

    // Covariance in linear components
    for (int j = 0; j < 2; ++j)
      for (int k = 0; k < 2; ++k) {
        cluster.cov.m[j][k] = cluster.c[j][k] / cluster.weight -
                              cluster.mean[j] * cluster.mean[k];
      }
    // Covariance in angular components;
    cluster.cov.m[2][2] =
        -2 *
        log(sqrt(cluster.m[2] * cluster.m[2] + cluster.m[3] * cluster.m[3]));
    // Cluster mean weight
    cluster.meanWeight = cluster.weight / cluster.count;
  }

  //! Compute overall filter stats
  set.mean[0] = m[0] / weight;
  set.mean[1] = m[1] / weight;
  set.mean.setHeadingR(atan2(m[3], m[2]));

  //! Covariance in linear components
  for (int j = 0; j < 2; ++j) {
    for (int k = 0; k < 2; ++k) {
      set.cov.m[j][k] = c[j][k] / weight - set.mean[j] * set.mean[k];
    }
  }

  set.cov.m[2][2] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));
}

Pose AMCL::RandomPose(const bool &use_yaw) {
  double w = 450;
  double h = 300;

  Pose p;
  p.x = uniform_.sample() * w;
  p.y = uniform_.sample() * h;
  if (use_yaw) {
    p.heading = yaw_;
    ;
  } else {
    p.heading = uniform_.sample() * 180;
  }
  return p;
}

void AMCL::falldownGauss() {
  Gaussian g(0, 10);
  for (auto &sample : sets_[current_set_].samples) {
    auto x = sample.pose.x;
    auto y = sample.pose.y;

    sample.pose.x = x + g.sample();
    sample.pose.y = y + g.sample();
  }
}

void AMCL::onLandMarkCallback(const imb::MarkInfo::ConstPtr &msg) {
  measurement_ = *msg;
}

void AMCL::onMotionCallback(const geometry_msgs::Vector3::ConstPtr &msg) {
  control_.x += msg->x;
  control_.y += msg->y;
  control_.z = msg->z;
  pose_.heading = msg->z;
  for (int i = 0; i < 2; ++i) {
    auto &set = sets_[i];
    for (auto &p : set.samples)
      p.pose.heading = msg->z;

    set.mean.heading = msg->z;
  }
}