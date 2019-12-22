#include "astar.hpp"
#include <algorithm>

template <typename T>
float DegreeToRadian(T angle) {
  return angle / 180.f * M_PI;
}

AStar::AStar(ros::NodeHandle* nh) : nh_(nh) {
  robot_pos_.x = 0;
  robot_pos_.y = 0;
  robot_pos_.z = 0;
  target_.x = 20;
  target_.y = 10;
  target_.y = 0;

  dirs_.push_back(DIR(10, 0, 0, 10));
  dirs_.push_back(DIR(-10, 0, 0, 10));
  dirs_.push_back(DIR(0, 10, 0, 10));
  dirs_.push_back(DIR(0, -10, 0, 10));
  dirs_.push_back(DIR(0, 0, 10, 5));
  dirs_.push_back(DIR(0, 0, -10, 5));

  sub_marks_ = nh_->subscribe<imb::MarkInfo>("/LandMark", 1,
                                             &AStar::marksCallback, this);
  sub_amcl_ =
      nh_->subscribe<imb::AMCLInfo>("/AMCL", 1, &AStar::amclCallback, this);
  pub_ = nh_->advertise<imb::AStarInfo>("/AStar", 1);
}

bool cmp(const STATUS* a, const STATUS* b) {
  return a->heuristic > a->heuristic;
}

AStar::~AStar() {}

void AStar::step() {
  // Calc path from robot_pos to target
  // Init
  route_.clear();
  heap_.clear();
  STATUS* start = new STATUS(robot_pos_, 0);
  start->prev = NULL;
  start->heuristic = getHeuristic(*start);
  heap_.push_back(start);
  STATUS* goal = NULL;
  // Start Search
  while (!heap_.empty()) {
    std::pop_heap(heap_.begin(), heap_.end(), &cmp);
    STATUS* curr = heap_.back();
    heap_.pop_back();
    for (auto d : dirs_) {
      STATUS* next = new STATUS();
      float rad_z = DegreeToRadian(float(curr->z));
      int dx = d.dx * std::cos(rad_z) + d.dy * std::sin(rad_z);
      int dy = d.dx * std::sin(rad_z) + d.dy * std::cos(rad_z);
      if (curr->x + dx < -BOUND_X || curr->x + dx > BOUND_X) continue;
      if (curr->y + dy < -BOUND_Y || curr->y + dy > BOUND_Y) continue;
      next->x = curr->x + dx;
      next->y = curr->y + dy;
      next->z = curr->z + d.dz;
      next->prev = curr;
      next->cost = curr->cost + d.cost;
      next->heuristic = getHeuristic(*next);
      if (next->z > 360) next->z -= 360;
      if (next->z < 0) next->z += 360;
      if (abs(target_.x - next->x) < 10 && abs(target_.y - next->y) < 10 &&
          abs(target_.z - next->z) < 10) {
        goal = next;
        break;
      }
      heap_.push_back(next);
      std::push_heap(heap_.begin(), heap_.end(), &cmp);
    }
    if (goal != NULL) break;
  }
  STATUS* p = goal;
  while (p != NULL) {
    geometry_msgs::Vector3 tmp;
    tmp.x = p->x;
    tmp.y = p->y;
    tmp.z = p->z;
    route_.push_back(tmp);
    p = p->prev;
  }

  imb::AStarInfo msg;
  msg.route = route_;
  pub_.publish(msg);
}

void AStar::marksCallback(const imb::MarkInfo::ConstPtr& msg) {
  target_ = msg->target;
}
void AStar::amclCallback(const imb::AMCLInfo::ConstPtr& msg) {
  robot_pos_ = msg->robot_pos;
}
int AStar::getHeuristic(const STATUS& a) {
  int heuristic =
      abs(target_.x - a.x) + abs(target_.y - a.y) + abs(target_.z - a.z) / 2;
  return heuristic;
}