#include "astar.hpp"
#include <algorithm>
#include "types.hpp"
#include "utils.hpp"

AStar::AStar(ros::NodeHandle* nh) : nh_(nh) {
  robot_pos_.x = 0;
  robot_pos_.y = 0;
  robot_pos_.z = 0;
  target_.x = 300;
  target_.y = 210;
  target_.y = 90;

  dirs_.push_back(DIR(10, 0, 0, 5));
  dirs_.push_back(DIR(-8, 0, 0, 25));
  dirs_.push_back(DIR(0, 8, 0, 30));
  dirs_.push_back(DIR(0, -8, 0, 30));
  dirs_.push_back(DIR(0, 0, 10, 2));
  dirs_.push_back(DIR(0, 0, -10, 2));

  sub_marks_ = nh_->subscribe<imb::MarkInfo>("/LandMark", 1,
                                             &AStar::marksCallback, this);
  sub_amcl_ =
      nh_->subscribe<imb::AMCLInfo>("/AMCL", 1, &AStar::amclCallback, this);
  pub_ = nh_->advertise<imb::AStarInfo>("/AStar", 1);
}

bool cmp(const STATUS* a, const STATUS* b) {
  return a->heuristic >= b->heuristic;
}

AStar::~AStar() {}

void AStar::step() {
  imb::AStarInfo msg;
  // Calc path from robot_pos to target
  if (abs(target_.x - robot_pos_.x) < 30 &&
      abs(target_.y - robot_pos_.y) < 20 &&
      abs(target_.z - robot_pos_.z) < 30) {
    std::vector<geometry_msgs::Vector3> route;
    msg.route = route;
    pub_.publish(msg);
    return;
  }
  // Init
  for (int i = 0; i < 100; i++)
    for (int j = 0; j < 100; j++)
      for (int k = 0; k < 100; k++) map_[i][j][k] = -1;
  route_.clear();
  heap_.clear();
  STATUS* start = new STATUS(robot_pos_, 0);
  start->prev = NULL;
  start->heuristic = getHeuristic(*start);
  heap_.push_back(start);
  STATUS* goal = NULL;
  // Start Search
  int step = 0;
  while (step < 900 && !heap_.empty()) {
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
      next->heuristic = next->cost + getHeuristic(*next);
      if (next->z > 180) next->z -= 360;
      if (next->z < -180) next->z += 360;
      if (abs(target_.x - next->x) < 30 && abs(target_.y - next->y) < 20 &&
          abs(target_.z - next->z) < 30) {
        goal = next;
        break;
      }
      int mx = (next->x + 450) / 10;
      int my = (next->y + 300) / 10;
      int mz = (next->z + 180) / 10;
      if (map_[mx][my][mz] < 0 || map_[mx][my][mz] < next->heuristic) {
        map_[mx][my][mz] = next->heuristic;
        heap_.push_back(next);
        std::push_heap(heap_.begin(), heap_.end(), &cmp);
      }
      // for (auto i : heap_) std::cout << i->heuristic << " ";
      // std::cout << std::endl;
    }
    if (goal != NULL) break;
  }
  STATUS* p = goal;
  while (p != NULL) {
    geometry_msgs::Vector3 tmp;
    tmp.x = p->x;
    tmp.y = p->y;
    tmp.z = p->z;
    route_.insert(route_.begin(), tmp);
    p = p->prev;
  }

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
  Pose delta_field = getFieldPosition(Pose(a.x, a.y, a.z),
                              Pose(target_.x - a.x, target_.y - a.y, 0));
  int heuristic = abs(delta_field.x) + abs(delta_field.y) * 3 +
                  abs(target_.z - a.z) * 6;
  return heuristic;
}