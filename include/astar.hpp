#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <vector>
#include "imb/AMCLInfo.h"
#include "imb/AStarInfo.h"
#include "imb/MarkInfo.h"

const int BOUND_X = 450;
const int BOUND_Y = 300;

struct DIR {
  DIR(int dx, int dy, int dz, int cost = 1)
      : dx(dx), dy(dy), dz(dz), cost(cost) {}
  int dx, dy, dz;
  int cost;
};

struct STATUS {
  STATUS() {}
  STATUS(int x, int y, int z, int cost) : x(x), y(y), z(z), cost(cost) {}
  STATUS(geometry_msgs::Vector3 pos, int cost)
      : x(pos.x), y(pos.y), z(pos.z), cost(cost) {}
  int x, y, z;
  int cost, heuristic;
  STATUS* prev = NULL;
};

class AStar {
 public:
  AStar(ros::NodeHandle* nh);
  ~AStar();

  void step();

 private:
  ros::NodeHandle* nh_;
  ros::Subscriber sub_marks_;
  ros::Subscriber sub_amcl_;
  ros::Publisher pub_;

  void marksCallback(const imb::MarkInfo::ConstPtr& msg);
  void amclCallback(const imb::AMCLInfo::ConstPtr& msg);

  geometry_msgs::Vector3 robot_pos_;
  geometry_msgs::Vector3 target_;
  std::vector<geometry_msgs::Vector3> route_;

  // For AMCL
  // Moving steps
  std::vector<DIR> dirs_;
  // Heap for storing astar status
  std::vector<STATUS*> heap_;
  // Map
  int map_[100][100][40];

  int getHeuristic(const STATUS& a);
};
