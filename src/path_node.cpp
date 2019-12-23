#include <signal.h>
#include "astar.hpp"

void signalHandler(int sig) {
  ROS_WARN("trying to exit!");
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "astar");
  ros::NodeHandle nh("~");
  signal(SIGINT, signalHandler);

  AStar path(&nh);

  ros::Rate r(3);
  while (ros::ok()) {
      // Calc path
      path.step();
      ros::spinOnce();
      r.sleep();
  }
  return 0;
}