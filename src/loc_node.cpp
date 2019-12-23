
#include <signal.h>
#include "amcl.hpp"

void signalHandler(int sig) {
  ROS_WARN("trying to exit!");
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "astar");
  ros::NodeHandle nh("~");
  signal(SIGINT, signalHandler);

  AMCL amcl(&nh);

  ros::Rate r(10);
  while (ros::ok()) {
      // Calc location
      amcl.step();
      ros::spinOnce();
      r.sleep();
  }
  return 0;
}