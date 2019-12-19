#include <QApplication>
#include "ros/ros.h"
#include "mainwindow.hpp"

int main(int argc, char** argv) {
  QApplication app(argc, argv);

  ros::init(argc, argv, "imb");
  ros::NodeHandle nh("~");

  // dviz::MainWindow win;
  // win.show();
  MainWindow win;
  win.show();

  return app.exec();
}
