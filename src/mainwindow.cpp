#include "mainwindow.hpp"
#include <ros/ros.h>
#include <QDockWidget>
#include <QGraphicsView>
#include <QLabel>
#include <QStatusBar>
#include <QTimer>
#include "item_manager.hpp"
#include "render.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  int w = 1040;
  int h = 740;
  resize(w, h);

  // Left dock
  //   QDockWidget* control_panel = new QDockWidget("Control");
  //   ControlWidget* c = new ControlWidget(control_panel);
  //   control_panel->setWidget(c);
  //   this->addDockWidget(Qt::LeftDockWidgetArea, control_panel);

  //   auto s = new Streamer();

  //   this->addDockWidget(Qt::LeftDockWidgetArea, s);

  // Central Area (soccer field)
  auto render = new Render(this);
  auto itemManager = new ItemManager(this, render);
  itemManager->Init();

  auto view = new QGraphicsView(this);
  view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  view->setScene(render);

  setCentralWidget(view);

  // Timer for updating view
  auto timer = new QTimer(this);
  connect(timer, &QTimer::timeout, [=]() {
    view->update();
    view->scene()->update();
    // s->update();
  });
  timer->start(1000 / 30.0);

  // startRos();
}

MainWindow::~MainWindow() { ros_thread_.join(); }

void MainWindow::startRos() {
  ros_thread_ = std::thread([&]() {
    ros::Rate r(100);
    while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
    }
  });
}

void MainWindow::joinRos() { ros_thread_.join(); }