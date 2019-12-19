#pragma once
#include <QMainWindow>
#include <thread>


class MainWindow : public QMainWindow
{
    Q_OBJECT
  public:
    /**
     * @brief MainWindow constructor
     *
     * @param parent - parennt of QWidget
     */
    MainWindow(QWidget* parent = 0);
    /**
     * @brief MainWindow destructor
     */
    ~MainWindow();

    void startRos();

    void joinRos();

  private:
    std::thread ros_thread_;
};