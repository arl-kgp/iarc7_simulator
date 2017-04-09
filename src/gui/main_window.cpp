#include "main_window.h"
#include "ui_main_window.h"

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
  QMainWindow(parent),
  ros_thread(argc, argv),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  world = ui->world;
  common = new Common;
  ros_thread.common = common;
  world->common = common;

  connect(&ros_thread, &ROSThread::newPose, world, &World::updateGBotPose);
  ros_thread.init();
}

MainWindow::~MainWindow()
{
  delete common;
  delete ui;
}
