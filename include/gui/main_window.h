#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include "world.h"
#include "ros_thread.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  Common *common;

  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();
public slots:
private slots:

private:
  Ui::MainWindow *ui;
  World *world;
  ROSThread ros_thread;
};

#endif // MAIN_WINDOW_H
