#ifndef ROS_THREAD_H
#define ROS_THREAD_H

#include <QtCore>
#include <QThread>
#include <stdlib.h>
#include <QMutex>
#include <iostream>
#include "common.h"
#include "assert.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

class ROSThread : public QObject
{
  Q_OBJECT
public:
  Common *common;

  ROSThread(int argc, char** argv);
  virtual ~ROSThread();
  bool init();
  void quadCallback(const nav_msgs::OdometryConstPtr& msg);
  void gBot0Callback(const nav_msgs::OdometryConstPtr& msg);
  void gBot1Callback(const nav_msgs::OdometryConstPtr& msg);
  void gBot2Callback(const nav_msgs::OdometryConstPtr& msg);
  void gBot3Callback(const nav_msgs::OdometryConstPtr& msg);
  void gBot4Callback(const nav_msgs::OdometryConstPtr& msg);
  void gBot5Callback(const nav_msgs::OdometryConstPtr& msg);
  void gBot6Callback(const nav_msgs::OdometryConstPtr& msg);
  void gBot7Callback(const nav_msgs::OdometryConstPtr& msg);
  void gBot8Callback(const nav_msgs::OdometryConstPtr& msg);
  void gBot9Callback(const nav_msgs::OdometryConstPtr& msg);
void gBot10Callback(const nav_msgs::OdometryConstPtr& msg);
void gBot11Callback(const nav_msgs::OdometryConstPtr& msg);
void gBot12Callback(const nav_msgs::OdometryConstPtr& msg);
void gBot13Callback(const nav_msgs::OdometryConstPtr& msg);
private:
  int m_argc;
  char** m_argv;
  QThread *m_pThread;
  ros::Subscriber sub_quad, sub_gbot0, sub_gbot1, sub_gbot2, sub_gbot3, sub_gbot4, sub_gbot5, sub_gbot6, sub_gbot7, sub_gbot8, sub_gbot9, sub_gbot10, sub_gbot11, sub_gbot12, sub_gbot13;
signals:
  void newPose();
public slots:
  void run();
};

#endif // ROS_THREAD_H
