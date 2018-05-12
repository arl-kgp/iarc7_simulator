#include "include/ros_thread.h"

ROSThread::ROSThread(int argc, char** argv):
  m_argc(argc),
  m_argv(argv)
{

}

ROSThread::~ROSThread()
{
  if (ros::isStarted()) {
    ros::shutdown();
    ros::waitForShutdown();
  }
  m_pThread->quit();
}

bool ROSThread::init()
{
  m_pThread = new QThread();
  this->moveToThread(m_pThread);

  connect(m_pThread, &QThread::started, this, &ROSThread::run);
  ros::init(m_argc, m_argv, "iarc7_sim_node");

  if (!ros::master::check())
    return false;

  ros::start();
  ros::Time::init();
  ros::NodeHandle nh;
  sub_quad = nh.subscribe("/ground_truth/state", 10, &ROSThread::quadCallback, this);
  sub_gbot0 = nh.subscribe("/robot0/odom", 10, &ROSThread::gBot0Callback, this);
  sub_gbot1 = nh.subscribe("/robot1/odom", 10, &ROSThread::gBot1Callback, this);
  sub_gbot2 = nh.subscribe("/robot2/odom", 10, &ROSThread::gBot2Callback, this);
  sub_gbot3 = nh.subscribe("/robot3/odom", 10, &ROSThread::gBot3Callback, this);
  sub_gbot4 = nh.subscribe("/robot4/odom", 10, &ROSThread::gBot4Callback, this);
  sub_gbot5 = nh.subscribe("/robot5/odom", 10, &ROSThread::gBot5Callback, this);
  sub_gbot6 = nh.subscribe("/robot6/odom", 10, &ROSThread::gBot6Callback, this);
  sub_gbot7 = nh.subscribe("/robot7/odom", 10, &ROSThread::gBot7Callback, this);
  sub_gbot8 = nh.subscribe("/robot8/odom", 10, &ROSThread::gBot8Callback, this);
  sub_gbot9   = nh.subscribe("/robot9/odom", 10, &ROSThread::gBot9Callback, this);
 sub_gbot10 = nh.subscribe("/robot10/odom", 10, &ROSThread::gBot10Callback, this);
 sub_gbot11 = nh.subscribe("/robot11/odom", 10, &ROSThread::gBot11Callback, this);
 sub_gbot12 = nh.subscribe("/robot12/odom", 10, &ROSThread::gBot12Callback, this);
 sub_gbot13 = nh.subscribe("/robot13/odom", 10, &ROSThread::gBot13Callback, this);

 esti_gbot4 = nh.subscribe("/robot4/estimated", 10, &ROSThread::esti_gBot4Callback, this);
  esti_gbot5 = nh.subscribe("/robot5/estimated", 10, &ROSThread::esti_gBot5Callback, this);
  esti_gbot6 = nh.subscribe("/robot6/estimated", 10, &ROSThread::esti_gBot6Callback, this);
  esti_gbot7 = nh.subscribe("/robot7/estimated", 10, &ROSThread::esti_gBot7Callback, this);
  esti_gbot8 = nh.subscribe("/robot8/estimated", 10, &ROSThread::esti_gBot8Callback, this);
  esti_gbot9 = nh.subscribe("/robot9/estimated", 10, &ROSThread::esti_gBot9Callback, this);
 esti_gbot10 = nh.subscribe("/robot10/estimated", 10, &ROSThread::esti_gBot10Callback, this);
 esti_gbot11 = nh.subscribe("/robot11/estimated", 10, &ROSThread::esti_gBot11Callback, this);
 esti_gbot12 = nh.subscribe("/robot12/estimated", 10, &ROSThread::esti_gBot12Callback, this);
 esti_gbot13 = nh.subscribe("/robot13/estimated", 10, &ROSThread::esti_gBot13Callback, this);
  m_pThread->start();
  return true;
}

void ROSThread::quadCallback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  p.quat_w = msg->pose.pose.orientation.w;
  p.quat_x = msg->pose.pose.orientation.x;
  p.quat_y = msg->pose.pose.orientation.y;
  p.quat_z = msg->pose.pose.orientation.z;
  p.velocity.lin_x = msg->twist.twist.linear.x;
  p.velocity.lin_y = msg->twist.twist.linear.y;
  p.velocity.lin_z = msg->twist.twist.linear.z;
  p.velocity.ang_x = msg->twist.twist.angular.x;
  p.velocity.ang_y = msg->twist.twist.angular.y;
  p.velocity.ang_z = msg->twist.twist.angular.z;
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setQuadPose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}

void ROSThread::gBot0Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  p.quat_w = msg->pose.pose.orientation.w;
  p.quat_x = msg->pose.pose.orientation.x;
  p.quat_y = msg->pose.pose.orientation.y;
  p.quat_z = msg->pose.pose.orientation.z;
  p.velocity.lin_x = msg->twist.twist.linear.x;
  p.velocity.lin_y = msg->twist.twist.linear.y;
  p.velocity.lin_z = msg->twist.twist.linear.z;
  p.velocity.ang_x = msg->twist.twist.angular.x;
  p.velocity.ang_y = msg->twist.twist.angular.y;
  p.velocity.ang_z = msg->twist.twist.angular.z;
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot0Pose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}

void ROSThread::gBot1Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  p.quat_w = msg->pose.pose.orientation.w;
  p.quat_x = msg->pose.pose.orientation.x;
  p.quat_y = msg->pose.pose.orientation.y;
  p.quat_z = msg->pose.pose.orientation.z;
  p.velocity.lin_x = msg->twist.twist.linear.x;
  p.velocity.lin_y = msg->twist.twist.linear.y;
  p.velocity.lin_z = msg->twist.twist.linear.z;
  p.velocity.ang_x = msg->twist.twist.angular.x;
  p.velocity.ang_y = msg->twist.twist.angular.y;
  p.velocity.ang_z = msg->twist.twist.angular.z;
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot1Pose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}

void ROSThread::gBot2Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  p.quat_w = msg->pose.pose.orientation.w;
  p.quat_x = msg->pose.pose.orientation.x;
  p.quat_y = msg->pose.pose.orientation.y;
  p.quat_z = msg->pose.pose.orientation.z;
  p.velocity.lin_x = msg->twist.twist.linear.x;
  p.velocity.lin_y = msg->twist.twist.linear.y;
  p.velocity.lin_z = msg->twist.twist.linear.z;
  p.velocity.ang_x = msg->twist.twist.angular.x;
  p.velocity.ang_y = msg->twist.twist.angular.y;
  p.velocity.ang_z = msg->twist.twist.angular.z;
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot2Pose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}

void ROSThread::gBot3Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  p.quat_w = msg->pose.pose.orientation.w;
  p.quat_x = msg->pose.pose.orientation.x;
  p.quat_y = msg->pose.pose.orientation.y;
  p.quat_z = msg->pose.pose.orientation.z;
  p.velocity.lin_x = msg->twist.twist.linear.x;
  p.velocity.lin_y = msg->twist.twist.linear.y;
  p.velocity.lin_z = msg->twist.twist.linear.z;
  p.velocity.ang_x = msg->twist.twist.angular.x;
  p.velocity.ang_y = msg->twist.twist.angular.y;
  p.velocity.ang_z = msg->twist.twist.angular.z;
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot3Pose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}

void ROSThread::gBot4Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  p.quat_w = msg->pose.pose.orientation.w;
  p.quat_x = msg->pose.pose.orientation.x;
  p.quat_y = msg->pose.pose.orientation.y;
  p.quat_z = msg->pose.pose.orientation.z;
  p.velocity.lin_x = msg->twist.twist.linear.x;
  p.velocity.lin_y = msg->twist.twist.linear.y;
  p.velocity.lin_z = msg->twist.twist.linear.z;
  p.velocity.ang_x = msg->twist.twist.angular.x;
  p.velocity.ang_y = msg->twist.twist.angular.y;
  p.velocity.ang_z = msg->twist.twist.angular.z;
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot4Pose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}


void ROSThread::gBot5Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  p.quat_w = msg->pose.pose.orientation.w;
  p.quat_x = msg->pose.pose.orientation.x;
  p.quat_y = msg->pose.pose.orientation.y;
  p.quat_z = msg->pose.pose.orientation.z;
  p.velocity.lin_x = msg->twist.twist.linear.x;
  p.velocity.lin_y = msg->twist.twist.linear.y;
  p.velocity.lin_z = msg->twist.twist.linear.z;
  p.velocity.ang_x = msg->twist.twist.angular.x;
  p.velocity.ang_y = msg->twist.twist.angular.y;
  p.velocity.ang_z = msg->twist.twist.angular.z;
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot5Pose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}

void ROSThread::gBot6Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  p.quat_w = msg->pose.pose.orientation.w;
  p.quat_x = msg->pose.pose.orientation.x;
  p.quat_y = msg->pose.pose.orientation.y;
  p.quat_z = msg->pose.pose.orientation.z;
  p.velocity.lin_x = msg->twist.twist.linear.x;
  p.velocity.lin_y = msg->twist.twist.linear.y;
  p.velocity.lin_z = msg->twist.twist.linear.z;
  p.velocity.ang_x = msg->twist.twist.angular.x;
  p.velocity.ang_y = msg->twist.twist.angular.y;
  p.velocity.ang_z = msg->twist.twist.angular.z;
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot6Pose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}

void ROSThread::gBot7Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  p.quat_w = msg->pose.pose.orientation.w;
  p.quat_x = msg->pose.pose.orientation.x;
  p.quat_y = msg->pose.pose.orientation.y;
  p.quat_z = msg->pose.pose.orientation.z;
  p.velocity.lin_x = msg->twist.twist.linear.x;
  p.velocity.lin_y = msg->twist.twist.linear.y;
  p.velocity.lin_z = msg->twist.twist.linear.z;
  p.velocity.ang_x = msg->twist.twist.angular.x;
  p.velocity.ang_y = msg->twist.twist.angular.y;
  p.velocity.ang_z = msg->twist.twist.angular.z;
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot7Pose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}

void ROSThread::gBot8Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  p.quat_w = msg->pose.pose.orientation.w;
  p.quat_x = msg->pose.pose.orientation.x;
  p.quat_y = msg->pose.pose.orientation.y;
  p.quat_z = msg->pose.pose.orientation.z;
  p.velocity.lin_x = msg->twist.twist.linear.x;
  p.velocity.lin_y = msg->twist.twist.linear.y;
  p.velocity.lin_z = msg->twist.twist.linear.z;
  p.velocity.ang_x = msg->twist.twist.angular.x;
  p.velocity.ang_y = msg->twist.twist.angular.y;
  p.velocity.ang_z = msg->twist.twist.angular.z;
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot8Pose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}

void ROSThread::gBot9Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  p.quat_w = msg->pose.pose.orientation.w;
  p.quat_x = msg->pose.pose.orientation.x;
  p.quat_y = msg->pose.pose.orientation.y;
  p.quat_z = msg->pose.pose.orientation.z;
  p.velocity.lin_x = msg->twist.twist.linear.x;
  p.velocity.lin_y = msg->twist.twist.linear.y;
  p.velocity.lin_z = msg->twist.twist.linear.z;
  p.velocity.ang_x = msg->twist.twist.angular.x;
  p.velocity.ang_y = msg->twist.twist.angular.y;
  p.velocity.ang_z = msg->twist.twist.angular.z;
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot9Pose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}

void ROSThread::gBot10Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  p.quat_w = msg->pose.pose.orientation.w;
  p.quat_x = msg->pose.pose.orientation.x;
  p.quat_y = msg->pose.pose.orientation.y;
  p.quat_z = msg->pose.pose.orientation.z;
  p.velocity.lin_x = msg->twist.twist.linear.x;
  p.velocity.lin_y = msg->twist.twist.linear.y;
  p.velocity.lin_z = msg->twist.twist.linear.z;
  p.velocity.ang_x = msg->twist.twist.angular.x;
  p.velocity.ang_y = msg->twist.twist.angular.y;
  p.velocity.ang_z = msg->twist.twist.angular.z;
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot10Pose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}

void ROSThread::gBot11Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  p.quat_w = msg->pose.pose.orientation.w;
  p.quat_x = msg->pose.pose.orientation.x;
  p.quat_y = msg->pose.pose.orientation.y;
  p.quat_z = msg->pose.pose.orientation.z;
  p.velocity.lin_x = msg->twist.twist.linear.x;
  p.velocity.lin_y = msg->twist.twist.linear.y;
  p.velocity.lin_z = msg->twist.twist.linear.z;
  p.velocity.ang_x = msg->twist.twist.angular.x;
  p.velocity.ang_y = msg->twist.twist.angular.y;
  p.velocity.ang_z = msg->twist.twist.angular.z;
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot11Pose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}


void ROSThread::gBot12Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  p.quat_w = msg->pose.pose.orientation.w;
  p.quat_x = msg->pose.pose.orientation.x;
  p.quat_y = msg->pose.pose.orientation.y;
  p.quat_z = msg->pose.pose.orientation.z;
  p.velocity.lin_x = msg->twist.twist.linear.x;
  p.velocity.lin_y = msg->twist.twist.linear.y;
  p.velocity.lin_z = msg->twist.twist.linear.z;
  p.velocity.ang_x = msg->twist.twist.angular.x;
  p.velocity.ang_y = msg->twist.twist.angular.y;
  p.velocity.ang_z = msg->twist.twist.angular.z;
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot12Pose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}

void ROSThread::gBot13Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  p.z = msg->pose.pose.position.z;
  p.quat_w = msg->pose.pose.orientation.w;
  p.quat_x = msg->pose.pose.orientation.x;
  p.quat_y = msg->pose.pose.orientation.y;
  p.quat_z = msg->pose.pose.orientation.z;
  p.velocity.lin_x = msg->twist.twist.linear.x;
  p.velocity.lin_y = msg->twist.twist.linear.y;
  p.velocity.lin_z = msg->twist.twist.linear.z;
  p.velocity.ang_x = msg->twist.twist.angular.x;
  p.velocity.ang_y = msg->twist.twist.angular.y;
  p.velocity.ang_z = msg->twist.twist.angular.z;
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot13Pose(p);   
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}

void ROSThread::esti_gBot4Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  Pose p1;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
  
 
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot4EstiPose(p);
  
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
 
}
void ROSThread::esti_gBot5Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
 
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot5EstiPose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}
void ROSThread::esti_gBot6Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
 
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot6EstiPose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}
void ROSThread::esti_gBot7Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
 
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot7EstiPose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}
void ROSThread::esti_gBot8Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
 
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot8EstiPose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}
void ROSThread::esti_gBot9Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
 
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot9EstiPose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}
void ROSThread::esti_gBot10Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
 
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot10EstiPose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}
void ROSThread::esti_gBot11Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
 
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot11EstiPose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}
void ROSThread::esti_gBot12Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
 
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot12EstiPose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}
void ROSThread::esti_gBot13Callback(const nav_msgs::OdometryConstPtr& msg)
{
  QMutex * pMutex = new QMutex();

  pMutex->lock();
  Pose p;
  p.x = msg->pose.pose.position.x;
  p.y = msg->pose.pose.position.y;
 
  for (int i = 0; i < 36; i++)
    p.covariance[i] = (double)msg->pose.covariance[i];
  common->setgBot13EstiPose(p);
  pMutex->unlock();

  delete pMutex;
  Q_EMIT newPose();
}


void ROSThread::run()
{
  ros::Rate loop_rate(10);
  QMutex * pMutex;
  while (ros::ok()) {
    pMutex = new QMutex();

    pMutex->lock();

    pMutex->unlock();

    ros::spinOnce();
    loop_rate.sleep();
    delete pMutex;
  }
}
