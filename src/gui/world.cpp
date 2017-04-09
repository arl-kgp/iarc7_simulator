#include "world.h"

World::World(QWidget *parent) : QWidget(parent)
{

}

void World::paintField(QPainter &painter)
{
  QRect field;
  field.setTopLeft(QPoint(this->x(), this->y()));
  field.setBottomRight(QPoint(this->width()-1, this->height()-1));

  painter.save();
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::black);
  painter.setBrush(QBrush(Qt::blue));
  painter.drawRect(field);
  painter.restore();
}

QPointF World::realToSimCoord(QPointF coord)
{
  double x_offset = (coord.y() * SIM_WORLD_WIDTH / REAL_WORLD_WIDTH);
  double y_offset = (coord.x() * SIM_WORLD_HEIGHT / REAL_WORLD_HEIGHT);
  return QPointF(SIM_WORLD_WIDTH / 2 - x_offset, SIM_WORLD_HEIGHT / 2 - y_offset);
}

vector< double > World::ellipseFromCovMatrix(double *covariance)
{
  double chisquare_val = 3.03;
  Mat covmat = (cv::Mat_<double>(2,2) << covariance[0], covariance[1], covariance[6], covariance[7]);
  //Mat covmat = (cv::Mat_<double>(2,2) << .1, .05, .05, .1);
  Mat eigenvalues, eigenvectors;
  cv::eigen(covmat, true, eigenvalues, eigenvectors);
  //Calculate the angle between the largest eigenvector and the x-axis
  double angle = atan2(eigenvectors.at<double>(0,1), eigenvectors.at<double>(0,0));

  //Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
  if(angle < 0)
    angle += 6.28318530718;

  //Conver to degrees instead of radians
  angle = 180 * angle / 3.14159265359;

  //Calculate the size of the minor and major axes
  double halfmajoraxissize = chisquare_val * sqrt(eigenvalues.at<double>(0));
  double halfminoraxissize = chisquare_val * sqrt(eigenvalues.at<double>(1));

  vector< double > params;
  params.push_back(halfmajoraxissize * SIM_WORLD_WIDTH / REAL_WORLD_WIDTH);
  params.push_back(halfminoraxissize * SIM_WORLD_HEIGHT / REAL_WORLD_HEIGHT);
  params.push_back(angle);
  return params;
}

void World::paintQuad(QPainter &painter, Pose p)
{
  double r = QUAD_RADIUS;
  double l = HEADING_ARROW_LENGTH;
  double roll, pitch, yaw;
  common->quatToEuler(p.quat_w, p.quat_x, p.quat_y, p.quat_z, yaw, pitch, roll);
  double heading = yaw;
  QPointF p1 = realToSimCoord(QPointF(p.x, p.y));
  QPointF p2 = QPointF(p1.x() - l*sin(heading), p1.y() - l*cos(heading));
  painter.save();
  painter.setRenderHint(QPainter::Antialiasing);
  vector< double > covEllipseParams = ellipseFromCovMatrix(common->getQuadPose().covariance);
  painter.setPen(Qt::NoPen);
  QColor green50 = Qt::green;
  green50.setAlphaF(0.6);
  painter.setBrush(green50);
  painter.translate(p1.x(), p1.y());
  painter.rotate(covEllipseParams[2]);
  painter.drawEllipse(QPointF(0,0), covEllipseParams[0], covEllipseParams[1]);
  painter.rotate(-covEllipseParams[2]);
  painter.translate(-p1.x(), -p1.y());
  painter.setBrush(QBrush(Qt::yellow));
  painter.setPen(Qt::black);
  painter.drawEllipse(p1, r, r);
  painter.drawLine(p1, p2);
  painter.restore();
}

void World::paintGBot(QPainter &painter, Pose p)
{
  double r = GROUND_BOT_RADIUS;
  double l = HEADING_ARROW_LENGTH;
  double roll, pitch, yaw;
  common->quatToEuler(p.quat_w, p.quat_x, p.quat_y, p.quat_z, yaw, pitch, roll);
  double heading = yaw;
  QPointF p1 = realToSimCoord(QPointF(p.x, p.y));
  QPointF p2 = QPointF(p1.x() - l*sin(heading), p1.y() - l*cos(heading));
  painter.save();
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setBrush(QBrush(Qt::white));
  painter.drawEllipse(p1, r, r);
  painter.setBrush(QBrush(Qt::red));
  painter.drawEllipse(p1, r/2, r/2);
  painter.setPen(Qt::black);
  painter.drawLine(p1, p2);
  painter.restore();
}

void World::paintObstacleBot(QPainter &painter, Pose p)
{
  double r = GROUND_BOT_RADIUS;
  double l = HEADING_ARROW_LENGTH;
  double roll, pitch, yaw;
  common->quatToEuler(p.quat_w, p.quat_x, p.quat_y, p.quat_z, yaw, pitch, roll);
  double heading = yaw;
  QPointF p1 = realToSimCoord(QPointF(p.x, p.y));
  QPointF p2 = QPointF(p1.x() - l*sin(heading), p1.y() - l*cos(heading));
  painter.save();
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setBrush(QBrush(Qt::black));
  painter.drawEllipse(p1, r, r);
  painter.setPen(Qt::black);
  painter.drawLine(p1, p2);
  painter.restore();
}

void World::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  paintField(painter);
  paintQuad(painter, common->getQuadPose());
  paintObstacleBot(painter, common->getgBot0Pose());
  paintObstacleBot(painter, common->getgBot1Pose());
  paintGBot(painter, common->getgBot5Pose());
  emit painting();
}

void World::updateGBotPose()
{
  update();
}
