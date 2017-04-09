#ifndef WORLD_H
#define WORLD_H

#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QDebug>
#include <vector>
#include <opencv2/opencv.hpp>
#include "common.h"
#include "drawing_constants.h"

using namespace cv;

class World : public QWidget
{
  Q_OBJECT
public:
  Common *common;

  explicit World(QWidget *parent = 0);
private:
  QPointF realToSimCoord(QPointF coord);
  vector< double > ellipseFromCovMatrix(double* covariance);
protected:
  void paintEvent(QPaintEvent *event) override;
  void paintField(QPainter& painter);
  void paintQuad(QPainter& painter, Pose p);
  void paintObstacleBot(QPainter& painter, Pose p);
  void paintGBot(QPainter& painter, Pose p);
signals:
  void painting();
public slots:
  void updateGBotPose();
};

#endif // WORLD_H
