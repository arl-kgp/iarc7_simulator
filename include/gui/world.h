#ifndef WORLD_H
#define WORLD_H

#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QDebug>
#include "common.h"
#include "drawing_constants.h"

class World : public QWidget
{
  Q_OBJECT
public:
  Common *common;

  explicit World(QWidget *parent = 0);
private:
  QPointF realToSimCoord(QPointF coord);
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
