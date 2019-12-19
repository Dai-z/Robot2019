#include "render.hpp"
#include <QTimer>
#include <QtCore>

Render::Render(QObject* parent) : QGraphicsScene(parent) {
  // set up background color
  this->setBackgroundBrush(QColor(109, 178, 255));
  // draw soccer field
  int w = 1040;
  int h = 740;
  this->setSceneRect(-w / 2, -h / 2, w, h);
}

Render::~Render() {}