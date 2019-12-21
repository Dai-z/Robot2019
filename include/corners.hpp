#pragma once

#include "item.hpp"

class Corners : public Item {
 public:
  /**
   * @brief Corners constructor
   *
   * @param parent - parent of this item
   */
  Corners(QGraphicsItem* parent = 0);

  /**
   * @brief Custom painter function
   *
   * @param painter - painter
   * @param option - option
   * @param widget - widget
   */
  void myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option,
               QWidget* widget);

  /**
   * @brief Draw corner point
   *
   * @param painter - painter
   * @param x - position x of corner point
   * @param y - position y of corner point
   */
  void drawCorner(QPainter* painter, qreal x, qreal y, QString type);
};
