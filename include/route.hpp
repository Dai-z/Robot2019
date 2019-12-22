#pragma once

#include "item.hpp"

class Route : public Item {
 public:
  /**
   * @brief Route constructor
   *
   * @param parent - parent of this item
   */
  Route(QGraphicsItem* parent = 0);

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
   * @brief Draw route post point
   *
   * @param painter - painter
   * @param x - position x of route point
   * @param y - position y of route point
   */
  void drawRoute(QPainter* painter, qreal x, qreal y);
};
