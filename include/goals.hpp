#pragma once

#include "item.hpp"

class Goals : public Item {
 public:
  /**
   * @brief Goals constructor
   *
   * @param parent - parent of this item
   */
  Goals(QGraphicsItem* parent = 0);

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
   * @brief Draw goal post point
   *
   * @param painter - painter
   * @param x - position x of goal post point
   * @param y - position y of goal post point
   */
  void drawGoal(QPainter* painter, qreal x, qreal y);
};
