#pragma once

#include "item.hpp"

/**
 * @brief Circle item
 */
class Circle : public Item {
 public:
  /**
   * @brief Circle constructor
   *
   * @param parent - parent of this item
   */
  Circle(QGraphicsItem* parent = 0);

  /**
   * @brief Custom painter function
   *
   * @param painter - painter
   * @param option - option
   * @param widget - widget
   */
  void myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option,
               QWidget* widget);

 private:
};