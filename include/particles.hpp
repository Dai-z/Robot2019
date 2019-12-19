
#pragma once

#include "item.hpp"

class Particles : public Item
{
  public:
    /**
     * @brief Particles constructor
     *
     * @param parent - parent of this item
     */
    Particles(QGraphicsItem* parent = 0);

    /**
     * @brief Custom painter function
     *
     * @param painter - painter
     * @param option - option
     * @param widget - widget
     */
    void myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);

    /**
     * @brief Draw particles
     *
     * @param painter - painter
     * @param x - position x of particle
     * @param y - position y of particle
     * @param angle - angle of particle
     * @param weight - wieght of particle
     */
    void drawP(QPainter* painter, qreal x, qreal y, qreal angle, qreal weight);

};
