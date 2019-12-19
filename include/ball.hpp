
#pragma once

#include "item.hpp"

/**
  * @brief Ball item
  */
class Ball : public Item
{
  public:
    /**
     * @brief Ball constructor
     *
     * @param loc - whether or not this ball is from localization information
     * @param parent - parent of this instance
     */
    Ball(bool loc, QGraphicsItem* parent = 0);

    /**
     * @brief Get bounding rectangle of ball
     *
     * @return bounding rectangle of ball
     */
    QRectF boundingRect() const;

    /**
     * @brief Paint ball
     *
     * @param painter - painter
     * @param option - option
     * @param widget - widget
     */
    void myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);

    void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;

    void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;

    void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
  public slots:
    /**
     * @brief Callback on collision 
     */
    void onCollision();

  private:
    //! whether or not this ball is from localization information
    bool loc_;

    bool selected_;
};
