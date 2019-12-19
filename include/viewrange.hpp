
#pragma once

#include "item.hpp"

static const float MAX_SEEN_DIST = 500;

class ViewRange : public Item {
 public:
  ViewRange(QGraphicsItem* parent = 0);

  /** * @brief Get bounding rectangle of ball
   *
   * @return bounding rectangle of ball
   */
  QRectF boundingRect() const;
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
   * @brief Check whether point is in view range
   *
   * @param x - point position x
   * @param y - point position y
   *
   * @return whether point is in view range
   */
  bool inView(float x, float y);

  void checkSimBallInView();
  void checkWhitePointsInView();
  void checkCirclePointInview();
  void checkGoalInview();
  // void checkSimObstacleInView();

 private:
  std::vector<QPointF> view_range_field_;
};
