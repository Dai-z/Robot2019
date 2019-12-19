#pragma once

#include "item.hpp"

class Robot : public Item {
 public:
  /**
   * @brief Robot constructor
   *
   * @param robot_id - robot id
   * @param loc - whether is from localization information
   * @param parent - parent of this item
   */
  Robot(bool loc, QGraphicsItem* parent = 0);
  /**
   * @brief Get bounding rectangle of ball
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
   * @brief Paint localization robot position
   *
   * @param painter - painter
   */
  void locPaint(QPainter* painter);
  /**
   * @brief Paint ground truth robot position in simulation mode
   *
   * @param painter - painter
   */
  void realPaint(QPainter* painter);

  /**
   * @brief Event on mouse press
   *
   * @param event - triggered event
   */
  void mouseMoveEvent(QGraphicsSceneMouseEvent* event);
  /**
   * @brief Event on mouse release
   *
   * @param event - triggered event
   */
  void mouseReleaseEvent(QGraphicsSceneMouseEvent* event);
  /**
   * @brief Event on mouse move
   *
   * @param event - triggered event
   */
  void mousePressEvent(QGraphicsSceneMouseEvent* event);

 private:
  //! whether this robot is from localization information
  bool loc_ = false;
  //! color of robot
  QColor color_;
  //! Flag whether robot is selected
  bool selected_ = false;
};