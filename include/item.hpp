#pragma once

#include <QGraphicsItem>
#include <QPainter>
#include <QTimer>
#include <QVector3D>
#include <QtCore>
#include "model.hpp"

class Item : public QGraphicsItem {
 public:
  /**
   * @brief Item constructor
   *
   * @param parent - parent of this instance
   */
  Item(QGraphicsItem* parent = 0);
  /**
   * @brief Item destructor
   */
  virtual ~Item();

  // Getter
  /**
   * @brief Get bounding rect of item
   *
   * @return bounding rect of item
   */
  QRectF boundingRect() const;

  /**
   * @brief Get position y of item
   *
   * @return position y of item
   */
  qreal y() const;
  /**
   * @brief Get position and angle
   *
   * @return position and angle of item
   */
  QVector3D pos() const;
  /**
   * @brief Get angle of item
   *
   * @return angle of item
   */
  qreal angle() const;

  // Setter
  /**
   * @brief Set position y of item
   *
   * @param y - desired position y
   */
  void setY(qreal y);
  /**
   * @brief Set position of item
   *
   * @param x - desired position x
   * @param y - desired position y
   */
  void setPos(qreal x, qreal y);
  /**
   * @brief Set position of item
   *
   * @param QPointF - desired position
   */
  void setPos(QPointF);
  /**
   * @brief Set position of item
   *
   * @param pos - desired position and angle
   */
  void setPos(QVector3D pos);

  /**
   * @brief paint - paint function
   *
   * @param painter - painter
   * @param option - option
   * @param widget - widget
   */
  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
             QWidget* widget);

  /**
   * @brief Flip y axis of painter
   */
  void flipPainterYAxis();

  /**
   * @brief Custom painter function
   *
   * @param painter - painter
   * @param option - option
   * @param widget - widget
   */
  virtual void myPaint(QPainter* painter,
                       const QStyleOptionGraphicsItem* option, QWidget* widget);

  /**
   * @brief Draw text on given position
   *
   * @param x - position x of text to draw
   * @param y - position y of text to draw
   * @param str - text to draw
   */
  void drawText(qreal x, qreal y, QString str);

  /**
   * @brief Draw text on given position
   *
   * @param p - position of text to draw
   * @param str - text to draw
   */
  void drawText(QPointF p, QString str);

  /**
   * @brief Set angle of item
   *
   * @param r - desired angle
   */
  void setRotation(qreal r);
  /**
   * @brief Rotate item with given angle
   *
   * @param r - desired angle in degree
   */
  void rotateDegree(qreal r);
  /**
   * @brief Rotate item with given angle
   *
   * @param r - desired angle in radian
   */
  void rotateRadian(qreal r);

 protected:
  /**
   * @brief Event on mouse press
   *
   * @param event - triggered event
   */
  void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
  /**
   * @brief Event on mouse release
   *
   * @param event - triggered event
   */
  void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;
  /**
   * @brief Event on mouse move
   *
   * @param event - triggered event
   */
  void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;

  /**
   * @brief Event on key press
   *
   * @param event - triggered event
   */
  void keyPressEvent(QKeyEvent* event) override;
  /**
   * @brief Event on key release
   *
   * @param event - triggered event
   */
  void keyReleaseEvent(QKeyEvent* event) override;

  /**
   * @brief Set painter
   *
   * @param p - given painter
   */
  void setPainter(QPainter* p);

  //! Robot model
  Model* model_;
  //! Painter
  QPainter* painter_;

  //! Angle of item
  qreal angle_ = 0;
  //! Flag for whether item is pressed by mouse
  bool pressed_ = false;
  //! Flag for whether rotation is enabled for this item
  bool rotate_enabled_ = false;
  //! Position of item when mouse is pressed
  QPointF pos_start_press_;
  //! Angle of item when mouse is pressed
  qreal angle_start_press_ = 0;
  //! Slope of item position and pressed position, used for rotation
  qreal angle_start_press_2_ = 0;
};