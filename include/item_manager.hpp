
#pragma once

#include <QGraphicsScene>
#include <QObject>

/**
 * @brief Manager for items in the soccer field
 */
class ControlWidget;
class ItemManager : public QObject {
  Q_OBJECT
 public:
  /**
   * @brief ItemManager constructor
   *
   * @param parent - parent of item manager
   * @param scene - scene for soccer field
   */
  ItemManager(QObject* parent, QGraphicsScene* scene, ControlWidget* control);
  /**
   * @brief ItemManager destructor
   */
  ~ItemManager();
  /**
   * @brief Init item manager
   */
  void Init();

 private:
  //! counter for looping cycle
  int cycle_ = 0;
  //! scene for soccer field
  QGraphicsScene* scene_;
  //! control panel
  ControlWidget* control_;
};
