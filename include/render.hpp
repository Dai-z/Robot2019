#pragma once
#include <QGraphicsScene>

/**
 * @brief View render
 */
class Render : public QGraphicsScene
{
    Q_OBJECT
  public:
    /**
     * @brief Render constructor
     *
     * @param parent - parent of QWidget
     */
    Render(QObject* parent = 0);
    /**
     * @brief Render destructor
     */
    ~Render();
};
