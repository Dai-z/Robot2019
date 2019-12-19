#include "item.hpp"

/**
 * @brief Field item
 */
class FieldItem : public Item
{
  public:
    /**
     * @brief FieldItem constructor
     *
     * @param parent - parent of this item
     */
    FieldItem(QGraphicsItem* parent = 0);
    /**
     * @brief FieldItem destructor
     */
    ~FieldItem();

    /**
     * @brief Get bounding rect of item
     *
     * @return bounding rect of item
     */
    QRectF boundingRect() const;
    /**
     * @brief Custom painter function
     *
     * @param painter - painter
     * @param option - option
     * @param widget - widget
     */
    void myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget);
};