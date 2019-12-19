#include "field.hpp"

FieldItem::FieldItem(QGraphicsItem* parent)
  : Item(parent)
{
    this->setFlag(QGraphicsItem::ItemIsMovable, false);
    setVisible(true);
}

FieldItem::~FieldItem()
{
}

QRectF
FieldItem::boundingRect() const
{
    return QRectF(0, 0, 0, 0);
}

void
FieldItem::myPaint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget)
{
    auto blackpen = QPen(Qt::white, 3);

    int w = 900;
    int h = 600;
    int e = 100;
    int f = 500;
    int r = 150;
    int c = 60;
    int d = 260;
    int g = 150;
    float lw = 5;
    int w_2 = w / 2; int h_2 = h / 2;

    QPen pen(Qt::white, lw);
    painter->setPen(pen);
    painter->drawRect(-w_2, -h_2, w, h);

    // goal area
    painter->drawRect(-w_2, -f / 2, e, f);
    painter->drawRect(w_2 - e, -f / 2, e, f);

    // center line
    painter->drawLine(0, h / 2, 0, -h / 2);
    // goal
    painter->drawRect(-w_2 - c, -d / 2, c, d);
    painter->drawRect(w_2, -d / 2, c, d);

    // center circle
    QRectF circle(-r / 2, -r / 2, r, r);
    painter->drawEllipse(circle);

    // penalty
    int p = -w_2 + g;
    painter->drawLine(p - lw, 0, p + lw, 0);
    painter->drawLine(p, lw, p, -lw);

    painter->drawLine(-p + lw, 0, -p - lw, 0);
    painter->drawLine(-p, lw, -p, -lw);
}