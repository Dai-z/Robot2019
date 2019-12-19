
#include "control_widget.hpp"
#include <QDebug>
#include <QLabel>
#include <QMainWindow>
#include <QRect>
#include <QTextEdit>
#include "model.hpp"

ControlWidget::ControlWidget(QWidget* parent) : QWidget(parent) {
  auto layout = new QVBoxLayout();
  auto vlayout = new QVBoxLayout();

  // set up view range checker
  {
    show_view_range_ = new QCheckBox();
    auto hlayout = new QHBoxLayout();
    QLabel* txt = new QLabel("viewRange");
    hlayout->addWidget(txt);
    hlayout->addStretch();
    hlayout->addWidget(show_view_range_);
    vlayout->addLayout(hlayout);

  }

  // set up particle checker
  {
    show_particle_ = new QCheckBox();
    auto hlayout = new QHBoxLayout();
    QLabel* txt = new QLabel("Particles");
    hlayout->addWidget(txt);
    hlayout->addStretch();
    hlayout->addWidget(show_particle_);
    vlayout->addLayout(hlayout);
  }

  layout->addLayout(vlayout);
  this->setLayout(layout);
}
