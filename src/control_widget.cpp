
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

  // Set up view range checker
  {
    show_view_range_ = new QCheckBox();
    show_view_range_->setChecked(true);
    auto hlayout = new QHBoxLayout();
    QLabel* txt = new QLabel("viewRange");
    hlayout->addWidget(txt);
    hlayout->addStretch();
    hlayout->addWidget(show_view_range_);
    vlayout->addLayout(hlayout);
  }

  // Set up particle checker
  {
    show_particle_ = new QCheckBox();
    show_particle_->setChecked(true);
    auto hlayout = new QHBoxLayout();
    QLabel* txt = new QLabel("Particles");
    hlayout->addWidget(txt);
    hlayout->addStretch();
    hlayout->addWidget(show_particle_);
    vlayout->addLayout(hlayout);
  }

  // Set up robot heading
  {
    heading_ = new QSlider(Qt::Horizontal);

    auto hlayout = new QHBoxLayout();
    QLabel* txt = new QLabel("Robot heading");
    heading_->setMinimum(-180);
    heading_->setMaximum(180);
    heading_->setValue(90);
    heading_->showNormal();
    vlayout->addStretch();
    vlayout->addWidget(txt);
    vlayout->addWidget(heading_);
    connect(heading_, &QSlider::valueChanged, &Model::setSimRobotYaw);
  }

  layout->addLayout(vlayout);
  this->setLayout(layout);
}
