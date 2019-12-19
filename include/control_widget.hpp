#pragma once

#include "render.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QRadioButton>
#include <QVBoxLayout>
#include <QWidget>
#include <vector>

/**
 * @brief Control widget
 */
class ControlWidget : public QWidget
{
    Q_OBJECT
  public:
    /**
     * @brief ControlWidget constructor
     *
     * @param parent - parent of QWidget
     */
    ControlWidget(QWidget* parent = 0);

    //! Checkboxes for whether to show view range of robot
    QCheckBox* show_view_range_;
    //! Checkboxes for whether to show particles from AMCL in robots
    QCheckBox* show_particle_;

  private:
};
