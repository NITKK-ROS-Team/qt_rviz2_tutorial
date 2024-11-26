/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RVIZ2PANEL_HELLO_TEMPLATE_MAINWINDOW_MAINWINDOW_HPP_
#define RVIZ2PANEL_HELLO_TEMPLATE_MAINWINDOW_MAINWINDOW_HPP_


#include <QMainWindow>
#include <QWidget>
#include "widget/qt_widget.hpp"

namespace rviz2panel_hello_template
{

class ExampleWindow : public QMainWindow
{
public:
  explicit ExampleWindow(QWidget * = nullptr);
};

} // namespace rviz2panel_hello_template

#endif //RVIZ2PANEL_HELLO_TEMPLATE_MAINWINDOW_MAINWINDOW_HPP_
