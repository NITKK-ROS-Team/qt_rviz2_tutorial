/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef QT5_HELLO_ROS2_CMAKE_MAINWINDOW_MAINWINDOW_HPP_
#define QT5_HELLO_ROS2_CMAKE_MAINWINDOW_MAINWINDOW_HPP_

#include <QMainWindow>
#include "widget/widget.hpp"

namespace qt5_hello_ros2_cmake
{

class ExampleWindow : public QMainWindow
{
public:
  explicit ExampleWindow(QWidget * = nullptr);
};

} // namespace qt5_hello_ros2_cmake

#endif // QT5_HELLO_ROS2_CMAKE_MAINWINDOW_MAINWINDOW_HPP_
