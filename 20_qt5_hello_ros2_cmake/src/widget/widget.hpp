/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef QT5_HELLO_ROS2_CMAKE_WIDGET_WIDGET_HPP_
#define QT5_HELLO_ROS2_CMAKE_WIDGET_WIDGET_HPP_

// 作成した.uiファイル名によって変わる
// xxx.ui -> ui_xxx.h
#include "ui_qt5_hello_ros2_cmake.h"

namespace qt5_hello_ros2_cmake
{

// Widgetのクラス
class ExampleWidget : public QWidget
{
public:
  // 初期化
  explicit ExampleWidget(QWidget *);

private:
  // ボタンがクリックされたときの処理
  void onPushButtonClicked();

private:
  Ui::ExampleWidget ui;
};

} // namespace qt5_hello_ros2_cmake

#endif //qt5_hello_ros2_cmake_WIDGET_WIDGET_HPP_
