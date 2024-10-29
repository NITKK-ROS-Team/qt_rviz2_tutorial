/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mainwindow.hpp"

namespace qt5_layout
{

ExampleWindow::ExampleWindow(QWidget * parent)
: QMainWindow(parent)
{
  // タイトルの設定
  setWindowTitle("Hello, Qt5!");
  // サイズの設定
  resize(800, 600);

  // ウィジェットの設定（中央に配置して新規作成）
  setCentralWidget(new ExampleWidget(this));
}

} // namespace qt5_layout
