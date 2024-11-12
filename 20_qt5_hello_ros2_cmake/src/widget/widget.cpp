/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "widget/widget.hpp"

namespace qt5_hello_ros2_cmake
{

ExampleWidget::ExampleWidget(QWidget * parent = nullptr)
: QWidget(parent)
{
  // UIの初期化
  ui.setupUi(this);
  // ボタンがクリックされたときの処理を設定
  connect(ui.example_push_button, &QPushButton::clicked, this, &ExampleWidget::onPushButtonClicked);
}

void ExampleWidget::onPushButtonClicked()
{
  static uint32_t counter = 0;
  // カウンタをインクリメントしてラベルに表示
  ui.example_label->setText(QString("%1").arg(++counter));
}

} // namespace qt5_hello_ros2_cmake
