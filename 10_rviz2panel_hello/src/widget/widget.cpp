/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "widget/widget.hpp"

namespace rviz2panel_hello
{

ExampleWidget::ExampleWidget(QWidget * parent = nullptr)
: rviz_common::Panel(parent)
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

} // namespace rviz2panel_hello

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2panel_hello::ExampleWidget, rviz_common::Panel)
