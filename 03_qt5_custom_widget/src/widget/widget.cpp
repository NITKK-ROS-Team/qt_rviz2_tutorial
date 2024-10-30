/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "widget/widget.hpp"

namespace qt5_custom_widget
{

ExampleWidget::ExampleWidget(QWidget * parent = nullptr)
: QWidget(parent)
{
  // UIの初期化
  ui.setupUi(this);
  // ボタンがクリックされたときの処理を設定
  connect(ui.example_push_button, &QPushButton::clicked, this, &ExampleWidget::onPushButtonClicked);

  ui.example_label->reset(300);
}

void ExampleWidget::onPushButtonClicked()
{
  ui.example_label->reset(300);
}

} // namespace qt5_custom_widget
