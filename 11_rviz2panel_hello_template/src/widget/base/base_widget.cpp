/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "widget/base/base_widget.hpp"

namespace rviz2panel_hello_template
{

template<class T>
BaseExampleWidget<T>::BaseExampleWidget(QWidget * parent)
: T(parent)
{
  // UIの初期化
  ui.setupUi(this);
  // ボタンがクリックされたときの処理を設定
  this->connect(
    ui.example_push_button, &QPushButton::clicked, this,
    &BaseExampleWidget::onPushButtonClicked);
}

template<class T>
void BaseExampleWidget<T>::onPushButtonClicked()
{
  static uint32_t counter = 0;
  // カウンタをインクリメントしてラベルに表示
  ui.example_label->setText(QString("%1").arg(++counter));
}

// エラー回避のため、明示的なインスタンス化
template class BaseExampleWidget<QWidget>;
template class BaseExampleWidget<rviz_common::Panel>;

} // namespace rviz2panel_hello_template
