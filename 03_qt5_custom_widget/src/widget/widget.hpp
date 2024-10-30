/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef QT5_CUSTOM_WIDGET_WIDGET_HPP_
#define QT5_CUSTOM_WIDGET_WIDGET_HPP_

// 作成した.uiファイル名によって変わる
// xxx.ui -> ui_xxx.h
#include "ui_qt5_custom_widget.h"

namespace qt5_custom_widget
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

} // namespace qt5_custom_widget

#endif //QT5_CUSTOM_WIDGET_WIDGET_HPP_
