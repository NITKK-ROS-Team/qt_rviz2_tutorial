/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef QT5_LAYOUT_WIDGET_WIDGET_HPP
#define QT5_LAYOUT_WIDGET_WIDGET_HPP

// 作成した.uiファイル名によって変わる
// xxx.ui -> ui_xxx.h
#include "ui_qt5_layout.h"

namespace qt5_layout
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

} // namespace qt5_layout

#endif //QT5_LAYOUT_WIDGET_WIDGET_HPP
