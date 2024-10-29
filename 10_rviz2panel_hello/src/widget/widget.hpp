/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RVIZ2PANEL_HELLO_WIDGET_HPP
#define RVIZ2PANEL_HELLO_WIDGET_HPP

// 作成した.uiファイル名によって変わる
// xxx.ui -> ui_xxx.h
#include "ui_rviz2panel_hello.h"

// Q_MOC_RUN is defined when this file is processed by moc
#ifndef Q_MOC_RUN
#include <rviz_common/panel.hpp>

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#endif

namespace rviz2panel_hello
{

// Widgetのクラス
class ExampleWidget : public rviz_common::Panel
{
public:
  // 初期化
  explicit ExampleWidget(QWidget *);

public:
// rviz_commonのAPI : デフォルトの実装は空なので、必要に応じてオーバーライドする

private:
  // ボタンがクリックされたときの処理
  void onPushButtonClicked();

private:
  Ui::ExampleWidget ui;
};

} // namespace rviz2panel_hello

#endif //RVIZ2PANEL_HELLO_WIDGET_HPP
