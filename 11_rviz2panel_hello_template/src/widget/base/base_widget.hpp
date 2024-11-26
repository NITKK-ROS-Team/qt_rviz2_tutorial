/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RVIZ2PANEL_HELLO_TEMPLATE_BASE_WIDGET_HPP_
#define RVIZ2PANEL_HELLO_TEMPLATE_BASE_WIDGET_HPP_

// 作成した.uiファイル名によって変わる
// xxx.ui -> ui_xxx.h
#include "ui_rviz2panel_hello_template.h"

// Q_MOC_RUN is defined when this file is processed by moc
#ifndef Q_MOC_RUN
#include <rviz_common/panel.hpp>

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>
#endif

#include <QObject>
#include <QWidget>


namespace rviz2panel_hello_template
{

// Widgetのクラス
template <class T>
class BaseExampleWidget : public T
{
public:
  // 初期化
  explicit BaseExampleWidget(QWidget *);

private:
  // ボタンがクリックされたときの処理
  void onPushButtonClicked();

private:
  Ui::ExampleWidget ui;
};

} // namespace rviz2panel_hello_template

#endif //RVIZ2PANEL_HELLO_TEMPLATE_BASE_WIDGET_HPP_
