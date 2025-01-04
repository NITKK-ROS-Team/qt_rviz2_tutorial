/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RVIZ2PANEL_ACTION_CLIENT_WIDGET_HPP_
#define RVIZ2PANEL_ACTION_CLIENT_WIDGET_HPP_

// 作成した.uiファイル名によって変わる
// xxx.ui -> ui_xxx.h
#include "ui_rviz2panel_action_client.h"
#include <QTimer>

// Q_MOC_RUN is defined when this file is processed by moc
#ifndef Q_MOC_RUN
#include <rviz_common/panel.hpp>

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

#include "rviz2panel_action_client/qt_node_action_client_handler.hpp"
#endif

namespace rviz2panel_action_client
{

// Widgetのクラス
class ExampleWidget : public rviz_common::Panel
{
public:
  // 初期化
  explicit ExampleWidget(QWidget *);

public:
// rviz_commonのAPI : デフォルトの実装は空なので、必要に応じてオーバーライドする
  void onInitialize() override;
  void load(const rviz_common::Config & config) override {}
  void save(rviz_common::Config config) const override {}

private:
  // ボタンがクリックされたときの処理
  void onPushButtonClicked();
  void onTimer();

private:
  QTimer timer_;
  Ui::ExampleWidget ui;

  QtNodeActionClientHandler<example_interfaces::action::Fibonacci> qt_node_action_client_handler_;
};

} // namespace rviz2panel_action_client

#endif //RVIZ2PANEL_ACTION_CLIENT_WIDGET_HPP_
