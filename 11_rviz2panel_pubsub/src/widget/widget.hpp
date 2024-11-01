/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef rviz2panel_pubsub_WIDGET_HPP_
#define rviz2panel_pubsub_WIDGET_HPP_

// 作成した.uiファイル名によって変わる
// xxx.ui -> ui_xxx.h
#include "ui_rviz2panel_pubsub.h"
#include <QTimer>

// Q_MOC_RUN is defined when this file is processed by moc
#ifndef Q_MOC_RUN
#include <rviz_common/panel.hpp>

#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

#include "rviz2panel_pubsub/qt_node_pub_handler.hpp"
#include "rviz2panel_pubsub/qt_node_sub_handler.hpp"
#endif

namespace rviz2panel_pubsub
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

  // パブリッシャのハンドラ
  QtNodePubHandler qt_node_pub_handler_;
  QtNodeSubHandler qt_node_sub_handler_;
};

} // namespace rviz2panel_pubsub

#endif //rviz2panel_pubsub_WIDGET_HPP_
