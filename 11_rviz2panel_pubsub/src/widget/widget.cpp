/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "widget/widget.hpp"

namespace rviz2panel_pubsub
{

ExampleWidget::ExampleWidget(QWidget * parent = nullptr)
: rviz_common::Panel(parent)
{
  // UIの初期化
  ui.setupUi(this);
  // ボタンがクリックされたときの処理を設定
  connect(ui.example_push_button, &QPushButton::clicked, this, &ExampleWidget::onPushButtonClicked);
  // timer_ = new QTimer(this);
}

void ExampleWidget::onInitialize()
{
  // パブリッシャの初期化
  qt_node_pub_handler_.setRosNodePtr(this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node());
  qt_node_pub_handler_.initializePublisher("example_topic");

  // サブスクライバの初期化
  qt_node_sub_handler_.setRosNodePtr(this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node());
  qt_node_sub_handler_.initializeSubscription("example_topic");

  // タイマーの設定 30hz
  timer_.setInterval(1000 / 30);
  timer_.start();
  connect(&timer_, &QTimer::timeout, this, &ExampleWidget::onTimer);
}

void ExampleWidget::onPushButtonClicked()
{
  static uint32_t counter = 0;
  // カウンタをインクリメントしてラベルに表示
  // ui.example_label->setText(QString("%1").arg(++counter));

  counter++;
  // パブリッシャを使ってメッセージを送信
  std_msgs::msg::String msg;
  msg.data = std::to_string(counter);
  qt_node_pub_handler_.publishMsg(msg);
}

void ExampleWidget::onTimer()
{
  // サブスクライバを使ってメッセージを受信
  std_msgs::msg::String::SharedPtr msg;
  if (qt_node_sub_handler_.getMsg(msg))
  {
    // 受信したメッセージをラベルに表示
    ui.example_label->setText(QString::fromStdString(msg->data));
  }
}

} // namespace rviz2panel_pubsub

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2panel_pubsub::ExampleWidget, rviz_common::Panel)
