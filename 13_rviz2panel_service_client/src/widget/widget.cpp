/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "widget/widget.hpp"

namespace rviz2panel_service_client
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
  // サービスクライアントの初期化
  qt_node_service_client_handler_.setRosNodePtr(
    this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node());
  qt_node_service_client_handler_.initializeClient("add_two_ints");

  // タイマーの設定 30hz
  timer_.setInterval(1000 / 30);
  timer_.start();
  connect(&timer_, &QTimer::timeout, this, &ExampleWidget::onTimer);
}

void ExampleWidget::onPushButtonClicked()
{
  static uint32_t counter = 0;
  // サービスリクエストの作成, a = counter, b = counter + 1
  auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  request->a = counter;
  request->b = counter + 1;
  // サービスリクエストの送信
  qt_node_service_client_handler_.sendRequest(request);
  counter++;
}

void ExampleWidget::onTimer()
{
  example_interfaces::srv::AddTwoInts::Response::SharedPtr msg;
  if (qt_node_service_client_handler_.getResponse(msg)) {
    // メッセージの表示
    int32_t sum = static_cast<int32_t>(msg->sum);
    std::string str = std::to_string((sum - 1) / 2) + " + " + std::to_string((sum - 1) / 2 + 1) +
      " = " + std::to_string(sum);
    ui.example_label->setText(QString::fromStdString(str));
  }
}

} // namespace rviz2panel_service_client

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2panel_service_client::ExampleWidget, rviz_common::Panel)
