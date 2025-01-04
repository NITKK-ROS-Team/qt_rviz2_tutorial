/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "widget/widget.hpp"

namespace rviz2panel_action_client
{

ExampleWidget::ExampleWidget(QWidget * parent = nullptr)
: rviz_common::Panel(parent)
{
  // UIの初期化
  ui.setupUi(this);
  // ボタンがクリックされたときの処理を設定
  connect(ui.example_push_button, &QPushButton::clicked, this, &ExampleWidget::onPushButtonClicked);
}

void ExampleWidget::onInitialize()
{
  qt_node_action_client_handler_.setRosNodePtr(
    this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node());
  qt_node_action_client_handler_.initializeActionClient("fibonacci");

  // タイマーの設定 30hz
  timer_.setInterval(1000 / 30);
  timer_.start();
  connect(&timer_, &QTimer::timeout, this, &ExampleWidget::onTimer);
}

void ExampleWidget::onPushButtonClicked()
{
  static uint32_t order = 0;
  // アクションゴールの作成
  example_interfaces::action::Fibonacci::Goal goal;
  goal.order = order;
  // アクションゴールの送信
  qt_node_action_client_handler_.sendGoal(goal);
  order++;
}

void ExampleWidget::onTimer()
{
  std::shared_ptr<const example_interfaces::action::Fibonacci::Result> msg_goal;
  std::shared_ptr<const example_interfaces::action::Fibonacci::Feedback> msg_feedback;

  if (qt_node_action_client_handler_.getResult(msg_goal)) {
    // メッセージの表示
    std::string str = "Result: (" + std::to_string(msg_goal->sequence.size()) + ") = [";
    for (size_t i = 0; i < msg_goal->sequence.size(); i++) {
      str += std::to_string(msg_goal->sequence[i]);
      if (i != msg_goal->sequence.size() - 1) {
        str += ", ";
      }
    }
    str += "]";
    ui.result_label->setText(QString::fromStdString(str));
    std::cout << str << std::endl;

  } else if (qt_node_action_client_handler_.getLatestFeedback(msg_feedback)) {
    std::string str = "Feedback: (" + std::to_string(msg_feedback->sequence.size()) + ") = [";
    for (size_t i = 0; i < msg_feedback->sequence.size(); i++) {
      str += std::to_string(msg_feedback->sequence[i]);
      if (i != msg_feedback->sequence.size() - 1) {
        str += ", ";
      }
    }
    str += "]";
    ui.feedback_label->setText(QString::fromStdString(str));
    std::cout << str << std::endl;
  }
}

} // namespace rviz2panel_action_client

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz2panel_action_client::ExampleWidget, rviz_common::Panel)
