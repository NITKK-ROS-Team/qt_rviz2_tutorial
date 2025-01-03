/*
 * Copyright (c) 2024 NITK.K ROS-Team
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RVIZ2PANEL_PUBSUB_QT_NODE_SUB_HANDLER_HPP_
#define RVIZ2PANEL_PUBSUB_QT_NODE_SUB_HANDLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <queue>
#include "ros_types.hpp"

namespace rviz2panel_pubsub
{

template<typename T>
class QtNodeSubHandler
{
public:
  QtNodeSubHandler() {}

  void setRosNodePtr(const rclcpp::Node::SharedPtr node_ptr)
  {
    node_ptr_ = node_ptr;
  }

  void initializeSubscription(const std::string topic_name, const uint16_t msg_queue_size = 10)
  {
    msg_queue_size_ = msg_queue_size;

    qt_node_subscription_ = node_ptr_->create_subscription<T>(
      topic_name, 1, std::bind(&QtNodeSubHandler::onMsgReceived, this, std::placeholders::_1));
  }

  void finalizeSubscription()
  {
    qt_node_subscription_.reset();
  }

  void onMsgReceived(const typename T::SharedPtr msg)
  {
    msg_queue_.push(msg);

    while (msg_queue_.size() > msg_queue_size_) {
      msg_queue_.pop();
    }
  }

  bool getMsg(typename T::SharedPtr & msg)
  {
    if (msg_queue_.empty()) {
      return false;
    }

    msg = msg_queue_.front();
    msg_queue_.pop();
    return true;
  }

  bool getLatestMsg(typename T::SharedPtr & msg)
  {
    if (msg_queue_.empty()) {
      return false;
    }

    msg = msg_queue_.back();
    // キューをクリア
    while (!msg_queue_.empty()) {
      msg_queue_.pop();
    }
    return true;
  }

private:
  rclcpp::Node::SharedPtr node_ptr_;
  typename rclcpp::Subscription<T>::SharedPtr qt_node_subscription_;
  std::queue<typename T::SharedPtr> msg_queue_;
  uint16_t msg_queue_size_;
};

} // namespace rviz2panel_pubsub

#endif // RVIZ2PANEL_PUBSUB_QT_NODE_SUB_HANDLER_HPP_
