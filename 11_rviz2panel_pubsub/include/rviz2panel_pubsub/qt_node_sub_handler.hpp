// Copyright 2024 StrayedCats.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <queue>

namespace rviz2panel_pubsub
{

class QtNodeSubHandler
{
public:
  QtNodeSubHandler(void) {}

  void setRosNodePtr(const rclcpp::Node::SharedPtr node_ptr)
  {
    node_ptr_ = node_ptr;
  }

  void initializeSubscription(const std::string topic_name)
  {
    qt_node_subscription_ =
        node_ptr_->create_subscription<std_msgs::msg::String>(
            topic_name, 10, std::bind(&QtNodeSubHandler::onMsgReceived, this, std::placeholders::_1));
  }

  void finalizeSubscription(void)
  {
    qt_node_subscription_.reset();
  }

  void onMsgReceived(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(node_ptr_->get_logger(), "I heard: '%s'", msg->data.c_str());
    msg_queue_.push(msg);
  }

  bool getMsg(std_msgs::msg::String::SharedPtr &msg)
  {
    if (msg_queue_.empty())
    {
      return false;
    }

    msg = msg_queue_.front();
    msg_queue_.pop();
    return true;
  }

  bool getLatestMsg(std_msgs::msg::String::SharedPtr &msg)
  {
    if (msg_queue_.empty())
    {
      return false;
    }

    msg = msg_queue_.back();
    // clean
    while (!msg_queue_.empty())
    {
      msg_queue_.pop();
    }
    return true;
  }


private:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qt_node_subscription_;

  std::queue<std_msgs::msg::String::SharedPtr> msg_queue_;

};

} // namespace rviz2panel_pubsub
