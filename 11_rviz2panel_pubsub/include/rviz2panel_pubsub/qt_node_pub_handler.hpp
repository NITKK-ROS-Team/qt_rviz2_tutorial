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


namespace rviz2panel_pubsub
{

class QtNodePubHandler
{
public:
  QtNodePubHandler(void) {}

  void setRosNodePtr(const rclcpp::Node::SharedPtr node_ptr)
  {
    node_ptr_ = node_ptr;
  }

  void initializePublisher(const std::string topic_name)
  {
    qt_node_publisher_ =
      node_ptr_->create_publisher<std_msgs::msg::String>(topic_name, 10);
  }

  void finalizePublisher(void)
  {
    qt_node_publisher_.reset();
  }

  void publishMsg(const std_msgs::msg::String & msg)
  {
    qt_node_publisher_->publish(msg);
  }

private:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qt_node_publisher_;
  
};

} // namespace rviz2panel_pubsub
