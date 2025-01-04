#ifndef RVIZ2PANEL_ACTION_CLIENT_HANDLER_HPP_
#define RVIZ2PANEL_ACTION_CLIENT_HANDLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <queue>
#include <memory>
#include "ros_types.hpp"


namespace rviz2panel_action_client
{
template<typename ActionT>
class QtNodeActionClientHandler
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  using Result = typename ActionT::Result;
  using Feedback = typename ActionT::Feedback;

  QtNodeActionClientHandler() = default;

  void setRosNodePtr(const rclcpp::Node::SharedPtr & node_ptr)
  {
    node_ptr_ = node_ptr;
  }

  void initializeActionClient(const std::string & action_name)
  {
    action_client_ = rclcpp_action::create_client<ActionT>(node_ptr_, action_name);
  }

  bool waitForServer(const std::chrono::seconds & timeout = std::chrono::seconds(10))
  {
    return action_client_->wait_for_action_server(timeout);
  }

  void resultCallback(const typename GoalHandle::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      result_queue_.push(result.result);
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "QtNodeActionClientHandler"), "Goal failed with code %d", static_cast<int>(result.code));
    }
  }

  void feedbackCallback(
    typename GoalHandle::SharedPtr,
    const std::shared_ptr<const Feedback> feedback)
  {
    feedback_queue_.push(feedback);
  }

  void sendGoal(const typename ActionT::Goal & goal_msg)
  {
    if (!action_client_) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Action client not initialized");
      return;
    }

    typename rclcpp_action::Client<ActionT>::SendGoalOptions options;

    options.result_callback = std::bind(
      &QtNodeActionClientHandler::resultCallback, this,
      std::placeholders::_1);
    options.feedback_callback = std::bind(
      &QtNodeActionClientHandler::feedbackCallback, this,
      std::placeholders::_1, std::placeholders::_2);


    action_client_->async_send_goal(goal_msg, options);
  }

  bool getResult(std::shared_ptr<const Result> & result)
  {
    if (result_queue_.empty()) {
      return false;
    }
    result = result_queue_.front();
    result_queue_.pop();
    return true;
  }

  bool getLatestFeedback(std::shared_ptr<const Feedback> & feedback)
  {
    if (feedback_queue_.empty()) {
      return false;
    }
    feedback = feedback_queue_.back();
    feedback_queue_ = {}; // Clear the queue
    return true;
  }

private:
  rclcpp::Node::SharedPtr node_ptr_;
  typename rclcpp_action::Client<ActionT>::SharedPtr action_client_;
  std::queue<std::shared_ptr<const Result>> result_queue_;
  std::queue<std::shared_ptr<const Feedback>> feedback_queue_;
};

}  // namespace rviz2panel_action_client

#endif  // RVIZ2PANEL_ACTION_CLIENT_HANDLER_HPP_
